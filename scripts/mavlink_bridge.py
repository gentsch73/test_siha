#!/usr/bin/env python3
"""
mavlink_bridge.py — Düzenlenmiş MAVLink Köprüsü
TEKNOFEST 2026 Savaşan İHA Yarışması

pymavlink ↔ ROS2 köprüsü.
Bu sürümde eklenenler:
  - Otomatik ARM → GUIDED → TAKEOFF zinciri
  - Takeoff başarılı olduğunun kontrolü (irtifa > hedef * 0.9)
  - RC override timeout koruması
  - FBWA / GUIDED mod geçişleri

Yayınlar:
  /mavlink/telemetry  (std_msgs/String) — telemetri JSON

Abonelikler:
  /mavlink/cmd/arm, /mavlink/cmd/mode, /mavlink/cmd/takeoff
  /mavlink/cmd/land, /mavlink/cmd/goto, /mavlink/cmd/velocity
  /mavlink/cmd/yaw

Kullanım:
  python3 scripts/mavlink_bridge.py
  python3 scripts/mavlink_bridge.py --url udp:127.0.0.1:14550
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import threading
import argparse

try:
    from pymavlink import mavutil
    PYMAVLINK = True
except ImportError:
    PYMAVLINK = False


# ─── RC Override timeout (saniye) ──────────────────────────────────────────
RC_OVERRIDE_TIMEOUT_S = 2.0

# ─── Takeoff dizisi gecikme sabitleri ──────────────────────────────────────
MODE_CHANGE_DELAY_S = 0.5   # GUIDED mod aktif olduktan sonra bekleme
ARM_DELAY_S         = 1.0   # ARM komutu sonrası bekleme


class MavlinkBridge(Node):
    def __init__(self, url: str = 'udp:127.0.0.1:14550'):
        super().__init__('mavlink_bridge')
        self.url = url
        self.conn = None
        self.armed = False
        self.mode  = 'MANUAL'
        self.target_system    = 1
        self.target_component = 1

        # Telemetri önbelleği
        self.latest_telem: dict = {}
        self._takeoff_target_alt: float = 0.0
        self._takeoff_confirmed: bool   = False

        # RC override zamanlayıcısı
        self._rc_override_last: float = 0.0
        self._rc_override_active: bool = False

        # ── Yayıncılar ──
        self.pub_telemetry = self.create_publisher(String, '/mavlink/telemetry', 10)

        # ── Abonelikler ──
        self.create_subscription(String, '/mavlink/cmd/arm',      self.on_arm,      10)
        self.create_subscription(String, '/mavlink/cmd/mode',     self.on_mode,     10)
        self.create_subscription(String, '/mavlink/cmd/takeoff',  self.on_takeoff,  10)
        self.create_subscription(String, '/mavlink/cmd/land',     self.on_land,     10)
        self.create_subscription(String, '/mavlink/cmd/goto',     self.on_goto,     10)
        self.create_subscription(String, '/mavlink/cmd/velocity', self.on_velocity, 10)
        self.create_subscription(String, '/mavlink/cmd/yaw',      self.on_yaw,      10)

        # ── Bağlantı ──
        if PYMAVLINK:
            self.connect()
        else:
            self.get_logger().warn('pymavlink bulunamadı — simülasyon modunda')

        # ── Thread'ler & zamanlayıcılar ──
        self.running = True
        if PYMAVLINK and self.conn:
            self._start_threads()

        self.create_timer(1.0, self._send_heartbeat)
        self.create_timer(0.1, self._publish_telemetry)

        self.get_logger().info(f'MAVLink Bridge başlatıldı: {url}')

    def _start_threads(self):
        telem_t = threading.Thread(target=self._telemetry_loop, daemon=True)
        telem_t.start()

    def connect(self):
        try:
            self.conn = mavutil.mavlink_connection(self.url)
            self.conn.wait_heartbeat(timeout=10)
            self.target_system    = self.conn.target_system
            self.target_component = self.conn.target_component
            self.get_logger().info(
                f'SITL bağlantısı kuruldu: sys={self.target_system} '
                f'comp={self.target_component}')
        except Exception as e:
            self.get_logger().error(f'SITL bağlantı hatası: {e}')
            self.conn = None

    # ─── Heartbeat & Telemetri ──────────────────────────────────────────────

    def _send_heartbeat(self):
        if not self.conn:
            return
        try:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
        except Exception:
            pass

    def _telemetry_loop(self):
        while self.running:
            if not self.conn:
                time.sleep(1)
                continue
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                self._parse_msg(msg)
            except Exception:
                pass

    def _parse_msg(self, msg):
        mt = msg.get_type()

        if mt == 'GLOBAL_POSITION_INT':
            self.latest_telem['lat'] = msg.lat / 1e7
            self.latest_telem['lon'] = msg.lon / 1e7
            self.latest_telem['alt'] = msg.relative_alt / 1000.0
            self.latest_telem['vx']  = msg.vx / 100.0
            self.latest_telem['vy']  = msg.vy / 100.0
            self.latest_telem['vz']  = msg.vz / 100.0
            self.latest_telem['hdg'] = msg.hdg / 100.0

            # Takeoff onayı
            if (self._takeoff_target_alt > 0
                    and not self._takeoff_confirmed
                    and self.latest_telem['alt'] >= self._takeoff_target_alt * 0.9):
                self._takeoff_confirmed = True
                self.get_logger().info(
                    f'[TAKEOFF] Hedef irtifaya ulaşıldı: '
                    f'{self.latest_telem["alt"]:.1f}m')

        elif mt == 'ATTITUDE':
            self.latest_telem['roll']  = math.degrees(msg.roll)
            self.latest_telem['pitch'] = math.degrees(msg.pitch)
            self.latest_telem['yaw']   = math.degrees(msg.yaw)

        elif mt == 'VFR_HUD':
            self.latest_telem['airspeed']    = msg.airspeed
            self.latest_telem['groundspeed'] = msg.groundspeed
            self.latest_telem['heading']     = msg.heading
            self.latest_telem['throttle']    = msg.throttle
            self.latest_telem['alt_vfr']     = msg.alt

        elif mt == 'SYS_STATUS':
            self.latest_telem['battery'] = msg.battery_remaining

        elif mt == 'HEARTBEAT':
            self.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.latest_telem['armed'] = self.armed
            mode_map = mavutil.mode_mapping_bynumber(msg.type)
            if mode_map and msg.custom_mode in mode_map:
                self.mode = mode_map[msg.custom_mode]
            self.latest_telem['mode'] = self.mode

        elif mt == 'GPS_RAW_INT':
            self.latest_telem['gps_fix']    = msg.fix_type
            self.latest_telem['satellites'] = msg.satellites_visible

    def _publish_telemetry(self):
        if not self.latest_telem:
            return
        m = String()
        m.data = json.dumps(self.latest_telem)
        self.pub_telemetry.publish(m)

    # ─── Komut İşleyicileri ─────────────────────────────────────────────────

    def on_arm(self, msg: String):
        """ARM/DISARM: {"arm": true/false}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            arm_val = 1 if data.get('arm', True) else 0
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, arm_val, 0, 0, 0, 0, 0, 0)
            self.get_logger().info(f'{"ARM" if arm_val else "DISARM"} komutu gönderildi')
        except Exception as e:
            self.get_logger().error(f'ARM hatası: {e}')

    def on_mode(self, msg: String):
        """Mod değiştir: {"mode": "GUIDED"}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            mode_str = data.get('mode', 'MANUAL')
            mode_id  = self.conn.mode_mapping().get(mode_str)
            if mode_id is not None:
                self.conn.set_mode(mode_id)
                self.get_logger().info(f'Mod değiştirildi: {mode_str}')
            else:
                self.get_logger().warn(f'Bilinmeyen mod: {mode_str}')
        except Exception as e:
            self.get_logger().error(f'Mod hatası: {e}')

    def on_takeoff(self, msg: String):
        """
        Otomatik takeoff dizisi:
          GUIDED moda geç → ARM → TAKEOFF komutu
        {"altitude": 30.0}
        """
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            alt  = float(data.get('altitude', 30.0))

            self._takeoff_target_alt = alt
            self._takeoff_confirmed  = False

            # Adım 1: GUIDED moda geç
            guided_id = self.conn.mode_mapping().get('GUIDED')
            if guided_id is not None:
                self.conn.set_mode(guided_id)
                self.get_logger().info('[TAKEOFF] GUIDED mod ayarlandı')
                time.sleep(MODE_CHANGE_DELAY_S)

            # Adım 2: ARM
            if not self.armed:
                self.conn.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0)
                self.get_logger().info('[TAKEOFF] ARM komutu gönderildi')
                time.sleep(ARM_DELAY_S)

            # Adım 3: TAKEOFF
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt)
            self.get_logger().info(f'[TAKEOFF] Kalkış komutu gönderildi: {alt}m')

        except Exception as e:
            self.get_logger().error(f'Takeoff hatası: {e}')

    def on_land(self, msg: String):
        """İniş komutu"""
        if not self.conn:
            return
        try:
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info('LAND komutu gönderildi')
        except Exception as e:
            self.get_logger().error(f'Land hatası: {e}')

    def on_goto(self, msg: String):
        """GPS konumuna git: {"lat": 41.51, "lon": 36.11, "alt": 60.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            lat  = float(data.get('lat', 0))
            lon  = float(data.get('lon', 0))
            alt  = float(data.get('alt', 60))

            self.conn.mav.set_position_target_global_int_send(
                0,
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,
                int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0,
                0, 0, 0,
                0, 0)
        except Exception as e:
            self.get_logger().error(f'Goto hatası: {e}')

    def on_velocity(self, msg: String):
        """NED hız komutu: {"vn": 5.0, "ve": 0.0, "vd": -1.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            vn   = float(data.get('vn', 0))
            ve   = float(data.get('ve', 0))
            vd   = float(data.get('vd', 0))

            # RC override timeout koruması
            now = time.time()
            if self._rc_override_active and (now - self._rc_override_last) > RC_OVERRIDE_TIMEOUT_S:
                self.get_logger().warn('[RC] Override timeout — komut reddedildi')
                return
            self._rc_override_last   = now
            self._rc_override_active = True

            self.conn.mav.set_position_target_local_ned_send(
                0,
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,
                vn, ve, vd,
                0, 0, 0,
                0, 0)
        except Exception as e:
            self.get_logger().error(f'Velocity hatası: {e}')

    def on_yaw(self, msg: String):
        """Yaw komutu: {"heading": 180.0, "rate": 10.0}"""
        if not self.conn:
            return
        try:
            data    = json.loads(msg.data)
            heading = float(data.get('heading', 0))
            rate    = float(data.get('rate', 0))
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0, heading, rate, 1, 0, 0, 0, 0)
        except Exception as e:
            self.get_logger().error(f'Yaw hatası: {e}')

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='SIHA MAVLink Bridge')
    parser.add_argument('--url', default='udp:127.0.0.1:14550',
                        help='MAVLink bağlantı URL\'i')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MavlinkBridge(url=args.url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
