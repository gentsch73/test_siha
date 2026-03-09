#!/usr/bin/env python3
"""
mavlink_bridge.py — pymavlink ↔ ROS2 Köprüsü

SITL (ArduPilot) ile ROS2 arasında MAVLink haberleşme sağlar.
C++ tarafındaki FlightController ROS2 topic'lerine yayın yapar,
bu köprü o komutları pymavlink ile SITL'a iletir ve
telemetri verisini geri ROS2'ye yayınlar.

Kullanım:
  python3 mavlink_bridge.py
  python3 mavlink_bridge.py --url udp:127.0.0.1:14550
  python3 mavlink_bridge.py --url tcp:127.0.0.1:5760
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time
import math
import threading
import argparse
from pymavlink import mavutil


class MavlinkBridge(Node):
    def __init__(self, url='udp:127.0.0.1:14550'):
        super().__init__('mavlink_bridge')
        self.url = url
        self.conn = None
        self.armed = False
        self.mode = 'MANUAL'
        self.target_system = 1
        self.target_component = 1

        # ── Telemetri Yayını (Bridge → C++ FlightController) ──
        self.pub_telemetry = self.create_publisher(
            String, '/mavlink/telemetry', 10)

        # ── Komut Abonelikleri (C++ FlightController → Bridge) ──
        self.sub_arm = self.create_subscription(
            String, '/mavlink/cmd/arm', self.on_arm, 10)
        self.sub_mode = self.create_subscription(
            String, '/mavlink/cmd/mode', self.on_mode, 10)
        self.sub_takeoff = self.create_subscription(
            String, '/mavlink/cmd/takeoff', self.on_takeoff, 10)
        self.sub_land = self.create_subscription(
            String, '/mavlink/cmd/land', self.on_land, 10)
        self.sub_goto = self.create_subscription(
            String, '/mavlink/cmd/goto', self.on_goto, 10)
        self.sub_velocity = self.create_subscription(
            String, '/mavlink/cmd/velocity', self.on_velocity, 10)
        self.sub_yaw = self.create_subscription(
            String, '/mavlink/cmd/yaw', self.on_yaw, 10)

        # ── Bağlantı ──
        self.connect()

        # ── Telemetri okuma thread ──
        self.running = True
        self.telem_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telem_thread.start()

        # ── Heartbeat timer (1 Hz) ──
        self.create_timer(1.0, self.send_heartbeat)

        # ── Telemetri yayın timer (10 Hz) ──
        self.latest_telem = {}
        self.create_timer(0.1, self.publish_telemetry)

        self.get_logger().info(f'MAVLink Bridge başlatıldı: {url}')

    def connect(self):
        try:
            self.conn = mavutil.mavlink_connection(self.url)
            self.conn.wait_heartbeat(timeout=10)
            self.target_system = self.conn.target_system
            self.target_component = self.conn.target_component
            self.get_logger().info(
                f'SITL bağlantısı kuruldu! sys={self.target_system} comp={self.target_component}')
        except Exception as e:
            self.get_logger().error(f'SITL bağlantı hatası: {e}')
            self.conn = None

    def send_heartbeat(self):
        if not self.conn:
            return
        try:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
        except Exception:
            pass

    # ── Telemetri Okuma ──

    def telemetry_loop(self):
        """Arka plan thread: SITL'dan gelen MAVLink mesajlarını oku"""
        while self.running:
            if not self.conn:
                time.sleep(1)
                continue
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'GLOBAL_POSITION_INT':
                    self.latest_telem['lat'] = msg.lat / 1e7
                    self.latest_telem['lon'] = msg.lon / 1e7
                    self.latest_telem['alt'] = msg.relative_alt / 1000.0  # mm → m
                    self.latest_telem['vx'] = msg.vx / 100.0   # cm/s → m/s
                    self.latest_telem['vy'] = msg.vy / 100.0
                    self.latest_telem['vz'] = msg.vz / 100.0
                    self.latest_telem['hdg'] = msg.hdg / 100.0  # cdeg → deg

                elif msg_type == 'ATTITUDE':
                    self.latest_telem['roll'] = math.degrees(msg.roll)
                    self.latest_telem['pitch'] = math.degrees(msg.pitch)
                    self.latest_telem['yaw'] = math.degrees(msg.yaw)

                elif msg_type == 'VFR_HUD':
                    self.latest_telem['airspeed'] = msg.airspeed
                    self.latest_telem['groundspeed'] = msg.groundspeed
                    self.latest_telem['heading'] = msg.heading
                    self.latest_telem['throttle'] = msg.throttle
                    self.latest_telem['alt_vfr'] = msg.alt

                elif msg_type == 'SYS_STATUS':
                    self.latest_telem['battery'] = msg.battery_remaining

                elif msg_type == 'HEARTBEAT':
                    self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.latest_telem['armed'] = self.armed
                    mode_map = mavutil.mode_mapping_bynumber(msg.type)
                    if mode_map and msg.custom_mode in mode_map:
                        self.mode = mode_map[msg.custom_mode]
                    self.latest_telem['mode'] = self.mode

                elif msg_type == 'GPS_RAW_INT':
                    self.latest_telem['gps_fix'] = msg.fix_type
                    self.latest_telem['satellites'] = msg.satellites_visible

            except Exception:
                pass

    def publish_telemetry(self):
        """Son telemetri verisini ROS2'ye yayınla"""
        if not self.latest_telem:
            return

        msg = String()
        msg.data = json.dumps(self.latest_telem)
        self.pub_telemetry.publish(msg)

    # ── Komut İşleyicileri ──

    def on_arm(self, msg):
        """ARM/DISARM komutu: {"arm": true/false}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            arm_value = 1 if data.get('arm', True) else 0
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, arm_value, 0, 0, 0, 0, 0, 0)
            self.get_logger().info(f'{"ARM" if arm_value else "DISARM"} komutu gönderildi')
        except Exception as e:
            self.get_logger().error(f'ARM hatası: {e}')

    def on_mode(self, msg):
        """Mod değiştir: {"mode": "GUIDED"}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            mode_str = data.get('mode', 'MANUAL')
            mode_id = self.conn.mode_mapping().get(mode_str)
            if mode_id is not None:
                self.conn.set_mode(mode_id)
                self.get_logger().info(f'Mod değiştirildi: {mode_str}')
            else:
                self.get_logger().warn(f'Bilinmeyen mod: {mode_str}')
        except Exception as e:
            self.get_logger().error(f'Mod hatası: {e}')

    def on_takeoff(self, msg):
        """Kalkış: {"altitude": 30.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            alt = data.get('altitude', 30.0)

            # Önce GUIDED moda geç
            guided_id = self.conn.mode_mapping().get('GUIDED')
            if guided_id is not None:
                self.conn.set_mode(guided_id)
                time.sleep(0.5)

            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt)
            self.get_logger().info(f'TAKEOFF komutu: {alt}m')
        except Exception as e:
            self.get_logger().error(f'Takeoff hatası: {e}')

    def on_land(self, msg):
        """İniş"""
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

    def on_goto(self, msg):
        """GPS konumuna git: {"lat": 41.51, "lon": 36.11, "alt": 60.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            lat = data.get('lat', 0)
            lon = data.get('lon', 0)
            alt = data.get('alt', 60)

            self.conn.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask: only position
                int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0)     # yaw, yaw_rate
            self.get_logger().debug(f'GOTO: {lat:.6f}, {lon:.6f}, {alt:.0f}m')
        except Exception as e:
            self.get_logger().error(f'Goto hatası: {e}')

    def on_velocity(self, msg):
        """NED hız komutu: {"vn": 5.0, "ve": 0.0, "vd": -1.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            vn = data.get('vn', 0)
            ve = data.get('ve', 0)
            vd = data.get('vd', 0)

            self.conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask: only velocity
                0, 0, 0,      # position (ignored)
                vn, ve, vd,   # velocity
                0, 0, 0,      # acceleration
                0, 0)         # yaw, yaw_rate
        except Exception as e:
            self.get_logger().error(f'Velocity hatası: {e}')

    def on_yaw(self, msg):
        """Yaw komutu: {"heading": 180.0, "rate": 10.0}"""
        if not self.conn:
            return
        try:
            data = json.loads(msg.data)
            heading = data.get('heading', 0)
            rate = data.get('rate', 0)
            # MAV_CMD_CONDITION_YAW
            self.conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0, heading, rate,
                1,  # direction: 1=CW
                0,  # 0=absolute, 1=relative
                0, 0, 0)
        except Exception as e:
            self.get_logger().error(f'Yaw hatası: {e}')

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='udp:127.0.0.1:14550')
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
