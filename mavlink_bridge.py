#!/usr/bin/env python3
"""
mavlink_bridge.py v2 — ArduPlane Uyumlu

Düzeltmeler:
  - CONDITION_YAW kaldırıldı (ArduPlane desteklemiyor)
  - Heading: DO_REPOSITION (GPS waypoint üzerinden)
  - Takeoff: mode GUIDED → arm force → mode TAKEOFF
  - /mavlink/cmd/navigate: heading+speed+alt → 500m ileri waypoint
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, math, threading
from pymavlink import mavutil


class MavlinkBridge(Node):
    def __init__(self, url='udp:127.0.0.1:14550'):
        super().__init__('mavlink_bridge')
        self.url = url
        self.conn = None
        self.armed = False
        self.mode = 'MANUAL'
        self.tsys = 1
        self.tcomp = 1

        self.pub_telem = self.create_publisher(String, '/mavlink/telemetry', 10)

        for topic, cb in [
            ('/mavlink/cmd/arm', self.on_arm),
            ('/mavlink/cmd/mode', self.on_mode),
            ('/mavlink/cmd/takeoff', self.on_takeoff),
            ('/mavlink/cmd/land', self.on_land),
            ('/mavlink/cmd/goto', self.on_goto),
            ('/mavlink/cmd/navigate', self.on_navigate),
            ('/mavlink/cmd/velocity', self.on_velocity),
        ]:
            self.create_subscription(String, topic, cb, 10)

        self.connect()
        self.running = True
        self.telem = {}
        self.tlock = threading.Lock()
        threading.Thread(target=self.telem_loop, daemon=True).start()
        self.create_timer(1.0, self.heartbeat)
        self.create_timer(0.1, self.pub_telem_cb)
        self.get_logger().info(f'MAVLink Bridge v2 (ArduPlane) — {url}')

    def connect(self):
        try:
            self.conn = mavutil.mavlink_connection(self.url)
            self.conn.wait_heartbeat(timeout=30)
            self.tsys = self.conn.target_system
            self.tcomp = self.conn.target_component
            self.get_logger().info(f'SITL bağlandı (sys={self.tsys})')
        except Exception as e:
            self.get_logger().error(f'Bağlantı hatası: {e}')
            self.conn = None

    def heartbeat(self):
        if self.conn:
            try: self.conn.mav.heartbeat_send(6, 8, 0, 0, 0)
            except: pass

    def telem_loop(self):
        while self.running:
            if not self.conn: time.sleep(1); continue
            try:
                m = self.conn.recv_match(blocking=True, timeout=1.0)
                if not m: continue
                t = m.get_type()
                with self.tlock:
                    if t == 'GLOBAL_POSITION_INT':
                        self.telem.update(lat=m.lat/1e7, lon=m.lon/1e7,
                            alt=m.relative_alt/1000.0, hdg=m.hdg/100.0)
                    elif t == 'VFR_HUD':
                        self.telem.update(airspeed=m.airspeed, groundspeed=m.groundspeed,
                            heading=m.heading, throttle=m.throttle)
                    elif t == 'ATTITUDE':
                        self.telem.update(roll=math.degrees(m.roll),
                            pitch=math.degrees(m.pitch), yaw=math.degrees(m.yaw))
                    elif t == 'SYS_STATUS':
                        self.telem['battery'] = m.battery_remaining
                    elif t == 'GPS_RAW_INT':
                        self.telem.update(gps_fix=m.fix_type, satellites=m.satellites_visible)
                    elif t == 'HEARTBEAT':
                        self.armed = bool(m.base_mode & 128)
                        self.telem['armed'] = self.armed
                        mm = mavutil.mode_mapping_bynumber(m.type)
                        if mm and m.custom_mode in mm:
                            self.mode = mm[m.custom_mode]
                        self.telem['mode'] = self.mode
            except: pass

    def pub_telem_cb(self):
        with self.tlock:
            if self.telem:
                msg = String(); msg.data = json.dumps(self.telem)
                self.pub_telem.publish(msg)

    # ── ARM (force) ──
    def on_arm(self, msg):
        if not self.conn: return
        d = json.loads(msg.data)
        v = 1 if d.get('arm', True) else 0
        self.conn.mav.command_long_send(self.tsys, self.tcomp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            v, 21196, 0, 0, 0, 0, 0)
        self.get_logger().info(f'{"ARM(force)" if v else "DISARM"}')

    # ── MOD ──
    def on_mode(self, msg):
        if not self.conn: return
        d = json.loads(msg.data)
        m = d.get('mode', 'MANUAL')
        mm = self.conn.mode_mapping()
        if m in mm:
            self.conn.set_mode(mm[m])
            self.get_logger().info(f'Mod → {m}')
        else:
            self.get_logger().warn(f'Mod yok: {m}')

    # ── TAKEOFF (ArduPlane) ──
    def on_takeoff(self, msg):
        if not self.conn: return
        d = json.loads(msg.data)
        alt = d.get('altitude', 30.0)
        mm = self.conn.mode_mapping()

        self.get_logger().info(f'Takeoff sekansı: {alt}m')

        # 1) GUIDED
        if 'GUIDED' in mm:
            self.conn.set_mode(mm['GUIDED']); time.sleep(0.5)
        # 2) ARM force
        self.conn.mav.command_long_send(self.tsys, self.tcomp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 21196, 0, 0, 0, 0, 0)
        time.sleep(1.0)
        # 3) TAKEOFF modu
        if 'TAKEOFF' in mm:
            self.conn.set_mode(mm['TAKEOFF'])
            self.get_logger().info('Mode TAKEOFF aktif')
        else:
            self.conn.mav.command_long_send(self.tsys, self.tcomp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                15, 0, 0, 0, 0, 0, alt)
            self.get_logger().info(f'NAV_TAKEOFF: {alt}m')

    # ── LAND ──
    def on_land(self, msg):
        if not self.conn: return
        mm = self.conn.mode_mapping()
        for m in ['QLAND', 'RTL']:
            if m in mm: self.conn.set_mode(mm[m]); return

    # ── GOTO (GPS) ──
    def on_goto(self, msg):
        if not self.conn: return
        d = json.loads(msg.data)
        self.conn.mav.command_int_send(self.tsys, self.tcomp,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
            d.get('speed', -1), 0, 0, float('nan'),
            int(d['lat']*1e7), int(d['lon']*1e7), d.get('alt', 60))

    # ── NAVIGATE (heading→waypoint) ──
    def on_navigate(self, msg):
        """heading+speed+alt → 500m ileri GPS noktası → DO_REPOSITION"""
        if not self.conn: return
        d = json.loads(msg.data)
        hdg = d.get('heading', 0)
        spd = d.get('speed', 20)
        alt = d.get('altitude', 60)

        with self.tlock:
            clat = self.telem.get('lat', 0)
            clon = self.telem.get('lon', 0)
        if clat == 0: return

        dist = 500.0
        dlat = dist * math.cos(math.radians(hdg)) / 111320.0
        dlon = dist * math.sin(math.radians(hdg)) / (111320.0 * math.cos(math.radians(clat)))

        self.conn.mav.command_int_send(self.tsys, self.tcomp,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
            spd, 0, 0, float('nan'),
            int((clat+dlat)*1e7), int((clon+dlon)*1e7), alt)

    # ── VELOCITY ──
    def on_velocity(self, msg):
        if not self.conn: return
        d = json.loads(msg.data)
        self.conn.mav.set_position_target_local_ned_send(
            0, self.tsys, self.tcomp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
            0,0,0, d.get('vn',0), d.get('ve',0), d.get('vd',0), 0,0,0, 0,0)

    def destroy_node(self):
        self.running = False; super().destroy_node()

def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--url', default='udp:127.0.0.1:14550')
    a, r = p.parse_known_args()
    rclpy.init(args=r)
    n = MavlinkBridge(url=a.url)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
