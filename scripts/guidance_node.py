#!/usr/bin/env python3
"""
guidance_node.py — Gelişmiş Yönlendirme Node'u
TEKNOFEST 2026 Savaşan İHA Yarışması

Görev fazına göre uygun yönlendirme modunu uygular:
  - SEARCH : GPS hedefine doğru yönelme
  - CHASE  : Vision tracker'dan gelen piksel hatasına göre yönelme (PID)
  - EVADE  : Kaçınma manevra komutları

Yayınlar:
  /mavlink/cmd/goto      (std_msgs/String) — GPS hedef komutu
  /mavlink/cmd/velocity  (std_msgs/String) — NED hız komutu
  /mavlink/cmd/yaw       (std_msgs/String) — Yaw komutu

Abonelikler:
  /mission/state      (std_msgs/String) — görev fazı
  /tracker/target     (std_msgs/String) — piksel hedef (nx, ny, lock_pct)
  /decision/target    (std_msgs/String) — GPS hedef (lat, lon, alt)
  /decision/threats   (std_msgs/String) — tehdit listesi (kaçınma vektörü)
  /telemetry/own      (std_msgs/String) — kendi telemetri
  /mavlink/telemetry  (std_msgs/String) — MAVLink telemetri

Kullanım:
  python3 scripts/guidance_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time
import argparse


# ─── PID Kontrolcü ─────────────────────────────────────────────────────────

class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt  = now - self._prev_time
        if dt <= 0:
            dt = 0.02
        self._integral  += error * dt
        derivative       = (error - self._prev_err) / dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_err   = error
        self._prev_time  = now
        return max(self.out_min, min(self.out_max, output))

    def reset(self):
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = time.time()


# ─── Sabitler ──────────────────────────────────────────────────────────────
CRUISE_SPEED_MS  = 20.0    # SEARCH modunda seyir hızı
CHASE_SPEED_MS   = 15.0    # CHASE modunda hız
EVADE_SPEED_MS   = 25.0    # Kaçınma sırasında hız
EVADE_DURATION_S =  5.0    # Kaçınma manevra süresi
VD_SCALE_FACTOR  =  5.0    # Dikey hız ölçek faktörü (PID çıkışı → m/s)


class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')

        # Durum
        self.mission_phase  = 'UNKNOWN'
        self.own_lat  = 0.0
        self.own_lon  = 0.0
        self.own_alt  = 0.0
        self.own_heading = 0.0
        self.own_speed   = 0.0

        # Hedef bilgileri
        self.gps_target: dict  = {}     # SEARCH modu
        self.pixel_target: dict = {}    # CHASE modu
        self.threats: list      = []    # EVADE modu

        self.evade_start   = 0.0
        self.evade_vector  = {}

        # PID kontrolcüler (yaw, pitch)
        self.pid_yaw   = PID(kp=0.8, ki=0.01, kd=0.2, out_min=-90, out_max=90)
        self.pid_pitch = PID(kp=0.6, ki=0.005, kd=0.15, out_min=-30, out_max=30)

        # Yayıncılar
        self.pub_goto     = self.create_publisher(String, '/mavlink/cmd/goto',     10)
        self.pub_velocity = self.create_publisher(String, '/mavlink/cmd/velocity', 10)
        self.pub_yaw      = self.create_publisher(String, '/mavlink/cmd/yaw',      10)

        # Abonelikler
        self.create_subscription(String, '/mission/state',     self._on_mission,      10)
        self.create_subscription(String, '/tracker/target',    self._on_pixel_target, 10)
        self.create_subscription(String, '/decision/target',   self._on_gps_target,   10)
        self.create_subscription(String, '/decision/threats',  self._on_threats,      10)
        self.create_subscription(String, '/telemetry/own',     self._on_own_telem,    10)
        self.create_subscription(String, '/mavlink/telemetry', self._on_mav_telem,    10)

        # Ana döngü (20 Hz)
        self.create_timer(0.05, self._guidance_tick)

        self.get_logger().info('Guidance node başlatıldı')

    # ─── Abonelik geri çağırmaları ──────────────────────────────────────────

    def _on_mission(self, msg: String):
        try:
            d = json.loads(msg.data)
            new_phase = d.get('phase', 'UNKNOWN')
            if new_phase != self.mission_phase:
                self.get_logger().info(f'[GUIDANCE] Faz: {self.mission_phase} → {new_phase}')
                self.mission_phase = new_phase
                self.pid_yaw.reset()
                self.pid_pitch.reset()
        except Exception:
            pass

    def _on_pixel_target(self, msg: String):
        try:
            self.pixel_target = json.loads(msg.data)
        except Exception:
            pass

    def _on_gps_target(self, msg: String):
        try:
            self.gps_target = json.loads(msg.data)
        except Exception:
            pass

    def _on_threats(self, msg: String):
        try:
            threats = json.loads(msg.data)
            if threats and not self.threats:
                # Yeni tehdit — kaçınma başlat
                self.evade_start  = time.time()
                self.evade_vector = threats[0].get('evade_vector', {})
                self.get_logger().warn('[EVADE] Kaçınma manevrasına başlanıyor')
            self.threats = threats
        except Exception:
            pass

    def _on_own_telem(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.own_lat     = d.get('iha_enlem',   self.own_lat)
            self.own_lon     = d.get('iha_boylam',  self.own_lon)
            self.own_alt     = d.get('iha_irtifa',  self.own_alt)
            self.own_heading = d.get('iha_yonelme', self.own_heading)
            self.own_speed   = d.get('iha_hiz',     self.own_speed)
        except Exception:
            pass

    def _on_mav_telem(self, msg: String):
        try:
            d = json.loads(msg.data)
            # MAVLink telemetri zaten ayrıntılı — kendi telemetri yoksa kullan
            if self.own_lat == 0.0:
                self.own_lat     = d.get('lat',     self.own_lat)
                self.own_lon     = d.get('lon',     self.own_lon)
                self.own_alt     = d.get('alt',     self.own_alt)
                self.own_heading = d.get('heading', self.own_heading)
                self.own_speed   = d.get('groundspeed', self.own_speed)
        except Exception:
            pass

    # ─── Ana Yönlendirme Döngüsü ───────────────────────────────────────────

    def _guidance_tick(self):
        phase = self.mission_phase

        if phase == 'SEARCH':
            self._guide_search()
        elif phase in ('CHASE', 'LOCK'):
            self._guide_chase()
        elif phase == 'EVADE':
            self._guide_evade()
        # Diğer fazlarda komut gönderilmez

    def _guide_search(self):
        """GPS hedefine doğru yönelme"""
        if not self.gps_target:
            return

        target_lat = self.gps_target.get('lat', 0.0)
        target_lon = self.gps_target.get('lon', 0.0)
        target_alt = self.gps_target.get('alt', self.own_alt)

        if target_lat == 0.0:
            return

        m = String()
        m.data = json.dumps({
            'lat': target_lat,
            'lon': target_lon,
            'alt': target_alt,
        })
        self.pub_goto.publish(m)

    def _guide_chase(self):
        """Piksel hatasına göre yaw/pitch PID kontrolü"""
        if not self.pixel_target:
            return

        nx = self.pixel_target.get('nx', 0.0)   # [-1, 1]  sağ/sol
        ny = self.pixel_target.get('ny', 0.0)   # [-1, 1]  aşağı/yukarı

        # Yaw komutu
        yaw_delta = self.pid_yaw.update(nx)
        new_heading = (self.own_heading + yaw_delta) % 360

        yaw_msg = String()
        yaw_msg.data = json.dumps({'heading': round(new_heading, 1), 'rate': 15.0})
        self.pub_yaw.publish(yaw_msg)

        # Hız komutu (NED) — yükseklik tutmak için vd=0
        speed = CHASE_SPEED_MS
        hdg_rad = math.radians(new_heading)
        vn = speed * math.cos(hdg_rad)
        ve = speed * math.sin(hdg_rad)
        vd = self.pid_pitch.update(-ny) * VD_SCALE_FACTOR   # piksel yukarı → tırman

        vel_msg = String()
        vel_msg.data = json.dumps({
            'vn': round(vn, 2),
            've': round(ve, 2),
            'vd': round(vd, 2),
        })
        self.pub_velocity.publish(vel_msg)

    def _guide_evade(self):
        """Kaçınma manevrasını uygula"""
        if not self.evade_vector:
            return

        evade_hdg   = self.evade_vector.get('heading', self.own_heading)
        evade_pitch = self.evade_vector.get('pitch_deg', 10.0)
        elapsed     = time.time() - self.evade_start

        if elapsed > EVADE_DURATION_S:
            return   # Kaçınma süresi doldu — mission_manager SEARCH'e geçirecek

        # Sert yaw komutu
        yaw_msg = String()
        yaw_msg.data = json.dumps({'heading': round(evade_hdg, 1), 'rate': 30.0})
        self.pub_yaw.publish(yaw_msg)

        # Hız komutu: tam gaz + tırmanma
        speed = EVADE_SPEED_MS
        hdg_rad = math.radians(evade_hdg)
        vn = speed * math.cos(hdg_rad)
        ve = speed * math.sin(hdg_rad)
        vd = -evade_pitch / 10.0 * speed   # yukarı = negatif vd

        vel_msg = String()
        vel_msg.data = json.dumps({
            'vn': round(vn, 2),
            've': round(ve, 2),
            'vd': round(vd, 2),
        })
        self.pub_velocity.publish(vel_msg)


def main():
    rclpy.init()
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
