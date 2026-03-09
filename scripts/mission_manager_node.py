#!/usr/bin/env python3
"""
mission_manager_node.py — Otomatik Takeoff & Görev Akışı
TEKNOFEST 2026 Savaşan İHA Yarışması

MAVLink bridge üzerinden ARM → GUIDED → TAKEOFF → SEARCH akışını yönetir.
Durum makinesi: IDLE → PRE_ARM → ARM → TAKEOFF → CLIMB → SEARCH →
                 CHASE → LOCK → EVADE → RTL

Yayınlar:
  /mission/state  (std_msgs/String) — mevcut görev fazı
  /mavlink/cmd/arm, /mavlink/cmd/mode, /mavlink/cmd/takeoff  — bridge komutları

Abonelikler:
  /mavlink/telemetry  (std_msgs/String) — telemetri (irtifa, mod, arm durumu)
  /tracker/events     (std_msgs/String) — kilitlenme olayları
  /decision/threats   (std_msgs/String) — tehdit listesi

Kullanım:
  python3 scripts/mission_manager_node.py
  python3 scripts/mission_manager_node.py --takeoff-alt 30.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import argparse


# ── Durum Makinesi Fazları ──
class Phase:
    IDLE      = 'IDLE'
    PRE_ARM   = 'PRE_ARM'
    ARM       = 'ARM'
    TAKEOFF   = 'TAKEOFF'
    CLIMB     = 'CLIMB'
    SEARCH    = 'SEARCH'
    CHASE     = 'CHASE'
    LOCK      = 'LOCK'
    EVADE     = 'EVADE'
    RTL       = 'RTL'


# ── Faz geçiş zaman sabitleri ──
IDLE_WAIT_S         = 2.0    # Başlangıçta telemetri bekleme süresi
PRE_ARM_TIMEOUT_S   = 3.0    # GUIDED mod geçiş bekleme süresi
ARM_CONFIRM_S       = 1.5    # ARM sonrası onay bekleme süresi
TAKEOFF_RETRY_S     = 30.0   # Takeoff yeniden deneme süresi
CLIMB_TIMEOUT_S     = 60.0   # Tırmanma zaman aşımı süresi
EVADE_CLEAR_S       = 3.0    # Tehdit geçtikten sonra SEARCH'e dönme süresi


class MissionManagerNode(Node):
    def __init__(self, takeoff_alt: float = 30.0):
        super().__init__('mission_manager')

        self.takeoff_alt = takeoff_alt
        self.phase = Phase.IDLE
        self.phase_entry_time = time.time()

        # Telemetri durumu
        self.telem: dict = {}
        self.armed = False
        self.current_mode = 'UNKNOWN'
        self.current_alt = 0.0

        # Tehdit & kilit sayacı
        self.active_threats: list = []
        self.total_locks: int = 0
        self.lock_state_reported = False

        # ── Yayıncılar ──
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_arm   = self.create_publisher(String, '/mavlink/cmd/arm', 10)
        self.pub_mode  = self.create_publisher(String, '/mavlink/cmd/mode', 10)
        self.pub_takeoff = self.create_publisher(String, '/mavlink/cmd/takeoff', 10)

        # ── Abonelikler ──
        self.create_subscription(String, '/mavlink/telemetry', self._on_telemetry, 10)
        self.create_subscription(String, '/tracker/events',    self._on_tracker_event, 10)
        self.create_subscription(String, '/decision/threats',  self._on_threats, 10)

        # ── Ana döngü zamanlayıcı (5 Hz) ──
        self.create_timer(0.2, self._state_machine_tick)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  MİSYON YÖNETİCİSİ başlatıldı          ║')
        self.get_logger().info(f'║  Takeoff irtifası: {takeoff_alt:.0f}m                ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

    # ── Abonelik geri çağırmaları ──

    def _on_telemetry(self, msg: String):
        try:
            self.telem = json.loads(msg.data)
            self.armed        = self.telem.get('armed', False)
            self.current_mode = self.telem.get('mode', 'UNKNOWN')
            self.current_alt  = self.telem.get('alt', 0.0)
        except Exception:
            pass

    def _on_tracker_event(self, msg: String):
        data = msg.data
        if data.startswith('LOCK_COMPLETE:'):
            self.total_locks += 1
            drone_id = data.split(':')[1]
            self.get_logger().info(f'[KİLİT] Drone {drone_id} kilitlendi — toplam: {self.total_locks}')
            # Kilit sonrası SEARCH'e dön
            if self.phase == Phase.LOCK:
                self._transition(Phase.SEARCH)

    def _on_threats(self, msg: String):
        try:
            self.active_threats = json.loads(msg.data)
        except Exception:
            self.active_threats = []

    # ── Durum makinesi ──

    def _transition(self, new_phase: str):
        old = self.phase
        self.phase = new_phase
        self.phase_entry_time = time.time()
        self.get_logger().info(f'[FAZ] {old} → {new_phase}')
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'phase': self.phase,
            'alt': round(self.current_alt, 1),
            'armed': self.armed,
            'mode': self.current_mode,
            'total_locks': self.total_locks,
            'threats': len(self.active_threats),
        })
        self.pub_state.publish(msg)

    def _elapsed(self) -> float:
        return time.time() - self.phase_entry_time

    def _send_arm(self, arm: bool):
        m = String()
        m.data = json.dumps({'arm': arm})
        self.pub_arm.publish(m)

    def _send_mode(self, mode: str):
        m = String()
        m.data = json.dumps({'mode': mode})
        self.pub_mode.publish(m)

    def _send_takeoff(self, alt: float):
        m = String()
        m.data = json.dumps({'altitude': alt})
        self.pub_takeoff.publish(m)

    def _state_machine_tick(self):
        """5 Hz döngüsünde durum makinesini işle"""
        self._publish_state()

        if self.phase == Phase.IDLE:
            # Otomatik olarak PRE_ARM'a geç
            if self._elapsed() > IDLE_WAIT_S:
                self.get_logger().info('[IDLE] Telemetri bekleniyor...')
                if self.telem:
                    self._transition(Phase.PRE_ARM)

        elif self.phase == Phase.PRE_ARM:
            # GUIDED moda geç
            if self._elapsed() < 1.0:
                self._send_mode('GUIDED')
            elif self._elapsed() > PRE_ARM_TIMEOUT_S:
                if self.current_mode == 'GUIDED':
                    self._transition(Phase.ARM)
                else:
                    # Tekrar dene
                    self._send_mode('GUIDED')
                    self.phase_entry_time = time.time()
                    self.get_logger().warn('[PRE_ARM] GUIDED mod bekleniyor, tekrar deneniyor...')

        elif self.phase == Phase.ARM:
            if not self.armed:
                self._send_arm(True)
                self.get_logger().info('[ARM] ARM komutu gönderildi')
            if self.armed and self._elapsed() > ARM_CONFIRM_S:
                self._transition(Phase.TAKEOFF)

        elif self.phase == Phase.TAKEOFF:
            if self._elapsed() < 1.0:
                self._send_takeoff(self.takeoff_alt)
                self.get_logger().info(f'[TAKEOFF] Kalkış komutu: {self.takeoff_alt}m')
            # Irtifa artmaya başladıysa CLIMB'a geç
            if self.current_alt > 2.0 and self._elapsed() > 2.0:
                self._transition(Phase.CLIMB)
            # Belirtilen süre içinde kalkış olmazsa yeniden dene
            elif self._elapsed() > TAKEOFF_RETRY_S:
                self.get_logger().warn('[TAKEOFF] Zaman aşımı — tekrar deneniyor')
                self._send_takeoff(self.takeoff_alt)
                self.phase_entry_time = time.time()

        elif self.phase == Phase.CLIMB:
            target = self.takeoff_alt * 0.9
            if self.current_alt >= target:
                self.get_logger().info(
                    f'[CLIMB] Hedef irtifaya ulaşıldı: {self.current_alt:.1f}m ≥ {target:.1f}m')
                self._transition(Phase.SEARCH)
            elif self._elapsed() > CLIMB_TIMEOUT_S:
                self.get_logger().warn(
                    f'[CLIMB] Zaman aşımı — mevcut irtifa: {self.current_alt:.1f}m, devam ediliyor')
                self._transition(Phase.SEARCH)

        elif self.phase == Phase.SEARCH:
            # Tehdit varsa EVADE'e geç
            if self.active_threats:
                self.get_logger().warn(f'[SEARCH] {len(self.active_threats)} tehdit — EVADE moduna geçiş')
                self._transition(Phase.EVADE)

        elif self.phase == Phase.CHASE:
            if self.active_threats:
                self._transition(Phase.EVADE)

        elif self.phase == Phase.LOCK:
            if self.active_threats:
                self._transition(Phase.EVADE)

        elif self.phase == Phase.EVADE:
            # Tehdit geçtikten sonra SEARCH'e dön
            if not self.active_threats and self._elapsed() > EVADE_CLEAR_S:
                self.get_logger().info('[EVADE] Tehdit geçti — SEARCH moduna dönülüyor')
                self._transition(Phase.SEARCH)

        elif self.phase == Phase.RTL:
            pass  # Son faz — insan müdahalesi bekleniyor

    # ── Dış tetikleyiciler (diğer node'lardan çağrılabilir) ──

    def request_chase(self):
        if self.phase == Phase.SEARCH:
            self._transition(Phase.CHASE)

    def request_lock(self):
        if self.phase == Phase.CHASE:
            self._transition(Phase.LOCK)

    def request_rtl(self):
        self._transition(Phase.RTL)
        self._send_mode('RTL')


def main():
    parser = argparse.ArgumentParser(description='SIHA Görev Yöneticisi')
    parser.add_argument('--takeoff-alt', type=float, default=30.0,
                        help='Otomatik kalkış irtifası (metre)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MissionManagerNode(takeoff_alt=args.takeoff_alt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
