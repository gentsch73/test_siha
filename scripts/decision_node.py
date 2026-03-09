#!/usr/bin/env python3
"""
decision_node.py — Akıllı Hedef Seçimi & Kaçınma Modülü
TEKNOFEST 2026 Savaşan İHA Yarışması

Sunucu telemetrisinden gelen rakip drone konumlarını değerlendirerek
en uygun hedefi seçer ve tehdit tespiti yapar.

Yayınlar:
  /decision/target   (std_msgs/String) — seçilen hedef (GPS)
  /decision/threats  (std_msgs/String) — aktif tehdit listesi

Abonelikler:
  /sunucu_telemetri  (std_msgs/String) — 15 rakip İHA konumu (JSON)
  /telemetry/own     (std_msgs/String) — kendi konum/heading/hız bilgisi
  /tracker/events    (std_msgs/String) — son kilitlenilen drone ID'si

Kullanım:
  python3 scripts/decision_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time


# ─── Yardımcı geometri fonksiyonları ───────────────────────────────────────

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki GPS koordinatı arasındaki yatay mesafeyi metre cinsinden döndürür."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """lat1/lon1'den lat2/lon2'ye olan pusulanı (0-360°) döndürür."""
    dlon = math.radians(lon2 - lon1)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    brng = math.degrees(math.atan2(x, y))
    return (brng + 360) % 360


def angle_diff(a: float, b: float) -> float:
    """İki açı arasındaki [-180, 180] farkı döndürür."""
    diff = (a - b + 180) % 360 - 180
    return diff


# ─── Skor hesaplayıcı sabitleri ────────────────────────────────────────────
MAX_RANGE_M     = 5000.0   # Bu mesafeden uzak drone'lar değerlendirme dışı
EVADE_RANGE_M   = 50.0     # Kaçınma mesafesi eşiği
EVADE_SPEED_MS  = 5.0      # Yaklaşma hızı eşiği (m/s)
EVADE_ANGLE_MIN = 120.0    # Arkadan gelen açı eşiği (±)
CLEAR_RANGE_M   = 100.0    # Tehdidin geçtiği mesafe
RESELECT_S      = 1.0      # Hedef yeniden değerlendirme periyodu


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Durum
        self.rivals: list = []           # Rakip drone listesi
        self.own_lat    = 0.0
        self.own_lon    = 0.0
        self.own_alt    = 0.0
        self.own_heading = 0.0           # derece
        self.own_speed   = 0.0           # m/s
        self.last_locked_id: int = -1    # Son kilitlenilen drone
        self.current_target_id: int = -1
        self._last_select_time: float = 0.0

        # Yayıncılar
        self.pub_target  = self.create_publisher(String, '/decision/target',  10)
        self.pub_threats = self.create_publisher(String, '/decision/threats', 10)

        # Abonelikler
        self.create_subscription(String, '/sunucu_telemetri', self._on_rivals,    10)
        self.create_subscription(String, '/telemetry/own',    self._on_own_telem, 10)
        self.create_subscription(String, '/tracker/events',   self._on_tracker,   10)

        # Zamanlayıcılar
        self.create_timer(RESELECT_S,   self._evaluate_targets)
        self.create_timer(0.2,           self._evaluate_threats)

        self.get_logger().info('Karar modülü başlatıldı')

    # ─── Abonelik geri çağırmaları ──────────────────────────────────────────

    def _on_rivals(self, msg: String):
        try:
            data = json.loads(msg.data)
            rivals = data.get('konumBilgileri', [])
            if isinstance(rivals, list):
                self.rivals = rivals
        except Exception:
            pass

    def _on_own_telem(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.own_lat     = d.get('iha_enlem',    self.own_lat)
            self.own_lon     = d.get('iha_boylam',   self.own_lon)
            self.own_alt     = d.get('iha_irtifa',   self.own_alt)
            self.own_heading = d.get('iha_yonelme',  self.own_heading)
            self.own_speed   = d.get('iha_hiz',      self.own_speed)
        except Exception:
            pass

    def _on_tracker(self, msg: String):
        data = msg.data
        if data.startswith('LOCK_COMPLETE:'):
            try:
                self.last_locked_id = int(data.split(':')[1])
            except ValueError:
                pass

    # ─── Hedef Değerlendirme ────────────────────────────────────────────────

    def _score_drone(self, drone: dict) -> float:
        """
        Bir rakip drone için bütünleşik skor hesapla (yüksek = daha iyi hedef).
        Faktörler:
          1. Mesafe skoru   — yakın olan daha yüksek
          2. Açı farkı      — kamera FOV'una yakın tercih, arkasındaki ceza
          3. Manevra maliyeti — heading farkı büyükse ceza
          4. Son kilit cezası — aynı drone'a tekrar kilitlenme yasağı
        """
        lat = drone.get('iha_enlem', 0.0)
        lon = drone.get('iha_boylam', 0.0)
        alt = drone.get('iha_irtifa', self.own_alt)
        tid = drone.get('takim_numarasi', -1)

        if self.own_lat == 0.0 and self.own_lon == 0.0:
            return 0.0

        # Yatay mesafe (m)
        h_dist = haversine_m(self.own_lat, self.own_lon, lat, lon)
        v_dist = abs(alt - self.own_alt)
        dist3d = math.sqrt(h_dist * h_dist + v_dist * v_dist)

        if dist3d > MAX_RANGE_M or dist3d < 1.0:
            return 0.0

        # 1. Mesafe skoru (0-40): yakın → yüksek
        dist_score = 40.0 * (1.0 - min(dist3d / MAX_RANGE_M, 1.0))

        # 2. Açı farkı skoru (0-30): doğrudan önde → 30, arkada → -20
        brng = bearing_deg(self.own_lat, self.own_lon, lat, lon)
        adiff = abs(angle_diff(brng, self.own_heading))   # 0-180
        if adiff <= 45:
            angle_score = 30.0
        elif adiff <= 90:
            angle_score = 30.0 - (adiff - 45) * (30.0 / 45.0)
        else:
            angle_score = -20.0 * ((adiff - 90) / 90.0)

        # 3. Manevra maliyeti (0 ile -20): heading farkı >90° ise ceza
        turn_cost = 0.0
        if adiff > 90:
            turn_cost = -20.0 * min((adiff - 90) / 90.0, 1.0)

        # 4. Son kilit cezası
        repeat_penalty = -50.0 if tid == self.last_locked_id else 0.0

        # 5. Ulaşma süresi tahmini (s) — küçük süre → bonus (maks 10 puan)
        speed = max(self.own_speed, 5.0)
        eta = dist3d / speed + abs(angle_diff(brng, self.own_heading)) / 90.0
        eta_score = max(0.0, 10.0 - eta / 10.0)

        return dist_score + angle_score + turn_cost + repeat_penalty + eta_score

    def _evaluate_targets(self):
        """1 Hz: en iyi hedefi seç ve /decision/target'a yayınla"""
        if not self.rivals or self.own_lat == 0.0:
            return

        best_score = -1e9
        best_drone = None

        for drone in self.rivals:
            score = self._score_drone(drone)
            if score > best_score:
                best_score = score
                best_drone = drone

        if best_drone is None:
            return

        tid  = best_drone.get('takim_numarasi', -1)
        lat  = best_drone.get('iha_enlem', 0.0)
        lon  = best_drone.get('iha_boylam', 0.0)
        alt  = best_drone.get('iha_irtifa', self.own_alt)
        dist = haversine_m(self.own_lat, self.own_lon, lat, lon)

        if tid != self.current_target_id:
            self.get_logger().info(
                f'[HEDEF] Yeni hedef: T{tid}  dist={dist:.0f}m  skor={best_score:.1f}')
            self.current_target_id = tid

        msg = String()
        msg.data = json.dumps({
            'takim_numarasi': tid,
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'mesafe_m': round(dist, 1),
            'skor': round(best_score, 2),
        })
        self.pub_target.publish(msg)

    # ─── Tehdit Değerlendirmesi ─────────────────────────────────────────────

    def _evaluate_threats(self):
        """5 Hz: arkadan yaklaşan drone tehdidi değerlendir"""
        threats = []

        for drone in self.rivals:
            lat = drone.get('iha_enlem', 0.0)
            lon = drone.get('iha_boylam', 0.0)
            alt = drone.get('iha_irtifa', self.own_alt)
            tid = drone.get('takim_numarasi', -1)
            speed = drone.get('iha_hiz', 0.0)

            if self.own_lat == 0.0 and self.own_lon == 0.0:
                continue

            h_dist = haversine_m(self.own_lat, self.own_lon, lat, lon)
            v_dist = abs(alt - self.own_alt)
            dist3d = math.sqrt(h_dist * h_dist + v_dist * v_dist)

            # CLEAR_RANGE_M'den uzaktaki drone'lar tehdit değil
            if dist3d > CLEAR_RANGE_M:
                continue

            # Tehdidin aktif eşiğini ayrıca kontrol et
            if dist3d > EVADE_RANGE_M:
                continue

            # Drone'un geldiği açıyı hesapla
            brng  = bearing_deg(lat, lon, self.own_lat, self.own_lon)   # bize doğru açı
            adiff = abs(angle_diff(brng, self.own_heading))              # heading'e göre

            # Arkadan yaklaşma: açı farkı > EVADE_ANGLE_MIN
            from_behind = adiff > EVADE_ANGLE_MIN

            # Tehdit: yakın + hızlı + arkadan
            if dist3d < EVADE_RANGE_M and speed > EVADE_SPEED_MS and from_behind:
                threats.append({
                    'takim_numarasi': tid,
                    'mesafe_m': round(dist3d, 1),
                    'hiz_ms': round(speed, 1),
                    'aci_fark': round(adiff, 1),
                    'evade_vector': self._evade_vector(brng),
                })
                self.get_logger().warn(
                    f'[TEHDİT] T{tid} dist={dist3d:.0f}m  hız={speed:.1f}m/s  açı={adiff:.0f}°')

        msg = String()
        msg.data = json.dumps(threats)
        self.pub_threats.publish(msg)

    def _evade_vector(self, threat_bearing: float) -> dict:
        """
        Tehdide dik kaçış vektörü hesapla.
        Sağa 90° dön + hafif tırman.
        """
        evade_hdg = (threat_bearing + 90) % 360   # dik kaçış
        return {
            'heading': round(evade_hdg, 1),
            'pitch_deg': 10.0,   # tırmanma
            'roll_deg': 45.0,    # sert dönüş
        }


def main():
    rclpy.init()
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
