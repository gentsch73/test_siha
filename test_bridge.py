#!/usr/bin/env python3
"""
siha_autonomy Test Köprüsü

NPC Publisher'dan gelen rakip telemetri verisini,
otonom sistemin beklediği ROS2 topic'lerine dönüştürür.
Ayrıca sahte kamera görüntüsü ve telemetri üretir.

Kullanım:
  python3 test_bridge.py                # Tam test
  python3 test_bridge.py --no-camera    # Kamerasız (sadece telemetri)
  python3 test_bridge.py --headless     # Görüntü penceresi olmadan
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import numpy as np
import cv2
import math
import time
import argparse
from datetime import datetime


class TestBridge(Node):
    def __init__(self, enable_camera=True, headless=False):
        super().__init__('test_bridge')
        self.bridge = CvBridge()
        self.enable_camera = enable_camera
        self.headless = headless

        # ── NPC verisi dinle ──
        self.sub_npc = self.create_subscription(
            String, '/sunucu_telemetri', self.on_npc_data, 10)

        # ── Sahte kamera yayını ──
        if enable_camera:
            self.pub_camera = self.create_publisher(
                Image, '/camera/image_raw', 10)
            self.camera_timer = self.create_timer(1.0 / 30.0, self.publish_camera)

        # ── Sahte kendi telemetri (otopilot yerine) ──
        self.pub_own_telem = self.create_publisher(
            String, '/telemetry/own', 10)
        self.telem_timer = self.create_timer(0.5, self.publish_own_telem)

        # ── Durum ──
        self.npc_list = []
        self.frame_w = 640
        self.frame_h = 480
        self.own_lat = 41.51
        self.own_lon = 36.11
        self.own_alt = 60.0
        self.own_heading = 0.0
        self.own_speed = 20.0
        self.t = 0.0

        self.get_logger().info('╔═══════════════════════════════════════════╗')
        self.get_logger().info('║   TEST KÖPRÜSÜ BAŞLATILDI                ║')
        self.get_logger().info('║   Kamera: ' + ('AÇIK' if enable_camera else 'KAPALI') + '                          ║')
        self.get_logger().info('╚═══════════════════════════════════════════╝')

    def on_npc_data(self, msg):
        """NPC Publisher verisini al"""
        try:
            data = json.loads(msg.data)
            self.npc_list = data.get('konumBilgileri', [])
        except Exception:
            pass

    def publish_camera(self):
        """Sahte kamera görüntüsü üret — NPC dronları piksel olarak çiz"""
        frame = np.zeros((self.frame_h, self.frame_w, 3), dtype=np.uint8)

        # Gökyüzü gradyanı
        for y in range(self.frame_h):
            ratio = y / self.frame_h
            b = int(200 - ratio * 80)
            g = int(180 - ratio * 60)
            r = int(160 - ratio * 40)
            frame[y, :] = [b, g, r]

        # NPC İHA'larını kamera görüntüsüne yansıt
        for npc in self.npc_list[:5]:  # En yakın 5 tanesini göster
            lat = npc.get('iha_enlem', 0)
            lon = npc.get('iha_boylam', 0)
            alt = npc.get('iha_irtifa', 60)
            tid = npc.get('takim_numarasi', 0)

            # GPS → piksel dönüşümü (basitleştirilmiş projeksiyon)
            dx = (lon - self.own_lon) * 111320 * math.cos(math.radians(self.own_lat))
            dy = (lat - self.own_lat) * 111320
            dz = alt - self.own_alt

            # Mesafe
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist < 10 or dist > 500:
                continue

            # Bearing farkı → piksel X
            bearing = math.degrees(math.atan2(dx, dy))
            angle_diff = bearing - self.own_heading
            angle_diff = (angle_diff + 180) % 360 - 180

            if abs(angle_diff) > 35:  # FOV dışında
                continue

            px = int(self.frame_w / 2 + (angle_diff / 35.0) * (self.frame_w / 2))
            py = int(self.frame_h / 2 - (dz / dist) * self.frame_h)

            # Boyut (mesafeye göre)
            size = int(max(15, min(80, 2000 / dist)))

            # Drone şekli çiz
            px = max(size, min(self.frame_w - size, px))
            py = max(size, min(self.frame_h - size, py))

            # Gövde
            cv2.rectangle(frame,
                          (px - size//2, py - size//4),
                          (px + size//2, py + size//4),
                          (40, 40, 40), -1)
            # Kanatlar
            cv2.line(frame,
                     (px - size, py),
                     (px + size, py),
                     (60, 60, 60), max(2, size//8))
            # Kuyruk
            cv2.line(frame,
                     (px, py),
                     (px - size//3, py - size//2),
                     (60, 60, 60), max(1, size//10))
            cv2.line(frame,
                     (px, py),
                     (px + size//3, py - size//2),
                     (60, 60, 60), max(1, size//10))

            # ID yazısı
            cv2.putText(frame, f'T{tid}',
                        (px - 10, py - size//2 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 255, 0), 1)

        # Bilgi overlay
        now = datetime.now()
        time_str = now.strftime('%H:%M:%S.') + f'{now.microsecond // 1000:03d}'
        cv2.putText(frame, time_str,
                    (self.frame_w - 150, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f'NPC: {len(self.npc_list)}',
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f'ALT: {self.own_alt:.0f}m  HDG: {self.own_heading:.0f}',
                    (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Vuruş alanı çerçevesi (sarı)
        sz_x1 = int(self.frame_w * 0.25)
        sz_y1 = int(self.frame_h * 0.10)
        sz_x2 = int(self.frame_w * 0.75)
        sz_y2 = int(self.frame_h * 0.90)
        cv2.rectangle(frame, (sz_x1, sz_y1), (sz_x2, sz_y2), (0, 255, 255), 1)

        # Crosshair
        cx, cy = self.frame_w // 2, self.frame_h // 2
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

        # ROS2'ye yayınla
        ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_img.header.stamp = self.get_clock().now().to_msg()
        self.pub_camera.publish(ros_img)

        # Pencere göster
        if not self.headless:
            cv2.imshow('Test Camera | SIHA Autonomy', frame)
            cv2.waitKey(1)

    def publish_own_telem(self):
        """Sahte kendi İHA telemetrisi (çemberde uçuş)"""
        self.t += 0.5

        # Basit çember uçuşu (merkezde)
        radius = 0.002  # derece (~200m)
        self.own_heading = (self.t * 5) % 360
        self.own_lat = 41.51 + radius * math.cos(math.radians(self.own_heading))
        self.own_lon = 36.11 + radius * math.sin(math.radians(self.own_heading))

        telem = {
            'iha_enlem': round(self.own_lat, 7),
            'iha_boylam': round(self.own_lon, 7),
            'iha_irtifa': self.own_alt,
            'iha_yonelme': int(self.own_heading),
            'iha_hiz': self.own_speed,
            'iha_batarya': 85,
            'iha_otonom': 1,
            'iha_dikilme': 0.0,
            'iha_yatis': 5.0,
        }

        msg = String()
        msg.data = json.dumps(telem)
        self.pub_own_telem.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='SIHA Autonomy Test Bridge')
    parser.add_argument('--no-camera', action='store_true', help='Kamera yayını yapma')
    parser.add_argument('--headless', action='store_true', help='OpenCV penceresi gösterme')
    args = parser.parse_args()

    rclpy.init()
    node = TestBridge(
        enable_camera=not args.no_camera,
        headless=args.headless)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
