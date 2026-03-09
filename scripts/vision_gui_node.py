#!/usr/bin/env python3
"""
vision_gui_node.py — YOLO Tespit GUI'si
TEKNOFEST 2026 Savaşan İHA Yarışması

YOLO inference çalıştırır, tespit edilen nesneleri ekrana çizer ve
hem OpenCV penceresi hem de ROS2 image topic olarak yayınlar.

Yayınlar:
  /vision/gui  (sensor_msgs/Image) — GUI görüntüsü

Abonelikler:
  /camera/image_raw   (sensor_msgs/Image)  — kamera girişi
  /detection/image    (sensor_msgs/Image)  — alternatif kaynak
  /mission/state      (std_msgs/String)    — görev fazı
  /tracker/events     (std_msgs/String)    — kilitlenme durumu
  /decision/target    (std_msgs/String)    — seçilen hedef
  /decision/threats   (std_msgs/String)    — tehdit listesi
  /telemetry/own      (std_msgs/String)    — kendi telemetri

Kullanım:
  python3 scripts/vision_gui_node.py
  python3 scripts/vision_gui_node.py --headless
  python3 scripts/vision_gui_node.py --model path/to/model.pt
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False

import json
import time
import math
import argparse

import numpy as np
import cv2

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ─── Sabitler ──────────────────────────────────────────────────────────────
LOCKON_DURATION_S        = 4.0    # Şartname gereği 4 saniyelik kilit
MIN_DETECTION_CONFIDENCE = 0.35   # YOLO tespit güven eşiği
AIRPLANE_CLASSES         = {'airplane', 'aeroplane', 'plane', 'drone', 'uav'}
STRIKE_COLOR       = (0, 0, 255)   # Kırmızı (BGR) — şartname uyumlu
CROSSHAIR_COLOR    = (0, 255, 0)
INFO_BG_COLOR      = (0, 0, 0)
INFO_TEXT_COLOR    = (255, 255, 255)
WARN_COLOR         = (0, 165, 255)


class VisionGuiNode(Node):
    def __init__(self, model_path: str = 'yolov8n.pt', headless: bool = False):
        super().__init__('vision_gui')

        self.headless   = headless
        self.model_path = model_path
        self.bridge     = CvBridge() if CV_BRIDGE else None

        # YOLO modeli yükle
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO modeli yüklendi: {model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO yüklenemedi: {e}')
        else:
            self.get_logger().warn('ultralytics bulunamadı — YOLO devre dışı')

        # Durum
        self.mission_phase  = 'UNKNOWN'
        self.own_telem: dict = {}
        self.target_info: dict = {}
        self.threats: list = []
        self.lock_start: float = 0.0
        self.lock_active: bool = False
        self.total_locks: int  = 0
        self.last_detections: list = []

        # Metrikler
        self._fps_times: list = []
        self._last_inference_ms: float = 0.0

        # Yayıncılar
        self.pub_gui = self.create_publisher(Image, '/vision/gui', 10)

        # Abonelikler
        self._setup_subscriptions()

        self.get_logger().info(f'Vision GUI başlatıldı (headless={headless})')

    def _setup_subscriptions(self):
        # Kamera girişi
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 1)
        self.create_subscription(Image, '/detection/image',  self._on_image, 1)
        # Durum bilgileri
        self.create_subscription(String, '/mission/state',  self._on_mission_state, 10)
        self.create_subscription(String, '/tracker/events', self._on_tracker,       10)
        self.create_subscription(String, '/decision/target', self._on_target,       10)
        self.create_subscription(String, '/decision/threats',self._on_threats,      10)
        self.create_subscription(String, '/telemetry/own',  self._on_own_telem,     10)

    # ─── Geri Çağırmalar ───────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self._process_frame(frame)

    def _on_mission_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.mission_phase = d.get('phase', 'UNKNOWN')
            self.total_locks   = d.get('total_locks', self.total_locks)
        except Exception:
            self.mission_phase = msg.data

    def _on_tracker(self, msg: String):
        if msg.data.startswith('LOCK_COMPLETE:'):
            self.lock_active = False
            self.total_locks += 1
        elif msg.data == 'LOCK_START':
            self.lock_start  = time.time()
            self.lock_active = True
        elif msg.data == 'LOCK_LOST':
            self.lock_active = False

    def _on_target(self, msg: String):
        try:
            self.target_info = json.loads(msg.data)
        except Exception:
            pass

    def _on_threats(self, msg: String):
        try:
            self.threats = json.loads(msg.data)
        except Exception:
            self.threats = []

    def _on_own_telem(self, msg: String):
        try:
            self.own_telem = json.loads(msg.data)
        except Exception:
            pass

    # ─── Görüntü İşleme ────────────────────────────────────────────────────

    def _process_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        t0 = time.time()

        # YOLO inference
        detections = []
        if self.model is not None:
            try:
                results = self.model(frame, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.model.names.get(cls_id, '').lower()
                        conf = float(box.conf[0])
                        if conf < MIN_DETECTION_CONFIDENCE:
                            continue
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detections.append({
                            'cls': cls_name,
                            'conf': conf,
                            'bbox': (x1, y1, x2, y2),
                            'is_target': cls_name in AIRPLANE_CLASSES,
                        })
            except Exception as e:
                self.get_logger().error(f'YOLO hatası: {e}')

        self._last_inference_ms = (time.time() - t0) * 1000
        self.last_detections = detections

        # Çizim
        canvas = frame.copy()
        self._draw_detections(canvas, detections, w, h)
        self._draw_strike_zone(canvas, w, h)
        self._draw_crosshair(canvas, w, h)
        self._draw_info_panel(canvas, w, h, len(detections))

        # FPS hesapla
        now = time.time()
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        self._fps_times.append(now)

        # Yayınla
        if self.bridge is not None:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.pub_gui.publish(ros_img)
            except Exception:
                pass

        if not self.headless:
            cv2.imshow('SIHA Vision GUI', canvas)
            cv2.waitKey(1)

    def _draw_detections(self, canvas, dets, w, h):
        for det in dets:
            x1, y1, x2, y2 = det['bbox']
            conf = det['conf']
            cls  = det['cls']
            is_target = det['is_target']

            # Bounding box rengi
            color = STRIKE_COLOR if is_target else (0, 200, 0)
            thickness = 3 if is_target else 2
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)

            # Etiket
            label = f'{cls} {conf:.0%}'
            label_y = max(y1 - 6, 14)
            cv2.putText(canvas, label, (x1, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            # Kilitlenme göstergesi
            if is_target and self.lock_active:
                elapsed = time.time() - self.lock_start
                pct = min(elapsed / LOCKON_DURATION_S, 1.0)
                bar_w = int((x2 - x1) * pct)
                cv2.rectangle(canvas, (x1, y2 + 4), (x1 + bar_w, y2 + 10),
                               (0, 255, 255), -1)
                cv2.rectangle(canvas, (x1, y2 + 4), (x2, y2 + 10),
                               (0, 255, 255), 1)

    def _draw_strike_zone(self, canvas, w, h):
        """Şartname uyumlu vuruş alanı çerçevesi"""
        sz_x1 = int(w * 0.25)
        sz_y1 = int(h * 0.10)
        sz_x2 = int(w * 0.75)
        sz_y2 = int(h * 0.90)
        cv2.rectangle(canvas, (sz_x1, sz_y1), (sz_x2, sz_y2),
                      (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, 'VURUS ALANI', (sz_x1 + 4, sz_y1 + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    def _draw_crosshair(self, canvas, w, h):
        cx, cy = w // 2, h // 2
        size = 18
        cv2.line(canvas, (cx - size, cy), (cx + size, cy), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
        cv2.line(canvas, (cx, cy - size), (cx, cy + size), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), size // 2, CROSSHAIR_COLOR, 1, cv2.LINE_AA)

    def _draw_info_panel(self, canvas, w, h, det_count):
        """Sol üst bilgi paneli"""
        panel_w = 260
        panel_h = 220
        overlay = canvas.copy()
        cv2.rectangle(overlay, (4, 4), (panel_w, panel_h), INFO_BG_COLOR, -1)
        cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)

        fps = len(self._fps_times)
        lock_pct = 0.0
        if self.lock_active:
            lock_pct = min((time.time() - self.lock_start) / LOCKON_DURATION_S, 1.0)

        # Durum rengi
        phase = self.mission_phase
        if phase in ('EVADE',):
            phase_color = (0, 0, 255)
        elif phase in ('LOCK', 'CHASE'):
            phase_color = (0, 200, 255)
        elif phase in ('SEARCH',):
            phase_color = (0, 255, 0)
        else:
            phase_color = INFO_TEXT_COLOR

        lines = [
            (f'DURUM : {phase}',            phase_color),
            (f'KİLİT : {"AKTIF" if self.lock_active else "YOK"}  {lock_pct:.0%}',
             (0, 255, 255) if self.lock_active else INFO_TEXT_COLOR),
            (f'TESPIT: {det_count}  KILIT: {self.total_locks}',  INFO_TEXT_COLOR),
            (f'FPS   : {fps}  YOLO: {self._last_inference_ms:.0f}ms', INFO_TEXT_COLOR),
            (f'TEHDİT: {len(self.threats)}', WARN_COLOR if self.threats else INFO_TEXT_COLOR),
        ]

        # Telemetri satırları
        if self.own_telem:
            lat = self.own_telem.get('iha_enlem',   0.0)
            lon = self.own_telem.get('iha_boylam',  0.0)
            alt = self.own_telem.get('iha_irtifa',  0.0)
            hdg = self.own_telem.get('iha_yonelme', 0.0)
            spd = self.own_telem.get('iha_hiz',     0.0)
            lines += [
                (f'LAT: {lat:.5f}  LON: {lon:.5f}', INFO_TEXT_COLOR),
                (f'ALT: {alt:.0f}m  HDG: {hdg:.0f}°  HİZ: {spd:.0f}m/s', INFO_TEXT_COLOR),
            ]

        if self.target_info:
            tdist = self.target_info.get('mesafe_m', 0)
            tid   = self.target_info.get('takim_numarasi', '?')
            lines.append((f'HEDEF: T{tid}  {tdist:.0f}m', (100, 255, 100)))

        for i, (text, color) in enumerate(lines):
            y = 22 + i * 20
            cv2.putText(canvas, text, (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # Kilitlenme progress bar (sol alt)
        if self.lock_active:
            bar_total = 200
            bar_filled = int(bar_total * lock_pct)
            by = h - 20
            cv2.rectangle(canvas, (8, by), (8 + bar_total, by + 12), (60, 60, 60), -1)
            cv2.rectangle(canvas, (8, by), (8 + bar_filled, by + 12), (0, 255, 255), -1)
            cv2.putText(canvas, f'KİLİT {lock_pct:.0%}', (12, by + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    def destroy_node(self):
        if not self.headless:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='SIHA Vision GUI')
    parser.add_argument('--headless', action='store_true',
                        help='OpenCV penceresi gösterme')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='YOLO model dosyası')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = VisionGuiNode(model_path=args.model, headless=args.headless)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
