#!/usr/bin/env python3
"""
tracker_node.py — Gelişmiş Hedef Takip Modülü (Kalman Filter)
TEKNOFEST 2026 Savaşan İHA Yarışması

Kamera görüntüsünde tespit edilen hedefi Kalman filter ile takip eder.
Şartname uyumlu 4 saniyelik kilitlenme mantığını uygular.

Yayınlar:
  /tracker/target  (std_msgs/String) — piksel hedef koordinatı
  /tracker/events  (std_msgs/String) — LOCK_START, LOCK_COMPLETE:<id>, LOCK_LOST

Abonelikler:
  /camera/image_raw  (sensor_msgs/Image)  — kamera girişi
  /detection/image   (sensor_msgs/Image)  — alternatif kaynak
  /decision/target   (std_msgs/String)    — seçilen hedef (GPS + ID)
  /mission/state     (std_msgs/String)    — görev fazı

Kullanım:
  python3 scripts/tracker_node.py
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


# ─── Kalman Filter için basit tracker ──────────────────────────────────────

class KalmanTracker:
    """2D piksel uzayında x, y, vx, vy durumlu Kalman filter."""

    def __init__(self, x: float, y: float):
        self.kf = cv2.KalmanFilter(4, 2)

        # Durum geçiş matrisi [x, y, vx, vy]
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        # Ölçüm matrisi (x, y gözlemlenebilir)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)

        self.kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kf.errorCovPost        = np.eye(4, dtype=np.float32)

        # Başlangıç durumu
        self.kf.statePost = np.array([[x], [y], [0.], [0.]], dtype=np.float32)

        self.last_update = time.time()

    def predict(self):
        pred = self.kf.predict()
        return float(pred[0][0]), float(pred[1][0])

    def update(self, x: float, y: float):
        self.kf.predict()   # Kalman filtre predict → correct döngüsü
        meas = np.array([[x], [y]], dtype=np.float32)
        self.kf.correct(meas)
        self.last_update = time.time()

    @property
    def state(self):
        s = self.kf.statePost
        return float(s[0][0]), float(s[1][0])


# ─── Sabitler ──────────────────────────────────────────────────────────────
LOCKON_DURATION_S        = 4.0
LOCKON_TOLERANCE_S       = 1.0    # Kilitlenme sırasında izin verilen kayıp süresi
FRAME_LOSS_TOLERANCE     = 0.05   # Saniyede izin verilen kayıp oranı
MIN_DETECTION_CONFIDENCE = 0.35   # YOLO tespit güven eşiği
AIRPLANE_CLASSES         = {'airplane', 'aeroplane', 'plane', 'drone', 'uav'}
MAX_TRACKER_AGE_S        = 2.0    # Bu süre güncellenmezse tracker sıfırlanır
LOCK_IOU_THRESHOLD       = 0.3    # Hedef eşleşme için minimum IoU


class TrackerNode(Node):
    def __init__(self, model_path: str = 'yolov8n.pt'):
        super().__init__('tracker_node')

        self.bridge     = CvBridge() if CV_BRIDGE else None
        self.model      = None

        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO yüklendi: {model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO yüklenemedi: {e}')

        # Takip durumu
        self.tracker: KalmanTracker | None = None
        self.track_bbox  = None       # (x1,y1,x2,y2)
        self.lock_start  = 0.0
        self.lock_active = False
        self.last_det_time = 0.0
        self.loss_start  = 0.0
        self.mission_phase = 'UNKNOWN'
        self.current_target_id = -1
        self.last_locked_id    = -1

        # Yayıncılar
        self.pub_target = self.create_publisher(String, '/tracker/target', 10)
        self.pub_events = self.create_publisher(String, '/tracker/events', 10)

        # Abonelikler
        self.create_subscription(Image,  '/camera/image_raw',  self._on_image,   1)
        self.create_subscription(Image,  '/detection/image',   self._on_image,   1)
        self.create_subscription(String, '/decision/target',   self._on_target,  10)
        self.create_subscription(String, '/mission/state',     self._on_mission, 10)

        self.get_logger().info('Tracker node başlatıldı')

    # ─── Geri Çağırmalar ───────────────────────────────────────────────────

    def _on_mission(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.mission_phase = d.get('phase', 'UNKNOWN')
        except Exception:
            pass

    def _on_target(self, msg: String):
        try:
            d = json.loads(msg.data)
            new_id = d.get('takim_numarasi', -1)
            if new_id != self.current_target_id:
                self.current_target_id = new_id
                # Yeni hedef — tracker sıfırla
                if self.lock_active:
                    self._reset_lock()
                self.tracker = None
        except Exception:
            pass

    def _on_image(self, msg: Image):
        if self.bridge is None or self.model is None:
            return
        if self.mission_phase not in ('SEARCH', 'CHASE', 'LOCK'):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self._process(frame)

    # ─── Ana İşleme ────────────────────────────────────────────────────────

    def _process(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        now  = time.time()

        # YOLO tespit
        detections = []
        try:
            results = self.model(frame, verbose=False)
            for r in results:
                for box in r.boxes:
                    cls_id   = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, '').lower()
                    conf     = float(box.conf[0])
                    if cls_name not in AIRPLANE_CLASSES or conf < MIN_DETECTION_CONFIDENCE:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    detections.append((x1, y1, x2, y2, conf))
        except Exception as e:
            self.get_logger().error(f'Tespit hatası: {e}')
            return

        # En iyi tespit seç (tracker ile eşleştir)
        best_bbox = self._match_detection(detections)

        if best_bbox is not None:
            x1, y1, x2, y2, _ = best_bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            if self.tracker is None:
                self.tracker = KalmanTracker(cx, cy)
            else:
                self.tracker.update(cx, cy)

            self.track_bbox    = (x1, y1, x2, y2)
            self.last_det_time = now
            self.loss_start    = 0.0
        else:
            # Hedef kaybedildi
            if self.tracker is not None:
                if self.loss_start == 0.0:
                    self.loss_start = now
                loss_dur = now - self.loss_start
                if loss_dur > MAX_TRACKER_AGE_S:
                    self._handle_loss()
                else:
                    # Kalman tahmini kullan
                    self.tracker.predict()
            else:
                return

        # Kilitlenme mantığı
        if self.tracker is not None:
            self._update_lock(now, w, h)
            self._publish_target(now, w, h)

    def _match_detection(self, dets):
        """Tracker bbox ile en yüksek IoU'lu tespiti döndür"""
        if not dets:
            return None
        if self.track_bbox is None:
            # İlk tespit — en büyük bbox'u al
            dets_sorted = sorted(dets, key=lambda d: (d[2]-d[0])*(d[3]-d[1]), reverse=True)
            return dets_sorted[0]

        tx1, ty1, tx2, ty2 = self.track_bbox
        best_iou = LOCK_IOU_THRESHOLD - 0.01  # Eşik altı başlangıç
        best_det = None
        for det in dets:
            x1, y1, x2, y2, _ = det
            iou = self._iou(tx1, ty1, tx2, ty2, x1, y1, x2, y2)
            if iou > best_iou:
                best_iou = iou
                best_det = det
        # Eşleşme bulunamazsa en yakın tespiti dene
        if best_det is None and dets:
            tcx = (tx1 + tx2) / 2
            tcy = (ty1 + ty2) / 2
            best_det = min(dets, key=lambda d: math.hypot(
                (d[0]+d[2])/2 - tcx, (d[1]+d[3])/2 - tcy))
        return best_det

    @staticmethod
    def _iou(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2) -> float:
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        union = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
        return inter / union if union > 0 else 0.0

    def _update_lock(self, now: float, w: int, h: int):
        """Kilitlenme zamanlayıcısını güncelle"""
        if not self.lock_active:
            # Kilitlenme başlat
            if self.current_target_id == self.last_locked_id:
                return   # Aynı İHA'ya tekrar kilit yasak
            self.lock_active = True
            self.lock_start  = now
            self._emit('LOCK_START')
            self.get_logger().info(
                f'[KİLİT] Kilitlenme başladı: T{self.current_target_id}')
        else:
            elapsed = now - self.lock_start
            if elapsed >= LOCKON_DURATION_S:
                # Kilitlenme tamamlandı
                self._emit(f'LOCK_COMPLETE:{self.current_target_id}')
                self.last_locked_id = self.current_target_id
                self.get_logger().info(
                    f'[KİLİT] TAMAMLANDI: T{self.current_target_id}  '
                    f'süre={elapsed:.1f}s')
                self._reset_lock()

    def _reset_lock(self):
        self.lock_active = False
        self.lock_start  = 0.0

    def _handle_loss(self):
        """Hedef kaybedildi"""
        self.get_logger().warn('[TAKİP] Hedef kaybedildi')
        self._emit('LOCK_LOST')
        self._reset_lock()
        self.tracker   = None
        self.track_bbox = None
        self.loss_start = 0.0

    def _emit(self, event: str):
        m = String()
        m.data = event
        self.pub_events.publish(m)

    def _publish_target(self, now: float, w: int, h: int):
        if self.tracker is None:
            return
        px, py = self.tracker.state
        lock_pct = 0.0
        if self.lock_active:
            lock_pct = min((now - self.lock_start) / LOCKON_DURATION_S, 1.0)

        # Normalize [-1, 1]
        nx = (px - w / 2) / (w / 2)
        ny = (py - h / 2) / (h / 2)

        m = String()
        m.data = json.dumps({
            'px': round(px, 1),
            'py': round(py, 1),
            'nx': round(nx, 3),
            'ny': round(ny, 3),
            'lock_pct': round(lock_pct, 3),
            'lock_active': self.lock_active,
            'target_id': self.current_target_id,
        })
        self.pub_target.publish(m)


def main():
    parser = argparse.ArgumentParser(description='SIHA Tracker Node')
    parser.add_argument('--model', default='yolov8n.pt', help='YOLO model dosyası')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TrackerNode(model_path=args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
