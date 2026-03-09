# SIHA Otonom Sistem — scripts/ Kullanım Kılavuzu

> TEKNOFEST 2026 Savaşan İHA Yarışması  
> Python tabanlı ROS2 Jazzy node'ları

---

## İçindekiler

1. [Genel Bakış](#genel-bakış)
2. [Dosya Yapısı](#dosya-yapısı)
3. [Bağımlılıklar & Kurulum](#bağımlılıklar--kurulum)
4. [Hızlı Başlangıç](#hızlı-başlangıç)
5. [Node Açıklamaları](#node-açıklamaları)
6. [ROS2 Topic Referansı](#ros2-topic-referansı)
7. [YOLO Modeli](#yolo-modeli)
8. [Sorun Giderme](#sorun-giderme)

---

## Genel Bakış

`scripts/` dizini, `siha_autonomy_v3/` C++ paketinin yanında çalışan Python
tabanlı yüksek seviyeli otonom karar, takip ve yönlendirme node'larını içerir.

```
C++ katmanı (siha_autonomy_v3): Düşük seviye uçuş, görüntü pipeline, kayıt
Python katmanı (scripts/)     : Otonom karar, akıllı hedef seçimi, GUI
```

### Eklenen Özellikler

| Özellik | Durum |
|---------|-------|
| Otomatik ARM → GUIDED → TAKEOFF | ✅ `mission_manager_node.py` |
| Durum makinesi (10 faz) | ✅ `mission_manager_node.py` |
| Akıllı hedef seçimi (skor bazlı) | ✅ `decision_node.py` |
| Arkadan yaklaşma tespiti | ✅ `decision_node.py` |
| Kaçınma manevraları | ✅ `decision_node.py` + `guidance_node.py` |
| YOLO GUI ekranı | ✅ `vision_gui_node.py` |
| Kalman filter takibi | ✅ `tracker_node.py` |
| 4 saniyelik kilit + yasak | ✅ `tracker_node.py` |

---

## Dosya Yapısı

```
scripts/
├── mission_manager_node.py  # Otomatik takeoff + görev akışı
├── decision_node.py         # Hedef seçimi + kaçınma modülü
├── vision_gui_node.py       # YOLO GUI + feedback ekranı
├── tracker_node.py          # Kalman filter takibi (4s kilit)
├── guidance_node.py         # Yönlendirme (GPS / Piksel / Kaçınma)
├── mavlink_bridge.py        # Düzenlenmiş MAVLink köprüsü
├── launch_all.sh            # Tek komutla başlatıcı
├── requirements.txt         # Python bağımlılıkları
└── README.md                # Bu dosya
```

---

## Bağımlılıklar & Kurulum

### Sistem Gereksinimleri

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Python 3.10+

### ROS2 Kurulumu

```bash
sudo apt install \
  ros-jazzy-rclpy \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-cv-bridge
```

### Python Bağımlılıkları

```bash
pip install -r scripts/requirements.txt
```

---

## Hızlı Başlangıç

### Tüm Node'ları Başlat

```bash
chmod +x scripts/launch_all.sh
./scripts/launch_all.sh
```

#### Seçenekler

```
--headless          OpenCV penceresi gösterme (sunucu modunda)
--no-gui            Vision GUI node'unu başlatma
--sitl-url URL      SITL bağlantı URL'i (varsayılan: udp:127.0.0.1:14550)
--takeoff-alt METRE Kalkış irtifası (varsayılan: 30m)
--model DOSYA       YOLO model dosyası (varsayılan: yolov8n.pt)
```

#### Örnekler

```bash
# Sunucu modunda, 40m'de kalkış
./scripts/launch_all.sh --headless --takeoff-alt 40.0

# Özel model ile
./scripts/launch_all.sh --model models/drone_detector.pt

# SITL TCP bağlantısı
./scripts/launch_all.sh --sitl-url tcp:127.0.0.1:5760
```

### Node'ları Ayrı Ayrı Başlat

```bash
# Terminal 1: MAVLink Bridge
python3 scripts/mavlink_bridge.py --url udp:127.0.0.1:14550

# Terminal 2: Görev Yöneticisi
python3 scripts/mission_manager_node.py --takeoff-alt 30.0

# Terminal 3: Karar Modülü
python3 scripts/decision_node.py

# Terminal 4: Tracker
python3 scripts/tracker_node.py --model yolov8n.pt

# Terminal 5: Guidance
python3 scripts/guidance_node.py

# Terminal 6: Vision GUI
python3 scripts/vision_gui_node.py --model yolov8n.pt
# Sunucu modu:
python3 scripts/vision_gui_node.py --headless
```

---

## Node Açıklamaları

### mission_manager_node.py

Görev akışını yöneten durum makinesi. IDLE'dan başlayarak otomatik ARM,
GUIDED moda geçiş ve TAKEOFF komutlarını gönderir.

**Durum Makinesi:**

```
IDLE → PRE_ARM → ARM → TAKEOFF → CLIMB → SEARCH → CHASE → LOCK → EVADE → RTL
```

| Faz | Açıklama |
|-----|---------|
| `IDLE` | Sistem başlangıcı, telemetri bekleniyor |
| `PRE_ARM` | GUIDED mod aktif ediliyor |
| `ARM` | Motor ARM komutu gönderildi |
| `TAKEOFF` | Kalkış komutu gönderildi |
| `CLIMB` | Hedef irtifaya tırmanılıyor |
| `SEARCH` | Rakip İHA aranıyor |
| `CHASE` | Hedef takip ediliyor |
| `LOCK` | 4 saniyelik kilitlenme süreci |
| `EVADE` | Kaçınma manevrasına girildi |
| `RTL` | Geri dönüş |

---

### decision_node.py

Sunucu telemetrisinden gelen 15 rakip drone için skor hesaplar ve en iyi
hedefi seçer. Aynı zamanda arkadan yaklaşan drone'ları tehdit olarak tespit eder.

**Hedef Skor Faktörleri:**

| Faktör | Ağırlık |
|--------|---------|
| Mesafe skoru | 0-40 puan |
| Açı farkı skoru | -20 ile +30 puan |
| Manevra maliyeti | 0 ile -20 puan |
| Tekrar kilit cezası | -50 puan |
| Ulaşma süresi | 0-10 puan |

**Tehdit Koşulları:**
- Mesafe < 50m
- Yaklaşma hızı > 5 m/s  
- Açı farkı > ±120° (arkadan geliyor)

---

### vision_gui_node.py

Kamera görüntüsü üzerine YOLO tespitleri ve bilgi paneli çizer.

**Ekran Öğeleri:**
- Bounding box (kırmızı #FF0000 — şartname uyumlu)
- Confidence değeri
- Crosshair (ekran merkezi)
- Vuruş alanı çerçevesi (sarı)
- Kilitlenme progress bar (0-4 saniye)
- Bilgi paneli: faz, tespit sayısı, kilit sayısı, FPS, tehdit sayısı
- Telemetri: lat, lon, alt, heading, hız

---

### tracker_node.py

YOLO tespitlerini Kalman filter ile takip eder.

**Özellikler:**
- `cv2.KalmanFilter` ile x, y, vx, vy durum tahmini
- IoU tabanlı tespit eşleşmesi
- Frame loss toleransı (2 saniye)
- Şartname uyumlu 4 saniyelik kilitlenme
- Aynı İHA'ya arka arkaya kilit yasağı

---

### guidance_node.py

Görev fazına göre farklı yönlendirme modları uygular.

| Faz | Mod | Komut |
|-----|-----|-------|
| `SEARCH` | GPS tabanlı | `/mavlink/cmd/goto` |
| `CHASE`/`LOCK` | PID piksel takibi | `/mavlink/cmd/velocity` + `/mavlink/cmd/yaw` |
| `EVADE` | Kaçınma vektörü | `/mavlink/cmd/velocity` + `/mavlink/cmd/yaw` |

---

### mavlink_bridge.py

Tam otomatik kalkış dizisi:

```
1. GUIDED mod aktif
2. ARM komutu
3. TAKEOFF komutu
4. İrtifa kontrolü (hedef * 0.9 eşiği)
```

**RC Override Koruması:** Son velocity komutundan 2 saniye geçerse
yeni komut gönderilmez.

---

## ROS2 Topic Referansı

### Yayınlanan Topic'ler

| Topic | Tip | Node | Açıklama |
|-------|-----|------|---------|
| `/mission/state` | `std_msgs/String` | mission_manager | Görev fazı (JSON) |
| `/decision/target` | `std_msgs/String` | decision | Seçilen hedef (GPS + skor) |
| `/decision/threats` | `std_msgs/String` | decision | Aktif tehdit listesi |
| `/vision/gui` | `sensor_msgs/Image` | vision_gui | GUI görüntüsü |
| `/tracker/target` | `std_msgs/String` | tracker | Piksel hedef (nx, ny) |
| `/tracker/events` | `std_msgs/String` | tracker | Kilit olayları |
| `/mavlink/telemetry` | `std_msgs/String` | mavlink_bridge | MAVLink telemetri |

### Abone Olunan Topic'ler

| Topic | Tip | Kaynak | Açıklama |
|-------|-----|--------|---------|
| `/sunucu_telemetri` | `std_msgs/String` | NPC Publisher | 15 rakip İHA konumu |
| `/telemetry/own` | `std_msgs/String` | test_bridge / SITL | Kendi telemetri |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo / kamera | Kamera görüntüsü |
| `/mavlink/telemetry` | `std_msgs/String` | mavlink_bridge | MAVLink telemetri |

### /mission/state Format

```json
{
  "phase": "SEARCH",
  "alt": 61.5,
  "armed": true,
  "mode": "GUIDED",
  "total_locks": 3,
  "threats": 0
}
```

### /decision/target Format

```json
{
  "takim_numarasi": 7,
  "lat": 41.5123,
  "lon": 36.1145,
  "alt": 58.0,
  "mesafe_m": 340.5,
  "skor": 52.3
}
```

### /tracker/events Değerleri

| Event | Açıklama |
|-------|---------|
| `LOCK_START` | Kilitlenme başladı |
| `LOCK_COMPLETE:<id>` | Kilitlenme tamamlandı (drone ID) |
| `LOCK_LOST` | Hedef kaybedildi |

---

## YOLO Modeli

### Mevcut Model

Varsayılan olarak `yolov8n.pt` kullanılır (Ultralytics genel COCO model).
`airplane` class'ını (class ID: 4) tespit eder.

### Özel Model Eğitimi

Yarışma performansı için drone'a özel model eğitimi önerilir:

```bash
# 1. Veri seti hazırla (YOLO formatında: images/ + labels/)
# 2. data.yaml oluştur
# 3. Eğit
yolo detect train \
  data=data.yaml \
  model=yolov8n.pt \
  epochs=100 \
  imgsz=640 \
  batch=16

# 4. Özel modeli kullan
./scripts/launch_all.sh --model runs/detect/train/weights/best.pt
```

**Önerilen veri seti kaynakları:**
- [Roboflow Universe — Drone Detection](https://universe.roboflow.com)
- TEKNOFEST geçmiş yarışma görüntüleri

### ONNX'e Dışa Aktarma (opsiyonel, C++ tarafı için)

```bash
yolo export model=best.pt format=onnx opset=12 simplify=True
```

---

## Sorun Giderme

### MAVLink bağlantısı kurulamıyor

```bash
# SITL çalışıyor mu kontrol et
nc -z 127.0.0.1 14550

# Farklı port dene
python3 scripts/mavlink_bridge.py --url tcp:127.0.0.1:5760
```

### YOLO modeli yüklenemiyor

```bash
# Bağımlılıkları kur
pip install ultralytics>=8.2.0

# Model dosyasını indir
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### cv_bridge ImportError

```bash
sudo apt install ros-jazzy-cv-bridge python3-cv-bridge
```

### Topic yayınlanmıyor

```bash
# Node'ların çalışıp çalışmadığını kontrol et
ros2 node list

# Topic'leri listele
ros2 topic list

# Belirli topic'i dinle
ros2 topic echo /mission/state
```
