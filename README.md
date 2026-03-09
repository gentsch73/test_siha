# SAEROTECH SİHA — TEKNOFEST 2026 Savaşan İHA Otonom Sistemi

<div align="center">

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![C++17](https://img.shields.io/badge/C%2B%2B-17-green)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

</div>

TEKNOFEST 2026 Savaşan İHA yarışması için geliştirilmiş tam otonom insansız hava aracı sistemi. C++17 ve ROS2 Jazzy tabanlı, ArduPlane + Gazebo SITL desteği.

---

## 📋 İçindekiler

- [Genel Bakış](#genel-bakış)
- [Mimari](#mimari)
- [Gereksinimler](#gereksinimler)
- [Kurulum](#kurulum)
- [Derleme](#derleme)
- [Çalıştırma](#çalıştırma)
- [ROS2 Topic'leri](#ros2-topicler)
- [Konfigürasyon](#konfigürasyon)
- [Dizin Yapısı](#dizin-yapısı)

---

## Genel Bakış

Bu sistem şu yeteneklere sahiptir:

| Yetenek | Açıklama |
|---------|----------|
| **Otonom Kalkış** | ARM → GUIDED mod → TAKEOFF otomatik sırası |
| **Hedef Tespiti** | YOLOv8n ONNX ile gerçek zamanlı nesne tespiti |
| **Hedef Takibi** | Kalman filtresi tabanlı çoklu hedef takibi (SORT) |
| **Akıllı Hedef Seçimi** | Manevra maliyeti ve mesafe skoru tabanlı karar motoru |
| **Şartname Kilitlenme** | 4 saniyelik kilitlenme, kırmızı dikdörtgen, %5 alan şartı |
| **Kaçınma** | Arkadan yaklaşan tehditlere otomatik kaçınma manevrası |
| **Sınır Koruma** | GPS bazlı arena sınırı takibi ve ihlal önleme |
| **Güvenlik** | 8 paralel güvenlik kontrolü (batarya, GPS, haberleşme...) |
| **Video Kayıt** | H.264 MP4 kayıt + kilitlenme overlay'i |
| **Debug Görüntü** | `/vision/debug_image` üzerinden canlı overlay |

---

## Mimari

```
┌─────────────────────────────────────────────────────────────┐
│                    MissionController (50 Hz)                 │
│  IDLE → PRE_ARM → ARMED → TAKEOFF → CLIMB → SEARCH         │
│  → TRACK → LOCKON → EVADE → RTL → LAND → EMERGENCY         │
└──────────┬────────────────────────────────────────┬─────────┘
           │                                        │
    ┌──────▼──────┐                        ┌───────▼──────┐
    │ Vision Pipeline│                     │FlightController│
    │  YoloDetector  │                     │  MAVLink stub  │
    │  TargetTracker │                     │  + ROS2 pub    │
    │  LockonManager │                     └───────┬──────┘
    │  Debug Overlay │                             │
    └──────────────┘                     ┌────────▼──────┐
                                         │ mavlink_bridge │
    ┌──────────────┐                     │   (Python)     │
    │ DecisionEngine│                    │ ArduPlane SITL │
    │ Hedef Seçimi  │                    └───────────────┘
    │ Kaçınma Algo  │
    └──────────────┘
```

---

## Gereksinimler

### Sistem
- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Python 3.12+

### ROS2 Paketleri
```bash
sudo apt install \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport
```

### Diğer Bağımlılıklar
```bash
# OpenCV
sudo apt install libopencv-dev python3-opencv

# Python (MAVLink bridge)
pip install pymavlink

# ArduPilot SITL (simülasyon)
pip install MAVProxy
sim_vehicle.py --help  # ardupilot kuruluysa

# Gazebo Harmonic (simülasyon)
sudo apt install gazebo ros-jazzy-ros-gz-bridge
```

---

## Kurulum

### 1. Repository Klonla
```bash
git clone https://github.com/gentsch73/test_siha.git
cd test_siha
```

### 2. ROS2 Workspace Oluştur
```bash
mkdir -p ~/ros2_ws/src
cp -r siha_autonomy ~/ros2_ws/src/
cd ~/ros2_ws
```

### 3. Bağımlılıkları Kur
```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Derleme

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash

# Release modda derle
colcon build --packages-select siha_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release

# Workspace'i aktif et
source install/setup.bash
```

---

## Çalıştırma

### Hızlı Başlangıç (Önerilen)

```bash
# Tüm sistemi tek komutla başlat
ros2 launch siha_autonomy siha_launch.py

# Seçeneklerle başlat
ros2 launch siha_autonomy siha_launch.py \
  sim:=true \
  takeoff_alt:=40.0 \
  team_id:=1 \
  server_ip:=192.168.1.100
```

### Manuel Başlatma (Çoklu Terminal)

**Terminal 1: ArduPlane SITL**
```bash
sim_vehicle.py -v ArduPlane -L 41.51,36.11,50,0 --map --console
```

**Terminal 2: Gazebo Simülasyon**
```bash
gz sim vtail_runway_savasan.sdf
```

**Terminal 3: Kamera Köprüsü**
```bash
python3 gz_camera_bridge.py
```

**Terminal 4: MAVLink Bridge**
```bash
python3 mavlink_bridge.py --url udp:127.0.0.1:14550
```

**Terminal 5: C++ Ana Node**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run siha_autonomy main_node \
  --ros-args \
  -p simulation:=true \
  -p team_id:=1
```

**Terminal 6: Debug Görüntü (opsiyonel)**
```bash
ros2 run rqt_image_view rqt_image_view
# Sol üstten /vision/debug_image seç
```

---

## ROS2 Topic'leri

### Yayınlanan (Published)

| Topic | Tip | Hz | Açıklama |
|-------|-----|----|----------|
| `/vision/debug_image` | `sensor_msgs/Image` | 30 | YOLO kutuları + kilitlenme overlay |
| `/decision/target` | `std_msgs/String` | 1 | Seçilen hedef (JSON) |
| `/decision/threats` | `std_msgs/String` | 5 | Aktif tehditler (JSON) |
| `/decision/evade` | `std_msgs/String` | olay | Kaçınma komutu (JSON) |
| `/mavlink/cmd/arm` | `std_msgs/String` | olay | Arm/Disarm komutu |
| `/mavlink/cmd/mode` | `std_msgs/String` | olay | Uçuş modu değişikliği |
| `/mavlink/cmd/takeoff` | `std_msgs/String` | olay | Kalkış komutu |

### Dinlenen (Subscribed)

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/sunucu_telemetri` | `std_msgs/String` | 15 rakip İHA konumu (JSON) |
| `/mavlink/telemetry` | `std_msgs/String` | Kendi telemetri verisi (bridge'den) |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo kamera görüntüsü |

### Sunucu Telemetri Formatı

```json
{
  "konumBilgileri": [
    {
      "takim_numarasi": 1,
      "iha_enlem": 41.51,
      "iha_boylam": 36.11,
      "iha_irtifa": 60,
      "iha_yonelme": 180
    }
  ]
}
```

---

## Konfigürasyon

`siha_autonomy/config/default.yaml` dosyasından tüm parametreler değiştirilebilir.

### Karar Motoru Parametreleri (YENİ)

```yaml
decision:
  weight_distance: 0.40    # Mesafe ağırlığı
  weight_angle: 0.30       # Açı ağırlığı
  weight_maneuver: 0.20    # Manevra maliyeti ağırlığı
  weight_eta: 0.10         # ETA ağırlığı
  evade_range_m: 50.0      # Kaçınma mesafesi (m)
  evade_speed_ms: 5.0      # Tehdit hız eşiği (m/s)
  relock_penalty: 50.0     # Tekrar kilit cezası
```

---

## Dizin Yapısı

```
test_siha/
├── README.md                    ← Bu dosya
├── mavlink_bridge.py            ← Python MAVLink köprüsü
├── gz_camera_bridge.py          ← Gazebo kamera köprüsü
├── gazebo_visualizer_v2.py      ← Rakip İHA görselleştirici
├── test_bridge.py               ← Bridge test aracı
├── vtail_runway_savasan.sdf     ← Gazebo world (Samsun)
├── run_all.sh                   ← Tüm servisleri başlatır
├── scripts/                     ← Python ROS2 node'ları
│   ├── mavlink_bridge.py
│   ├── mission_manager_node.py
│   ├── decision_node.py
│   ├── tracker_node.py
│   ├── guidance_node.py
│   ├── vision_gui_node.py
│   └── requirements.txt
└── siha_autonomy/               ← Ana C++ ROS2 paketi
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/default.yaml
    ├── launch/
    │   ├── autonomy_launch.py   ← Eski launch dosyası
    │   └── siha_launch.py       ← Yeni tek-komut launch
    ├── include/siha_autonomy/
    │   ├── core/                ← config.hpp, mission_controller.hpp
    │   ├── vision/              ← YOLO, tracker, lockon, pipeline
    │   ├── flight/              ← FlightController, GuidanceSystem, PID
    │   ├── mission/             ← Kamikaze, Evasion, Search, QR
    │   ├── decision/            ← DecisionEngine (YENİ)
    │   ├── safety/              ← SafetyMonitor
    │   ├── comm/                ← ServerComm, TelemetryManager
    │   └── recording/           ← VideoRecorder, OverlayRenderer
    └── src/
        ├── main_node.cpp
        ├── core/mission_controller.cpp
        ├── vision/
        ├── flight/
        ├── mission/
        ├── decision/            ← decision_engine.cpp (YENİ)
        ├── safety/
        ├── comm/
        └── recording/
```

---

## Lisans

MIT License — Bkz. [LICENSE](LICENSE)

---

*SAEROTECH — TEKNOFEST 2026*

