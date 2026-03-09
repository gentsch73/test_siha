# SAEROTECH SİHA Otonom Sistem — Çalıştırma Rehberi

## Genel Bakış

Sistem 5 ayrı bileşenden oluşuyor ve her birini ayrı terminalde çalıştırıyorsun:

```
┌──────────────────┐     ┌──────────────┐     ┌──────────────────┐
│  NPC Publisher   │────▶│ Test Bridge   │────▶│  OTONOM SİSTEM   │
│  (Rakip İHA'lar) │     │ (Sahte Kamera │     │  (siha_autonomy) │
│  [Python/ROS2]   │     │  + Telemetri) │     │  [C++/ROS2]      │
└──────────────────┘     └──────────────┘     └──────────────────┘
                              │
                         ┌────┴────┐
                         │ Gazebo  │ (opsiyonel)
                         │ + WebUI │
                         └─────────┘
```

---

## Ön Gereksinimler

- Ubuntu 24.04
- ROS2 Jazzy kurulu (`source /opt/ros/jazzy/setup.bash`)
- Mevcut Saerotech_SIHA projesi çalışıyor

---

## Adım 1: Bağımlılıkları Kur

```bash
sudo apt-get update && sudo apt-get install -y \
  libopencv-dev \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ffmpeg \
  python3-opencv
```

## Adım 2: Workspace Oluştur

```bash
# Yeni bir workspace oluştur (mevcut projeyle yan yana)
mkdir -p ~/siha_ws/src
cd ~/siha_ws/src

# Mevcut Python paketini link'le
ln -s ~/Saerotech_SIHA siha_telemetri

# Otonom paketi aç (indirdiğin tar.gz)
cd ~
tar xzf siha_autonomy.tar.gz
cp -r siha_autonomy ~/siha_ws/src/
```

Workspace yapısı şöyle olmalı:
```
~/siha_ws/
└── src/
    ├── siha_telemetri/     → (symlink) Mevcut Python projesi
    │   ├── npc_publisher2.py
    │   ├── gazebo_visualizer.py
    │   ├── harita_node.py
    │   └── ...
    └── siha_autonomy/      → Yeni C++ otonom sistem
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        ├── src/
        ├── config/
        └── launch/
```

## Adım 3: Derle

```bash
cd ~/siha_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Hata alırsan:
```bash
# Sadece autonomy paketini derle (hızlı debug)
colcon build --packages-select siha_autonomy
```

## Adım 4: YOLO Model Hazırla

```bash
# ultralytics kur
pip3 install ultralytics --break-system-packages

# Modeli ONNX formatına çevir
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=640)
"

# Model dosyasını taşı
mkdir -p ~/siha_ws/models
mv yolov8n.onnx ~/siha_ws/models/
```

## Adım 5: test_bridge.py Dosyasını Kopyala

```bash
cp test_bridge.py ~/siha_ws/src/siha_telemetri/
```

---

## ÇALIŞTIRMA

Her terminal için önce:
```bash
cd ~/siha_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Terminal 1 — NPC Publisher (Rakip İHA'lar)

```bash
cd ~/Saerotech_SIHA
ros2 run siha_telemetri npc_publisher2
```

Bu, 15 adet sahte rakip İHA yayınlar (`/sunucu_telemetri` topic'ine).

### Terminal 2 — Test Köprüsü

```bash
cd ~/Saerotech_SIHA
python3 test_bridge.py
```

Bu script:
- `/sunucu_telemetri`'den NPC verisi dinler
- `/camera/image_raw`'a sahte kamera görüntüsü yayınlar (NPC'leri piksel olarak çizer)
- `/telemetry/own`'a sahte kendi telemetrimizi yayınlar
- OpenCV penceresi açar (kapatmak için `--headless` ekle)

### Terminal 3 — Gazebo (Opsiyonel)

```bash
gz sim -r ~/Saerotech_SIHA/siha_world.sdf
```

Ayrı terminalde visualizer:
```bash
ros2 run siha_telemetri gazebo_visualizer
```

### Terminal 4 — Otonom Sistem

```bash
ros2 run siha_autonomy main_node --ros-args \
  -p simulation:=true \
  -p mission_type:=0 \
  -p team_id:=1 \
  -p vision.model_path:=$HOME/siha_ws/models/yolov8n.onnx
```

### Terminal 5 — Monitörleme

```bash
# Aktif topic'leri listele
ros2 topic list

# Telemetri verisini izle
ros2 topic echo /sunucu_telemetri --once

# Kamera FPS kontrol
ros2 topic hz /camera/image_raw

# Otonom sistem loglarını izle (renkli)
ros2 run siha_autonomy main_node 2>&1 | grep -E "Faz|LOCKON|Hedef|SINIR|BATARYA"
```

---

## Docker ile Çalıştırma (Alternatif)

Lokal kurulum yerine Docker da kullanabilirsin:

```bash
cd ~/Saerotech_SIHA

# Güncellenmiş Dockerfile'ı kopyala
cp ~/test_setup/Dockerfile .
cp ~/test_setup/docker-compose.yml .

# siha_autonomy'yi src/ içine kopyala
cp -r ~/siha_autonomy src/

# Build & Run
docker compose up --build
```

---

## ArduPilot SITL ile Tam Test (İleri Seviye)

Gerçek otopilot simülasyonu için ArduPilot SITL gerekli:

```bash
# 1. ArduPilot kur (bir kere)
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# 2. SITL başlat (mini_talon_vtail modeli)
cd ~/ardupilot
sim_vehicle.py -v ArduPlane --model gazebo-zephyr \
  -f gazebo-zephyr \
  --console --map \
  -l 41.51,36.11,0,0

# SITL varsayılan olarak UDP 14550 portu üzerinden MAVLink dinler
# main_node bu porta bağlanır: flight.mavlink_url:=udp:127.0.0.1:14550

# 3. MAVROS2 çalıştır (ayrı terminalde)
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

---

## Test Senaryoları

### Senaryo 1: Durum Makinesi Testi
Otonom sistemi başlat, loglardan faz geçişlerini izle:
```
IDLE → PRE_ARM → ARMED → TAKEOFF → CLIMB → SEARCH
```

### Senaryo 2: Kamera ve YOLO Testi
Test bridge çalışırken, OpenCV penceresinde NPC İHA'larını gör.
Autonomy loglarında YOLO inference süresini kontrol et.

### Senaryo 3: Kilitlenme Testi
NPC İHA'ları kameranın merkezinde görünürse, lockon sayacı artmalı.
4 saniye sonra `KİLİTLENME BAŞARILI!` logu gelmeli.

### Senaryo 4: Güvenlik Testi
Boundary.json'da küçük bir sınır çiz.
Kendi telemetriyi sınır dışına ayarla.
`SINIR AŞIMI!` alarmı ve `BOUNDARY_RETURN` fazı gelmeli.

---

## Sorun Giderme

| Sorun | Çözüm |
|-------|-------|
| `package not found` | `source ~/siha_ws/install/setup.bash` unuttun |
| `cv::VideoCapture yok` | `sudo apt install libopencv-dev` |
| `YOLO model yüklenemedi` | Model path'i kontrol et: `-p vision.model_path:=...` |
| `No camera data` | test_bridge.py çalışıyor mu? `ros2 topic hz /camera/image_raw` |
| `Heartbeat lost` | SITL çalışmıyorsa normal, sistem EMERGENCY'e geçer |
| Derleme hatası | `colcon build 2>&1 | tail -30` ile hatayı gör |
