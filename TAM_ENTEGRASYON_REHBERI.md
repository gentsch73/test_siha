# TAM ENTEGRASYON REHBERİ
# SAEROTECH — Savaşan İHA Simülasyon Ortamı

## Sistem Genel Görünüm

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO (runway world)                     │
│                                                             │
│   ✈ mini_talon_vtail  (senin dronun, SITL kontrollü)       │
│       └── nose_camera → /camera/image_raw (köprü ile)       │
│                                                             │
│   ✈ npc_1  ✈ npc_2  ✈ npc_3 ... ✈ npc_15                 │
│   (NPC Publisher → Visualizer tarafından spawn ediliyor)    │
│                                                             │
│   🟥 Arena sınır duvarları (boundary.json'dan)              │
└────────────────┬────────────────────────────────────────────┘
                 │ gz.transport
┌────────────────┴────────────────────────────────────────────┐
│                    ROS2 Topic'ler                            │
│                                                             │
│  /sunucu_telemetri    ← NPC Publisher (15 rakip telemetri)  │
│  /camera/image_raw    ← gz_camera_bridge (Gazebo kamera)    │
│  /telemetry/own       ← harita_node / MAVROS                │
│                                                             │
│  siha_autonomy        → YOLO + Tracker + Lockon + Güdüm     │
└─────────────────────────────────────────────────────────────┘
```

---

## KURULUM (Bir Kere)

### 1. Eksik paketleri kur

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  libopencv-dev \
  python3-opencv \
  ffmpeg
```

### 2. Dosyaları yerine koy

```bash
# Yeni world dosyası (Samsun koordinatlı)
cp vtail_runway_savasan.sdf ~/gz_ws/src/SITL_Models/Gazebo/worlds/

# Güncellenmiş visualizer
cp gazebo_visualizer_v2.py ~/Saerotech_SIHA/
# veya mevcut dosyanın yerine:
# cp gazebo_visualizer_v2.py ~/ros2_ws/src/siha_telemetri/siha_telemetri/gazebo_visualizer.py

# Kamera köprüsü
cp gz_camera_bridge.py ~/Saerotech_SIHA/
```

### 3. ROS2 workspace'i derle

```bash
cd ~/siha_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## ÇALIŞTIRMA (6 Terminal)

**Her terminalde başlangıç:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/siha_ws/install/setup.bash
```

### Terminal 1 — ArduPilot SITL

```bash
cd ~/ardupilot

sim_vehicle.py -v ArduPlane --model JSON \
  --add-param-file=$HOME/gz_ws/src/SITL_Models/Gazebo/config/mini_talon_vtail.param \
  -l 41.51,36.11,50,0 \
  --console --map
```

> `-l 41.51,36.11,50,0` → Samsun koordinatları (NPC Publisher ile aynı)

### Terminal 2 — Gazebo

```bash
gz sim -v4 -r ~/gz_ws/src/SITL_Models/Gazebo/worlds/vtail_runway_savasan.sdf
```

> mini_talon_vtail + runway yüklenir, SITL bağlantısını bekler

### Terminal 3 — Kamera Köprüsü

```bash
# Yöntem A: Doğrudan ros_gz_image kullan
ros2 run ros_gz_image image_bridge \
  /world/runway/model/mini_talon_vtail/link/base_link/sensor/nose_camera/image \
  /camera/image_raw

# Yöntem B: Script ile
cd ~/Saerotech_SIHA
python3 gz_camera_bridge.py
```

> Gazebo kamerasını `/camera/image_raw` ROS2 topic'ine köprüler

### Terminal 4 — NPC Publisher + Visualizer + Web UI + Harita

```bash
cd ~/Saerotech_SIHA

# 4a) NPC Publisher
ros2 run siha_telemetri npc_publisher2 &

# 4b) Visualizer (NPC dronları Gazebo'da göster)
python3 gazebo_visualizer_v2.py &

# 4c) Harita Node
ros2 run siha_telemetri harita_node &

# 4d) Web UI
python3 serve_ui.py &
```

> Hepsini tek terminalde arka planda çalıştırmak kolaylık sağlar.
> Veya her birini ayrı terminalde çalıştır.

**ÖNEMLİ:** Web UI'da (http://localhost:8080) sınırı çiz → NPC'ler spawn olur.

### Terminal 5 — Otonom Sistem

```bash
cd ~/siha_ws
source install/setup.bash

ros2 run siha_autonomy main_node --ros-args \
  -p simulation:=true \
  -p mission_type:=0 \
  -p team_id:=1 \
  -p vision.model_path:=$HOME/siha_ws/models/yolov8n.onnx \
  -p vision.camera_topic:=/camera/image_raw \
  -p flight.mavlink_url:=udp:127.0.0.1:14550
```

### Terminal 6 — Monitörleme

```bash
# Kamera görüntüsü geliyor mu?
ros2 topic hz /camera/image_raw

# NPC telemetrisi geliyor mu?
ros2 topic echo /sunucu_telemetri --once | head -20

# Aktif topic'ler
ros2 topic list

# Gazebo kamera topic'ini bul (eğer köprü çalışmıyorsa)
gz topic -l | grep image
```

---

## DOĞRULAMA ADIMLARI

### ✅ Gazebo açıldı mı?
- mini_talon_vtail pistte görünmeli
- SITL bağlanınca propeller dönmeye başlar

### ✅ SITL bağlandı mı?
- SITL konsolda `APM: ArduPlane` mesajı
- Gazebo'da uçak tepki veriyor

### ✅ Sınır çizildi mi?
- http://localhost:8080 aç, haritada polygon çiz
- NPC Publisher logunda: `15 adet IHA basariyla spawn edildi!`

### ✅ NPC'ler Gazebo'da mı?
- Visualizer logunda: `NPC #1 spawn edildi`
- Gazebo Entity Tree'de npc_1, npc_2, ... görünmeli
- Renkli mini uçak modelleri havada uçmalı

### ✅ Kamera çalışıyor mu?
- `ros2 topic hz /camera/image_raw` → ~30 Hz
- `ros2 run rqt_image_view rqt_image_view` ile görüntü izle

### ✅ Otonom sistem çalışıyor mu?
- Logda: `Faz geçişi: 0 → 1 → 2 → 3`
- `[YoloDetector] Model yüklendi`
- `[VideoPipeline] Başlatıldı: /camera/image_raw`

---

## SORUN GİDERME

| Sorun | Çözüm |
|-------|-------|
| Gazebo açılmıyor | `GZ_SIM_RESOURCE_PATH` kontrol et, model:// yolları |
| SITL bağlanmıyor | Gazebo'yu `-r` ile çalıştırdığından emin ol |
| NPC'ler görünmüyor | Sınırı web UI'dan çiz, visualizer logunu kontrol et |
| Kamera topic yok | `gz topic -l \| grep image` ile Gazebo topic'ini bul |
| Köprü çalışmıyor | `ros-jazzy-ros-gz-image` kurulu mu? |
| World adı yanlış | `gz topic -l` çıktısında `/world/runway/` olmalı |
| Koordinat uyumsuz | World, SITL ve NPC Publisher aynı lat/lon kullanmalı |
| venv numpy hatası | `deactivate` yap, ROS2 terminalinde venv olmamalı |

---

## NOTLAR

- **Koordinat sistemi:** Tüm bileşenler Samsun (41.51, 36.11) kullanır
- **World adı:** `runway` (vtail_runway_savasan.sdf)
- **Kamera topic:** Gazebo → `/world/runway/model/.../nose_camera/image` → Köprü → `/camera/image_raw`
- **NPC modeller:** Basitleştirilmiş sabit kanat (renkli, takım numarasına göre)
- **SITL komutu:** `-l 41.51,36.11,50,0` ile Samsun konumunda başlatılır
