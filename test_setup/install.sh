#!/bin/bash
# ============================================================
#  SAEROTECH SİHA OTONOM SİSTEM — KURULUM & TEST
#  Adım adım çalıştırma scripti
# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════╗"
echo "║  SAEROTECH SİHA OTONOM SİSTEM KURULUMU           ║"
echo "║  TEKNOFEST 2026 — Savaşan İHA Yarışması           ║"
echo "╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

# ─────────────────────────────────────────────────
#  AŞAMA 1: Sistem Bağımlılıklarını Kur
# ─────────────────────────────────────────────────
echo -e "${YELLOW}[1/6] Sistem bağımlılıkları kuruluyor...${NC}"

sudo apt-get update
sudo apt-get install -y \
    libopencv-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ffmpeg \
    python3-opencv \
    python3-pip

echo -e "${GREEN}[✓] Bağımlılıklar kuruldu.${NC}"

# ─────────────────────────────────────────────────
#  AŞAMA 2: Workspace Yapısını Hazırla
# ─────────────────────────────────────────────────
echo -e "${YELLOW}[2/6] Workspace hazırlanıyor...${NC}"

WORKSPACE=~/siha_ws
mkdir -p ${WORKSPACE}/src

# Mevcut projeyi kopyala (eğer varsa)
if [ -d ~/Saerotech_SIHA ]; then
    echo "  Mevcut Saerotech_SIHA projesi bulundu."
    ln -sf ~/Saerotech_SIHA ${WORKSPACE}/src/siha_telemetri 2>/dev/null || true
fi

# Autonomy paketini kopyala
if [ -d ~/siha_autonomy ]; then
    echo "  siha_autonomy paketi kopyalanıyor..."
    cp -r ~/siha_autonomy ${WORKSPACE}/src/
else
    echo -e "${RED}  HATA: ~/siha_autonomy bulunamadı!${NC}"
    echo "  Önce tar.gz dosyasını açın:"
    echo "    tar xzf siha_autonomy.tar.gz -C ~/"
    exit 1
fi

echo -e "${GREEN}[✓] Workspace hazır: ${WORKSPACE}${NC}"

# ─────────────────────────────────────────────────
#  AŞAMA 3: Derleme
# ─────────────────────────────────────────────────
echo -e "${YELLOW}[3/6] Derleniyor...${NC}"

cd ${WORKSPACE}
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select siha_autonomy 2>&1

if [ $? -eq 0 ]; then
    echo -e "${GREEN}[✓] Derleme başarılı!${NC}"
else
    echo -e "${RED}[✗] Derleme hatası. Logları kontrol edin.${NC}"
    exit 1
fi

source ${WORKSPACE}/install/setup.bash

# ─────────────────────────────────────────────────
#  AŞAMA 4: YOLO Model İndir
# ─────────────────────────────────────────────────
echo -e "${YELLOW}[4/6] YOLO modeli kontrol ediliyor...${NC}"

MODEL_DIR=${WORKSPACE}/models
mkdir -p ${MODEL_DIR}

if [ ! -f ${MODEL_DIR}/yolov8n.onnx ]; then
    echo "  YOLOv8n ONNX modeli indiriliyor..."
    pip3 install ultralytics --break-system-packages 2>/dev/null || pip3 install ultralytics
    python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=640)
import shutil
shutil.move('yolov8n.onnx', '${MODEL_DIR}/yolov8n.onnx')
print('Model exported: ${MODEL_DIR}/yolov8n.onnx')
"
    echo -e "${GREEN}[✓] YOLO modeli hazır.${NC}"
else
    echo -e "${GREEN}[✓] YOLO modeli zaten mevcut.${NC}"
fi

# ─────────────────────────────────────────────────
#  AŞAMA 5: Çalıştırma Bilgisi
# ─────────────────────────────────────────────────
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════${NC}"
echo -e "${CYAN}  KURULUM TAMAMLANDI! Çalıştırma Adımları:${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════${NC}"
echo ""
echo -e "${GREEN}  Her terminalde önce:${NC}"
echo "    source ${WORKSPACE}/install/setup.bash"
echo ""
echo -e "${YELLOW}  Terminal 1 — NPC Publisher (Rakip İHA'lar):${NC}"
echo "    cd ~/Saerotech_SIHA && ros2 run siha_telemetri npc_publisher2"
echo ""
echo -e "${YELLOW}  Terminal 2 — Test Köprüsü (Sahte Kamera + Telemetri):${NC}"
echo "    python3 ~/test_bridge.py"
echo ""
echo -e "${YELLOW}  Terminal 3 — Gazebo (Opsiyonel):${NC}"
echo "    gz sim -r siha_world.sdf"
echo ""
echo -e "${YELLOW}  Terminal 4 — Otonom Sistem:${NC}"
echo "    ros2 run siha_autonomy main_node --ros-args \\"
echo "      -p simulation:=true \\"
echo "      -p vision.model_path:=${MODEL_DIR}/yolov8n.onnx"
echo ""
echo -e "${YELLOW}  Terminal 5 — Web UI (Opsiyonel):${NC}"
echo "    cd ~/Saerotech_SIHA && python3 serve_ui.py"
echo ""
echo -e "${CYAN}  ROS2 Topic'leri İzleme:${NC}"
echo "    ros2 topic list"
echo "    ros2 topic echo /sunucu_telemetri"
echo "    ros2 topic echo /camera/image_raw --no-arr"
echo ""
