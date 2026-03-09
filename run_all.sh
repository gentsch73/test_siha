#!/bin/bash
# ════════════════════════════════════════════════════════
# SAEROTECH SİHA — Sistem Başlatıcı
# TEKNOFEST 2026 Savaşan İHA Yarışması
#
# Kullanım:
#   ./run_all.sh                 # varsayılan (sim=true)
#   ./run_all.sh --real          # gerçek donanım modu
#   ./run_all.sh --headless      # başsız (no GUI) mod
# ════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SIHA_WS:-$HOME/siha_ws}"
ARDU_DIR="${ARDU_DIR:-$HOME/ardupilot}"

SIM_MODE=true
HEADLESS=false

# Argüman ayrıştırma
while [[ $# -gt 0 ]]; do
  case "$1" in
    --real)     SIM_MODE=false; shift ;;
    --headless) HEADLESS=true;  shift ;;
    *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

RS="source /opt/ros/jazzy/setup.bash; source $WS_DIR/install/setup.bash 2>/dev/null"

TERM_CMD="gnome-terminal"
if $HEADLESS || ! command -v gnome-terminal &>/dev/null; then
  TERM_CMD="xterm -T"
fi

open_term() {
  local title="$1"
  local cmd="$2"
  if $HEADLESS; then
    bash -c "$cmd" &
  else
    gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
  fi
}

echo "════════════════════════════════════"
echo " SAEROTECH SİHA — Başlatılıyor"
echo " Mod: $([ $SIM_MODE = true ] && echo SİMÜLASYON || echo GERÇEK DONANIM)"
echo "════════════════════════════════════"

if $SIM_MODE; then
  # 1) ArduPlane SITL
  open_term "1-SITL" "
    cd $ARDU_DIR
    sim_vehicle.py -v ArduPlane -l 41.51,36.11,50,0 --console --map"
  sleep 4

  # 2) Gazebo
  open_term "2-Gazebo" "
    $RS
    gz sim -v4 -r $SCRIPT_DIR/vtail_runway_savasan.sdf"
  sleep 5

  # 3) Kamera Köprüsü
  open_term "3-Kamera" "
    $RS
    python3 $SCRIPT_DIR/gz_camera_bridge.py"
  sleep 2

  # 4) Rakip İHA Görselleştirici
  open_term "4-Visualizer" "
    $RS
    python3 $SCRIPT_DIR/gazebo_visualizer_v2.py"
  sleep 2
fi

# 5) MAVLink Bridge
open_term "5-MAVLink" "
  $RS
  python3 $SCRIPT_DIR/mavlink_bridge.py"
sleep 3

# 6) Otonom Sistem (C++ — ros2 launch ile)
open_term "6-Otonom" "
  $RS
  ros2 launch siha_autonomy siha_launch.py \
    sim:=$SIM_MODE \
    team_id:=1"

echo ""
echo "════════════════════════════════════"
echo " Sistem başlatıldı!"
echo " Debug görüntüsü için:"
echo "   ros2 run rqt_image_view rqt_image_view"
echo "   (Sol üstten /vision/debug_image seç)"
echo ""
echo " Karar motoru çıktısı:"
echo "   ros2 topic echo /decision/target"
echo ""
echo " Çıkmak için Ctrl+C"
echo "════════════════════════════════════"

