#!/bin/bash
# launch_all.sh — SIHA Otonom Sistem Tam Başlatıcı
# TEKNOFEST 2026 Savaşan İHA Yarışması
#
# Kullanım:
#   chmod +x scripts/launch_all.sh
#   ./scripts/launch_all.sh [--headless] [--no-gui] [--sitl-url udp:127.0.0.1:14550]
#
# Tüm node'ları ayrı terminal pencerelerinde başlatır.
# gnome-terminal yoksa tmux kullanılır.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="${WS_DIR:-$HOME/siha_ws}"

# ── Argümanlar ──────────────────────────────────────────────────────────────
HEADLESS=false
NO_GUI=false
SITL_URL="udp:127.0.0.1:14550"
TAKEOFF_ALT=30.0
MODEL_PATH="yolov8n.pt"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)  HEADLESS=true ; shift ;;
    --no-gui)    NO_GUI=true   ; shift ;;
    --sitl-url)  SITL_URL="$2" ; shift 2 ;;
    --takeoff-alt) TAKEOFF_ALT="$2" ; shift 2 ;;
    --model)     MODEL_PATH="$2" ; shift 2 ;;
    *) echo "Bilinmeyen argüman: $1" >&2 ; exit 1 ;;
  esac
done

# ── ROS2 ortamı ──────────────────────────────────────────────────────────────
# ROS dağıtımını otomatik algıla
if [[ -n "${ROS_DISTRO:-}" ]]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
else
  ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
fi
WS_SETUP="$WS_DIR/install/setup.bash"
RS="source $ROS_SETUP"
if [[ -f "$WS_SETUP" ]]; then
  RS="$RS && source $WS_SETUP"
fi

# ── Terminal başlatma yardımcısı ─────────────────────────────────────────────
launch_in_terminal() {
  local title="$1"
  local cmd="$2"

  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$RS && $cmd; exec bash" &
  elif command -v tmux &>/dev/null; then
    tmux new-window -n "$title" "$RS && $cmd"
  else
    echo "[WARN] gnome-terminal ve tmux bulunamadı — arka planda çalıştırılıyor: $title"
    bash -c "$RS && $cmd" &
  fi
}

echo "╔══════════════════════════════════════════════════════╗"
echo "║   SAEROTECH SİHA — Tam Sistem Başlatıcı             ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  SITL URL  : $SITL_URL"
echo "║  Takeoff   : ${TAKEOFF_ALT}m"
echo "║  Model     : $MODEL_PATH"
echo "║  Headless  : $HEADLESS"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# 1) MAVLink Bridge
echo "[1/6] MAVLink Bridge başlatılıyor..."
launch_in_terminal "1-MAVLink" \
  "python3 $SCRIPT_DIR/mavlink_bridge.py --url $SITL_URL"
sleep 2

# 2) Görev Yöneticisi
echo "[2/6] Görev Yöneticisi başlatılıyor..."
launch_in_terminal "2-MissionManager" \
  "python3 $SCRIPT_DIR/mission_manager_node.py --takeoff-alt $TAKEOFF_ALT"
sleep 1

# 3) Karar Modülü
echo "[3/6] Karar Modülü başlatılıyor..."
launch_in_terminal "3-Decision" \
  "python3 $SCRIPT_DIR/decision_node.py"
sleep 1

# 4) Tracker
echo "[4/6] Tracker başlatılıyor..."
launch_in_terminal "4-Tracker" \
  "python3 $SCRIPT_DIR/tracker_node.py --model $MODEL_PATH"
sleep 1

# 5) Guidance
echo "[5/6] Guidance başlatılıyor..."
launch_in_terminal "5-Guidance" \
  "python3 $SCRIPT_DIR/guidance_node.py"
sleep 1

# 6) Vision GUI (opsiyonel)
if [[ "$NO_GUI" == false ]]; then
  echo "[6/6] Vision GUI başlatılıyor..."
  GUI_ARGS="--model $MODEL_PATH"
  if [[ "$HEADLESS" == true ]]; then
    GUI_ARGS="$GUI_ARGS --headless"
  fi
  launch_in_terminal "6-VisionGUI" \
    "python3 $SCRIPT_DIR/vision_gui_node.py $GUI_ARGS"
else
  echo "[6/6] Vision GUI atlandı (--no-gui)"
fi

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Tüm node'lar başlatıldı!                           ║"
echo "║                                                      ║"
echo "║  Topic'leri kontrol et:                             ║"
echo "║    ros2 topic list                                   ║"
echo "║    ros2 topic echo /mission/state                   ║"
echo "║    ros2 topic echo /decision/target                 ║"
echo "╚══════════════════════════════════════════════════════╝"
