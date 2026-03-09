#!/bin/bash
SIHA_DIR="$HOME/Saerotech_SIHA"
WS_DIR="$HOME/siha_ws"
GZ_CONFIG="$HOME/gz_ws/src/SITL_Models/Gazebo/config"
GZ_WORLDS="$HOME/gz_ws/src/SITL_Models/Gazebo/worlds"
ARDU_DIR="$HOME/ardu_ws/src/ardupilot"

RS="source /opt/ros/jazzy/setup.bash; source $WS_DIR/install/setup.bash 2>/dev/null"

echo "═══ SAEROTECH SİSTEM BAŞLATICI ═══"

# 1) SITL
gnome-terminal --title="1-SITL" -- bash -c "
  cd $ARDU_DIR
  sim_vehicle.py -v ArduPlane --model JSON \
    --add-param-file=$GZ_CONFIG/mini_talon_vtail.param \
    -l 41.51,36.11,50,0 --console --map
  exec bash" &
sleep 3

# 2) Gazebo
gnome-terminal --title="2-Gazebo" -- bash -c "
  $RS
  gz sim -v4 -r $GZ_WORLDS/vtail_runway_savasan.sdf
  exec bash" &
sleep 5

# 3) NPC Publisher
gnome-terminal --title="3-NPC" -- bash -c "
  $RS
  cd $SIHA_DIR
  ros2 run siha_telemetri npc_publisher2
  exec bash" &
sleep 2

# 4) Harita + WebUI
gnome-terminal --title="4-WebUI" -- bash -c "
  $RS
  cd $SIHA_DIR
  ros2 run siha_telemetri harita_node &
  sleep 1
  python3 serve_ui.py
  exec bash" &
sleep 2

# 5) Visualizer
gnome-terminal --title="5-Visualizer" -- bash -c "
  $RS
  cd $SIHA_DIR
  python3 gazebo_visualizer_v2.py
  exec bash" &
sleep 2

# 6) Kamera Köprüsü
gnome-terminal --title="6-Kamera" -- bash -c "
  $RS
  ros2 run ros_gz_image image_bridge /camera /camera/image_raw
  exec bash" &
sleep 2

# 7) MAVLink Bridge
gnome-terminal --title="7-MAVLink" -- bash -c "
  $RS
  cd $SIHA_DIR
  python3 mavlink_bridge.py
  exec bash" &
sleep 3

# 8) Otonom Sistem
gnome-terminal --title="8-Otonom" -- bash -c "
  $RS
  ros2 run siha_autonomy main_node --ros-args \
    -p simulation:=true \
    -p mission_type:=0 \
    -p team_id:=1 \
    -p vision.model_path:=$WS_DIR/models/yolov8n.onnx \
    -p vision.camera_topic:=/camera/image_raw
  exec bash" &

echo ""
echo "═══ 8 pencere açıldı! ═══"
echo "Web UI: http://localhost:8080"
echo "Sınırı çizmeyi unutma!"
