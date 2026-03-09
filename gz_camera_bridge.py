#!/usr/bin/env python3
"""
gz_camera_bridge.py — Gazebo Kamera → ROS2 Köprüsü

mini_talon_vtail modelindeki nose_camera sensöründen gelen
Gazebo görüntüsünü ROS2 /camera/image_raw topic'ine köprüler.

ros2 run ros_gz_image image_bridge komutu da kullanılabilir ama
bu script daha esnek ve kamera topic'ini otomatik bulur.

Kullanım:
  python3 gz_camera_bridge.py
  
  # veya ros2 run ile (pakete eklendiğinde):
  ros2 run siha_telemetri gz_camera_bridge
"""

import subprocess
import sys


def main():
    # Gazebo kamera topic'i (world/model/link/sensor bazlı)
    gz_topic = "/world/runway/model/mini_talon_vtail/link/base_link/sensor/nose_camera/image"
    ros_topic = "/camera/image_raw"

    print(f"[CameraBridge] Gazebo → ROS2 köprüsü başlatılıyor")
    print(f"  Gazebo topic: {gz_topic}")
    print(f"  ROS2 topic:   {ros_topic}")
    print(f"  Ctrl+C ile durdurun.")

    # ros_gz_bridge kullan
    cmd = [
        "ros2", "run", "ros_gz_image", "image_bridge",
        gz_topic, ros_topic
    ]

    try:
        proc = subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n[CameraBridge] Durduruldu.")
    except FileNotFoundError:
        print("[CameraBridge] ros_gz_image paketi bulunamadı!")
        print("  Kur: sudo apt install ros-jazzy-ros-gz-image")
        sys.exit(1)


if __name__ == '__main__':
    main()
