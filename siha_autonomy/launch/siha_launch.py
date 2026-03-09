"""
siha_launch.py — SAEROTECH SİHA Otonom Sistem Launch Dosyası
TEKNOFEST 2026 Savaşan İHA Yarışması

Tüm node'ları tek komutla başlatır:
  - main_node (C++ — MissionController + tüm modüller)
  - mavlink_bridge (Python — MAVLink ↔ ROS2)
  - (opsiyonel) gazebo_visualizer

Kullanım:
  ros2 launch siha_autonomy siha_launch.py
  ros2 launch siha_autonomy siha_launch.py sim:=true takeoff_alt:=40.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("siha_autonomy")

    # ─── Launch Argümanları ────────────────────────────────────────────────
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Simülasyon modu: true = Gazebo, false = Gerçek donanım",
    )

    declare_mission_type = DeclareLaunchArgument(
        "mission_type",
        default_value="0",
        description="Görev tipi: 0 = Savaşan İHA, 1 = Kamikaze",
    )

    declare_team_id = DeclareLaunchArgument(
        "team_id",
        default_value="1",
        description="Takım numarası (1–15)",
    )

    declare_takeoff_alt = DeclareLaunchArgument(
        "takeoff_alt",
        default_value="30.0",
        description="Kalkış irtifası (metre)",
    )

    declare_mavlink_url = DeclareLaunchArgument(
        "mavlink_url",
        default_value="udp:127.0.0.1:14550",
        description="MAVLink bağlantı URL'si",
    )

    declare_server_ip = DeclareLaunchArgument(
        "server_ip",
        default_value="192.168.1.100",
        description="Yarışma sunucusu IP adresi",
    )

    declare_with_bridge = DeclareLaunchArgument(
        "with_bridge",
        default_value="true",
        description="MAVLink bridge'i başlat (true/false)",
    )

    # ─── C++ Ana Node ──────────────────────────────────────────────────────
    main_node = Node(
        package="siha_autonomy",
        executable="main_node",
        name="mission_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            # Konfigürasyon dosyası
            os.path.join(pkg_share, "config", "default.yaml"),
            # Komut satırı argümanlarıyla üst üste yaz
            {
                "simulation": LaunchConfiguration("sim"),
                "mission_type": LaunchConfiguration("mission_type"),
                "team_id": LaunchConfiguration("team_id"),
                "flight.takeoff_altitude": LaunchConfiguration("takeoff_alt"),
                "flight.mavlink_url": LaunchConfiguration("mavlink_url"),
                "comm.server_ip": LaunchConfiguration("server_ip"),
            },
        ],
        # Logları renklendir (opsiyonel, konsola ANSI renk kodu)
        additional_env={"RCUTILS_COLORIZED_OUTPUT": "1"},
    )

    # ─── MAVLink Bridge (Python) ───────────────────────────────────────────
    # Root dizinindeki mavlink_bridge.py'yi başlat
    ws_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    )
    bridge_script = os.path.join(ws_root, "mavlink_bridge.py")
    # Alternatif: scripts/ içinde varsa oradan al
    if not os.path.exists(bridge_script):
        bridge_script = os.path.join(
            os.path.dirname(pkg_share), "mavlink_bridge.py"
        )

    mavlink_bridge = ExecuteProcess(
        cmd=[
            "python3",
            bridge_script,
            "--url", LaunchConfiguration("mavlink_url"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_bridge")),
    )

    # ─── Launch Description ───────────────────────────────────────────────
    return LaunchDescription(
        [
            declare_sim,
            declare_mission_type,
            declare_team_id,
            declare_takeoff_alt,
            declare_mavlink_url,
            declare_server_ip,
            declare_with_bridge,
            LogInfo(msg="=== SAEROTECH SİHA OTONOM SİSTEM BAŞLATILIYOR ==="),
            main_node,
            mavlink_bridge,
        ]
    )
