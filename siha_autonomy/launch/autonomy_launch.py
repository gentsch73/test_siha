"""
SAEROTECH SİHA Otonom Sistem — ROS2 Launch Dosyası

Kullanım:
  Simülasyon:  ros2 launch siha_autonomy autonomy_launch.py mode:=sim
  Gerçek:      ros2 launch siha_autonomy autonomy_launch.py mode:=real
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch argümanları
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description='Çalışma modu: sim (simülasyon) veya real (gerçek)')

    team_arg = DeclareLaunchArgument(
        'team_id', default_value='1',
        description='Takım numarası')

    mission_arg = DeclareLaunchArgument(
        'mission', default_value='0',
        description='Görev tipi: 0=Savaşan, 1=Kamikaze')

    model_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n.onnx',
        description='YOLO model dosya yolu')

    # Ana otonom düğüm
    autonomy_node = Node(
        package='siha_autonomy',
        executable='main_node',
        name='mission_controller',
        output='screen',
        parameters=[{
            'simulation': LaunchConfiguration('mode') == 'sim',
            'mission_type': LaunchConfiguration('mission'),
            'team_id': LaunchConfiguration('team_id'),
            'vision.model_path': LaunchConfiguration('model_path'),
            'vision.camera_topic': '/camera/image_raw',
            'vision.camera_device': '/dev/video0',
            'vision.frame_width': 640,
            'vision.frame_height': 480,
            'vision.target_fps': 30,
            'vision.confidence_threshold': 0.45,
            'vision.use_tensorrt': False,
            'flight.mavlink_url': 'udp:127.0.0.1:14550',
            'flight.takeoff_altitude': 30.0,
            'flight.cruise_altitude': 60.0,
            'flight.cruise_speed': 25.0,
            'comm.server_ip': '192.168.1.100',
            'comm.server_port': 8080,
        }]
    )

    return LaunchDescription([
        mode_arg,
        team_arg,
        mission_arg,
        model_arg,
        autonomy_node,
    ])
