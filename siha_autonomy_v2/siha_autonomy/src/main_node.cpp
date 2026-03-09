/**
 * @file main_node.cpp
 * @brief SAEROTECH SİHA Otonom Sistem — ROS2 Ana Giriş Noktası
 *
 * Bu dosya tüm sistemi başlatır:
 *   1) Konfigürasyon yükle (YAML veya parametreler)
 *   2) MissionController oluştur
 *   3) ROS2 executor ile çalıştır
 *
 * Kullanım:
 *   ros2 run siha_autonomy main_node --ros-args -p simulation:=true
 *   ros2 run siha_autonomy main_node --ros-args -p simulation:=false
 */

#include "siha_autonomy/core/mission_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

siha::SystemConfig load_config(rclcpp::Node::SharedPtr param_node) {
    siha::SystemConfig cfg;

    // ROS2 parametrelerinden oku
    param_node->declare_parameter("simulation", true);
    param_node->declare_parameter("mission_type", 0);  // 0=savasan, 1=kamikaze
    param_node->declare_parameter("team_id", 1);

    // Vision
    param_node->declare_parameter("vision.model_path", "yolov8n.onnx");
    param_node->declare_parameter("vision.camera_topic", "/camera/image_raw");
    param_node->declare_parameter("vision.camera_device", "/dev/video0");
    param_node->declare_parameter("vision.frame_width", 640);
    param_node->declare_parameter("vision.frame_height", 480);
    param_node->declare_parameter("vision.target_fps", 30);
    param_node->declare_parameter("vision.confidence_threshold", 0.45);
    param_node->declare_parameter("vision.use_tensorrt", false);

    // Flight
    param_node->declare_parameter("flight.mavlink_url", "udp:127.0.0.1:14550");
    param_node->declare_parameter("flight.takeoff_altitude", 30.0);
    param_node->declare_parameter("flight.cruise_altitude", 60.0);
    param_node->declare_parameter("flight.cruise_speed", 25.0);

    // Comm
    param_node->declare_parameter("comm.server_ip", "192.168.1.100");
    param_node->declare_parameter("comm.server_port", 8080);

    // Parametreleri oku
    cfg.simulation_mode = param_node->get_parameter("simulation").as_bool();
    int mission_int = param_node->get_parameter("mission_type").as_int();
    cfg.mission_type = static_cast<siha::MissionType>(mission_int);
    cfg.comm.team_id = param_node->get_parameter("team_id").as_int();

    cfg.vision.model_path = param_node->get_parameter("vision.model_path").as_string();
    cfg.vision.camera_topic = param_node->get_parameter("vision.camera_topic").as_string();
    cfg.vision.camera_device = param_node->get_parameter("vision.camera_device").as_string();
    cfg.vision.frame_width = param_node->get_parameter("vision.frame_width").as_int();
    cfg.vision.frame_height = param_node->get_parameter("vision.frame_height").as_int();
    cfg.vision.target_fps = param_node->get_parameter("vision.target_fps").as_int();
    cfg.vision.confidence_thresh =
        static_cast<float>(param_node->get_parameter("vision.confidence_threshold").as_double());
    cfg.vision.use_tensorrt = param_node->get_parameter("vision.use_tensorrt").as_bool();

    cfg.flight.mavlink_url = param_node->get_parameter("flight.mavlink_url").as_string();
    cfg.flight.takeoff_altitude = param_node->get_parameter("flight.takeoff_altitude").as_double();
    cfg.flight.cruise_altitude = param_node->get_parameter("flight.cruise_altitude").as_double();
    cfg.flight.cruise_speed = param_node->get_parameter("flight.cruise_speed").as_double();

    cfg.comm.server_ip = param_node->get_parameter("comm.server_ip").as_string();
    cfg.comm.server_port = param_node->get_parameter("comm.server_port").as_int();

    return cfg;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::cout << R"(
    ╔══════════════════════════════════════════════════╗
    ║     SAEROTECH SİHA OTONOM SİSTEM v1.0           ║
    ║     TEKNOFEST 2026 — Savaşan İHA Yarışması       ║
    ╚══════════════════════════════════════════════════╝
    )" << std::endl;

    // Parametreleri yüklemek için geçici node
    auto param_node = std::make_shared<rclcpp::Node>("param_loader");
    auto config = load_config(param_node);
    param_node.reset();

    std::cout << "[Config] Mod: " << (config.simulation_mode ? "SİMÜLASYON" : "GERÇEK")
              << std::endl;
    std::cout << "[Config] Görev: " << (config.mission_type == siha::MissionType::SAVASAN_IHA
                                         ? "SAVAŞAN İHA" : "KAMİKAZE")
              << std::endl;
    std::cout << "[Config] Takım: " << config.comm.team_id << std::endl;

    // Ana görev yöneticisini oluştur
    auto mission = std::make_shared<siha::MissionController>(config);

    // Multi-threaded executor (vision + flight + comm paralel çalışsın)
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);  // 4 thread
    executor.add_node(mission);

    std::cout << "[System] Çalışıyor. Ctrl+C ile durdurun." << std::endl;
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
