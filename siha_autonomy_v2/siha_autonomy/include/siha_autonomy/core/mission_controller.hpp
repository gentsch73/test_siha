/**
 * @file mission_controller.hpp
 * @brief Ana Görev Yöneticisi — Tam ROS2 entegrasyonlu
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include "siha_autonomy/vision/yolo_detector.hpp"
#include "siha_autonomy/vision/target_tracker.hpp"
#include "siha_autonomy/vision/lockon_manager.hpp"
#include "siha_autonomy/flight/flight_controller.hpp"
#include "siha_autonomy/flight/guidance_system.hpp"
#include "siha_autonomy/mission/kamikaze_module.hpp"
#include "siha_autonomy/mission/evasion_module.hpp"
#include "siha_autonomy/mission/search_pattern.hpp"
#include "siha_autonomy/comm/server_comm.hpp"
#include "siha_autonomy/comm/telemetry_manager.hpp"
#include "siha_autonomy/safety/safety_monitor.hpp"
#include "siha_autonomy/recording/video_recorder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <unordered_set>

namespace siha {

class MissionController : public rclcpp::Node {
public:
    explicit MissionController(const SystemConfig& cfg);
    ~MissionController() override;

    void tick();
    FlightPhase current_phase() const { return phase_; }
    void set_mission_type(MissionType type);

private:
    // ── Durum Geçişleri ──
    void transition_to(FlightPhase new_phase);

    // ── Faz İşleyicileri ──
    void handle_idle();
    void handle_pre_arm();
    void handle_armed();
    void handle_takeoff();
    void handle_climb();
    void handle_mission_select();
    void handle_search();
    void handle_track();
    void handle_lockon();
    void handle_evade();
    void handle_kamikaze_climb();
    void handle_kamikaze_align();
    void handle_kamikaze_dive();
    void handle_kamikaze_pullup();
    void handle_loiter();
    void handle_rtl();
    void handle_land();
    void handle_boundary_return();
    void handle_signal_lost();
    void handle_emergency();

    // ── Yardımcı ──
    void process_vision_frame();
    void check_safety_conditions();
    void send_telemetry();
    void record_frame();
    bool is_target_blacklisted(int id);
    void blacklist_target(int id);

    // ── ROS2 Callback'ler ──
    void on_camera_image(const sensor_msgs::msg::Image::SharedPtr msg);
    void on_npc_telemetry(const std_msgs::msg::String::SharedPtr msg);

    // ── Konfigürasyon ──
    SystemConfig config_;
    FlightPhase  phase_ = FlightPhase::IDLE;
    MissionType  mission_type_ = MissionType::SAVASAN_IHA;

    // ── Alt Modüller ──
    std::unique_ptr<YoloDetector>      yolo_detector_;
    std::unique_ptr<TargetTracker>     target_tracker_;
    std::unique_ptr<LockonManager>     lockon_manager_;
    std::unique_ptr<FlightController>  flight_ctrl_;
    std::unique_ptr<GuidanceSystem>    guidance_;
    std::unique_ptr<KamikazeModule>    kamikaze_;
    std::unique_ptr<EvasionModule>     evasion_;
    std::unique_ptr<SearchPattern>     search_pattern_;
    std::unique_ptr<ServerComm>        server_comm_;
    std::unique_ptr<TelemetryManager>  telemetry_mgr_;
    std::unique_ptr<SafetyMonitor>     safety_monitor_;
    std::unique_ptr<VideoRecorder>     video_recorder_;
    std::unique_ptr<OverlayRenderer>   overlay_;

    // ── ROS2 Subscriptions ──
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_npc_;

    // ── Kamera verisi ──
    cv::Mat latest_camera_frame_;
    std::mutex camera_mutex_;
    bool has_new_frame_ = false;
    double camera_fps_ = 0.0;
    int camera_frame_count_ = 0;
    std::chrono::steady_clock::time_point camera_fps_start_;

    // ── Durum ──
    Telemetry own_telemetry_;
    std::unordered_set<int> blacklisted_targets_;

    // ── Timer'lar ──
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    // ── İstatistik ──
    int total_lockons_ = 0;
    std::chrono::steady_clock::time_point phase_entry_time_;
};

}  // namespace siha
