/**
 * @file mission_controller.hpp
 * @brief Ana Görev Yöneticisi — Tüm uçuş fazlarını yöneten sonlu durum makinesi
 *
 * Takeoff → Search → Track → Lockon → RTL → Land akışını
 * ve tüm dallanma/hata durumlarını kontrol eder.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include "siha_autonomy/vision/yolo_detector.hpp"
#include "siha_autonomy/vision/target_tracker.hpp"
#include "siha_autonomy/vision/lockon_manager.hpp"
#include "siha_autonomy/vision/video_pipeline.hpp"
#include "siha_autonomy/flight/flight_controller.hpp"
#include "siha_autonomy/flight/guidance_system.hpp"
#include "siha_autonomy/mission/kamikaze_module.hpp"
#include "siha_autonomy/mission/evasion_module.hpp"
#include "siha_autonomy/mission/search_pattern.hpp"
#include "siha_autonomy/comm/server_comm.hpp"
#include "siha_autonomy/comm/telemetry_manager.hpp"
#include "siha_autonomy/safety/safety_monitor.hpp"
#include "siha_autonomy/recording/video_recorder.hpp"
#include "siha_autonomy/decision/decision_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <functional>
#include <unordered_set>

namespace siha {

/**
 * @class MissionController
 * @brief Merkezi görev yöneticisi.
 *
 * Her ROS2 spin döngüsünde çağrılan `tick()` metodu,
 * mevcut duruma göre doğru alt modülü çalıştırır.
 */
class MissionController : public rclcpp::Node {
public:
    explicit MissionController(const SystemConfig& cfg);
    ~MissionController() override;

    /// Ana döngüde çağrılır (timer callback)
    void tick();

    /// Mevcut uçuş fazı
    FlightPhase current_phase() const { return phase_; }

    /// Görev tipi değiştir
    void set_mission_type(MissionType type);

private:
    // ── Durum Geçişleri ──
    void transition_to(FlightPhase new_phase);
    bool can_transition(FlightPhase from, FlightPhase to) const;

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

    // ── Yardımcı Metotlar ──
    void process_vision_frame();          // Kameradan frame al → YOLO → Tracker
    void check_safety_conditions();       // Güvenlik kontrolü (her tick)
    void send_telemetry();                // Sunucuya telemetri gönder
    void update_server_time();            // Sunucu saatini güncelle
    void record_frame();                  // Video kayıt
    void publish_debug_image(const cv::Mat& frame,
                              const std::vector<BoundingBox>& detections,
                              const LockonInfo& lock_info);  // Debug görüntü yayınla
    bool is_target_blacklisted(int id);   // Kara liste kontrolü
    void blacklist_target(int id);        // Vurulan hedefi kara listeye ekle

    // ── ROS2 Abonelik Geri Çağırmaları ──
    void on_sunucu_telemetri(const std_msgs::msg::String::SharedPtr msg);
    void on_mavlink_telemetry(const std_msgs::msg::String::SharedPtr msg);

    // ── MAVLink Bridge Komut Yayınlama ──
    void publish_arm_cmd(bool arm);
    void publish_mode_cmd(const std::string& mode_str);
    void publish_takeoff_cmd(double altitude_m);

    // ── Alt Modüller (Composition over Inheritance) ──
    SystemConfig config_;
    FlightPhase  phase_ = FlightPhase::IDLE;
    MissionType  mission_type_ = MissionType::SAVASAN_IHA;

    std::unique_ptr<VideoPipeline>     vision_pipeline_;
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
    std::unique_ptr<DecisionEngine>    decision_engine_;

    // ── Durum Verileri ──
    Telemetry           own_telemetry_;
    ServerTime          server_time_;
    std::vector<CompetitorUAV> competitors_;
    std::unordered_set<int>    blacklisted_targets_;  // vurulan hedefler

    // ── ROS2 Pub/Sub ──
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  pub_debug_image_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_mavlink_arm_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_mavlink_mode_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_mavlink_takeoff_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sunucu_telemetri_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mavlink_telemetry_;

    // ── Zamanlayıcılar ──
    rclcpp::TimerBase::SharedPtr main_timer_;        // ana döngü (50 Hz)
    rclcpp::TimerBase::SharedPtr telemetry_timer_;   // telemetri (2 Hz)

    // ── İstatistik ──
    int    total_lockons_    = 0;
    double mission_start_time_ = 0.0;
    double autonomous_time_    = 0.0;

    // ── Faz geçiş zamanı ──
    std::chrono::steady_clock::time_point phase_entry_time_;
};

}  // namespace siha
