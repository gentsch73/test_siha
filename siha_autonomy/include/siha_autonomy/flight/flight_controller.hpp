#pragma once
#include "siha_autonomy/core/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include <chrono>

namespace siha {

enum class ArduMode : uint8_t {
    MANUAL = 0, FBWA = 5, FBWB = 6, CRUISE = 7,
    AUTO = 10, RTL = 11, LOITER = 12, GUIDED = 15,
    QLOITER = 18, QLAND = 20, QRTL = 21
};

class FlightController {
public:
    FlightController(const FlightConfig& cfg, rclcpp::Node* node);
    ~FlightController();
    bool connect();
    void disconnect();
    bool is_connected() const { return connected_.load(); }
    bool arm();
    bool disarm();
    bool takeoff(double altitude_m);
    bool land();
    bool return_to_launch();
    bool set_mode(ArduMode mode);
    ArduMode current_mode() const { return current_mode_; }
    bool goto_position(double lat, double lon, double alt);
    bool set_velocity_ned(double vn, double ve, double vd);
    bool set_yaw(double heading_deg, double yaw_rate_deg_s = 0.0);
    bool set_heading_and_speed(double heading_deg, double speed_mps, double altitude_m);
    Telemetry get_telemetry() const;
    bool is_armed() const;
    int gps_fix_type() const;
    int battery_percent() const;
    void on_heartbeat_lost(std::function<void()> cb) { heartbeat_lost_cb_ = cb; }

private:
    void on_telemetry(const std_msgs::msg::String::SharedPtr msg);
    std::string mode_to_string(ArduMode mode) const;

    FlightConfig config_;
    rclcpp::Node* node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_arm_, pub_mode_,
        pub_takeoff_, pub_land_, pub_goto_, pub_velocity_, pub_guided_heading_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_telemetry_;
    std::atomic<bool> connected_{false};
    ArduMode current_mode_ = ArduMode::MANUAL;
    Telemetry latest_telemetry_;
    mutable std::mutex telemetry_mutex_;
    int gps_fix_ = 0;
    std::function<void()> heartbeat_lost_cb_;
    std::chrono::steady_clock::time_point last_telem_time_;
};

}  // namespace siha
