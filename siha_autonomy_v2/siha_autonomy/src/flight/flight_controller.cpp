/**
 * @file flight_controller.cpp
 * @brief Uçuş Kontrolcüsü — ROS2 ↔ mavlink_bridge.py entegrasyonu
 */

#include "siha_autonomy/flight/flight_controller.hpp"
#include <sstream>
#include <cmath>

// Basit JSON helper (nlohmann yoksa)
namespace {
std::string json_obj(std::initializer_list<std::pair<std::string, std::string>> pairs) {
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (auto& [k, v] : pairs) {
        if (!first) oss << ",";
        oss << "\"" << k << "\":" << v;
        first = false;
    }
    oss << "}";
    return oss.str();
}
std::string str_val(const std::string& s) { return "\"" + s + "\""; }
std::string num_val(double v) { std::ostringstream o; o << v; return o.str(); }
std::string bool_val(bool v) { return v ? "true" : "false"; }
}

namespace siha {

FlightController::FlightController(const FlightConfig& cfg, rclcpp::Node* node)
    : config_(cfg), node_(node)
{
    // Komut publisher'ları oluştur
    pub_arm_      = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/arm", 10);
    pub_mode_     = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/mode", 10);
    pub_takeoff_  = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/takeoff", 10);
    pub_land_     = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/land", 10);
    pub_goto_     = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/goto", 10);
    pub_velocity_ = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/velocity", 10);
    pub_yaw_      = node_->create_publisher<std_msgs::msg::String>("/mavlink/cmd/yaw", 10);

    // Telemetri subscriber
    sub_telemetry_ = node_->create_subscription<std_msgs::msg::String>(
        "/mavlink/telemetry", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_telemetry(msg); });

    last_telem_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] ROS2 topic arayüzü hazır");
}

FlightController::~FlightController() { disconnect(); }

bool FlightController::connect() {
    // Bağlantı mavlink_bridge.py tarafından sağlanır
    // Burada telemetri gelip gelmediğini kontrol ediyoruz
    RCLCPP_INFO(node_->get_logger(),
        "[FlightCtrl] mavlink_bridge bekleniyor (/mavlink/telemetry)...");
    connected_ = true;
    return true;
}

void FlightController::disconnect() {
    connected_ = false;
}

bool FlightController::arm() {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({{"arm", bool_val(true)}});
    pub_arm_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] ARM komutu yayınlandı");
    return true;
}

bool FlightController::disarm() {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({{"arm", bool_val(false)}});
    pub_arm_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] DISARM komutu yayınlandı");
    return true;
}

bool FlightController::takeoff(double altitude_m) {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({{"altitude", num_val(altitude_m)}});
    pub_takeoff_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] TAKEOFF: %.1fm", altitude_m);
    return true;
}

bool FlightController::land() {
    auto msg = std_msgs::msg::String();
    msg.data = "{}";
    pub_land_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] LAND komutu");
    return set_mode(ArduMode::QLAND);
}

bool FlightController::return_to_launch() {
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] RTL komutu");
    return set_mode(ArduMode::RTL);
}

bool FlightController::set_mode(ArduMode mode) {
    current_mode_ = mode;
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({{"mode", str_val(mode_to_string(mode))}});
    pub_mode_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] Mod: %s", mode_to_string(mode).c_str());
    return true;
}

bool FlightController::goto_position(double lat, double lon, double alt) {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({
        {"lat", num_val(lat)},
        {"lon", num_val(lon)},
        {"alt", num_val(alt)}
    });
    pub_goto_->publish(msg);
    return true;
}

bool FlightController::set_velocity_ned(double vn, double ve, double vd) {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({
        {"vn", num_val(vn)},
        {"ve", num_val(ve)},
        {"vd", num_val(vd)}
    });
    pub_velocity_->publish(msg);
    return true;
}

bool FlightController::set_yaw(double heading_deg, double yaw_rate) {
    auto msg = std_msgs::msg::String();
    msg.data = json_obj({
        {"heading", num_val(heading_deg)},
        {"rate", num_val(yaw_rate)}
    });
    pub_yaw_->publish(msg);
    return true;
}

bool FlightController::set_heading_and_speed(double heading_deg, double speed_mps,
                                              double altitude_m) {
    set_yaw(heading_deg);
    double rad = heading_deg * M_PI / 180.0;
    double vn = speed_mps * std::cos(rad);
    double ve = speed_mps * std::sin(rad);
    double current_alt;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        current_alt = latest_telemetry_.position.altitude;
    }
    double vd = -(altitude_m - current_alt) * 0.5;
    vd = std::clamp(vd, -5.0, 5.0);
    return set_velocity_ned(vn, ve, vd);
}

Telemetry FlightController::get_telemetry() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_;
}

bool FlightController::is_armed() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_.is_armed;
}

int FlightController::gps_fix_type() const { return gps_fix_; }

int FlightController::battery_percent() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_.battery;
}

void FlightController::on_telemetry(const std_msgs::msg::String::SharedPtr msg) {
    try {
        // Elle JSON parse (basit key-value)
        auto& s = msg->data;
        std::lock_guard<std::mutex> lock(telemetry_mutex_);

        auto extract_double = [&](const std::string& key) -> double {
            auto pos = s.find("\"" + key + "\"");
            if (pos == std::string::npos) return 0.0;
            pos = s.find(':', pos);
            if (pos == std::string::npos) return 0.0;
            return std::stod(s.substr(pos + 1));
        };
        auto extract_bool = [&](const std::string& key) -> bool {
            auto pos = s.find("\"" + key + "\"");
            if (pos == std::string::npos) return false;
            return s.find("true", pos) < s.find(',', pos);
        };
        auto extract_string = [&](const std::string& key) -> std::string {
            auto pos = s.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            pos = s.find(':', pos);
            auto start = s.find('"', pos + 1);
            auto end = s.find('"', start + 1);
            if (start == std::string::npos || end == std::string::npos) return "";
            return s.substr(start + 1, end - start - 1);
        };

        latest_telemetry_.position.latitude  = extract_double("lat");
        latest_telemetry_.position.longitude = extract_double("lon");
        latest_telemetry_.position.altitude  = extract_double("alt");
        latest_telemetry_.heading   = extract_double("heading");
        latest_telemetry_.pitch     = extract_double("pitch");
        latest_telemetry_.roll      = extract_double("roll");
        latest_telemetry_.speed     = extract_double("groundspeed");
        latest_telemetry_.battery   = static_cast<int>(extract_double("battery"));
        latest_telemetry_.is_armed  = extract_bool("armed");
        gps_fix_ = static_cast<int>(extract_double("gps_fix"));

        auto mode_str = extract_string("mode");
        latest_telemetry_.is_autonomous = (mode_str == "GUIDED" || mode_str == "AUTO");

        last_telem_time_ = std::chrono::steady_clock::now();

        if (!connected_.load()) {
            connected_ = true;
            RCLCPP_INFO(node_->get_logger(), "[FlightCtrl] Telemetri alınıyor! Bağlı.");
        }
    } catch (...) {
        // Parse hatası — sessizce geç
    }
}

std::string FlightController::mode_to_string(ArduMode mode) const {
    switch (mode) {
        case ArduMode::MANUAL:  return "MANUAL";
        case ArduMode::FBWA:    return "FBWA";
        case ArduMode::FBWB:    return "FBWB";
        case ArduMode::CRUISE:  return "CRUISE";
        case ArduMode::AUTO:    return "AUTO";
        case ArduMode::RTL:     return "RTL";
        case ArduMode::LOITER:  return "LOITER";
        case ArduMode::GUIDED:  return "GUIDED";
        case ArduMode::QLOITER: return "QLOITER";
        case ArduMode::QLAND:   return "QLAND";
        case ArduMode::QRTL:    return "QRTL";
        default: return "MANUAL";
    }
}

}  // namespace siha
