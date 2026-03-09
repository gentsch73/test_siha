/**
 * @file flight_controller.cpp
 * @brief Uçuş Kontrolcüsü implementasyonu — MAVLink stub
 *
 * NOT: Gerçek MAVLink entegrasyonu için mavlink C kütüphanesi
 * veya MAVSDK kullanılmalıdır. Bu dosya arayüz iskeletini sağlar.
 */

#include "siha_autonomy/flight/flight_controller.hpp"
#include <iostream>
#include <cmath>

namespace siha {

FlightController::FlightController(const FlightConfig& cfg)
    : config_(cfg)
{
}

FlightController::~FlightController() {
    disconnect();
}

bool FlightController::connect() {
    // TODO: Gerçek MAVLink bağlantısı
    // mavlink_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    // ...
    std::cout << "[FlightCtrl] Bağlanılıyor: " << config_.mavlink_url << std::endl;
    connected_ = true;
    running_ = true;
    // recv_thread_ = std::thread(&FlightController::mavlink_receive_loop, this);
    last_heartbeat_ = std::chrono::steady_clock::now();
    return true;
}

void FlightController::disconnect() {
    running_ = false;
    connected_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
}

bool FlightController::arm() {
    if (!connected_) return false;
    std::cout << "[FlightCtrl] ARM komutu gönderildi" << std::endl;
    // TODO: MAV_CMD_COMPONENT_ARM_DISARM gönder
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        latest_telemetry_.is_armed = true;
    }
    return true;
}

bool FlightController::disarm() {
    std::cout << "[FlightCtrl] DISARM komutu gönderildi" << std::endl;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        latest_telemetry_.is_armed = false;
    }
    return true;
}

bool FlightController::takeoff(double altitude_m) {
    std::cout << "[FlightCtrl] TAKEOFF: " << altitude_m << "m" << std::endl;
    // TODO: MAV_CMD_NAV_TAKEOFF gönder
    return true;
}

bool FlightController::land() {
    std::cout << "[FlightCtrl] LAND komutu" << std::endl;
    return set_mode(ArduMode::QLAND);
}

bool FlightController::return_to_launch() {
    std::cout << "[FlightCtrl] RTL komutu" << std::endl;
    return set_mode(ArduMode::RTL);
}

bool FlightController::set_mode(ArduMode mode) {
    current_mode_ = mode;
    // TODO: MAV_CMD_DO_SET_MODE gönder
    return true;
}

bool FlightController::goto_position(double lat, double lon, double alt) {
    // TODO: SET_POSITION_TARGET_GLOBAL_INT gönder
    return true;
}

bool FlightController::set_velocity_ned(double vn, double ve, double vd) {
    // TODO: SET_POSITION_TARGET_LOCAL_NED gönder (velocity modu)
    return true;
}

bool FlightController::set_yaw(double heading_deg, double yaw_rate) {
    // TODO: MAV_CMD_CONDITION_YAW gönder
    return true;
}

bool FlightController::set_heading_and_speed(double heading_deg,
                                               double speed_mps,
                                               double altitude_m) {
    // Heading, speed ve altitude komutlarını birleştir
    set_yaw(heading_deg);

    double rad = heading_deg * M_PI / 180.0;
    double vn = speed_mps * std::cos(rad);
    double ve = speed_mps * std::sin(rad);

    // İrtifa farkından dikey hız hesapla
    double current_alt;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        current_alt = latest_telemetry_.position.altitude;
    }
    double vd = -(altitude_m - current_alt) * 0.5;  // basit P kontrol
    vd = std::clamp(vd, -5.0, 5.0);

    return set_velocity_ned(vn, ve, vd);
}

Telemetry FlightController::get_telemetry() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_;
}

void FlightController::update_telemetry(const Telemetry& telem) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    latest_telemetry_ = telem;
}

bool FlightController::is_armed() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_.is_armed;
}

int FlightController::gps_fix_type() const {
    return 3;  // TODO: gerçek GPS fix
}

int FlightController::battery_percent() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return latest_telemetry_.battery;
}

void FlightController::mavlink_receive_loop() {
    // TODO: MAVLink mesajları parse et
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        last_heartbeat_ = std::chrono::steady_clock::now();
    }
}

void FlightController::send_heartbeat() {
    // TODO: HEARTBEAT mesajı gönder
}

}  // namespace siha
