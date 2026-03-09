/**
 * @file guidance_system.cpp
 * @brief Güdüm Sistemi implementasyonu
 */

#include "siha_autonomy/flight/guidance_system.hpp"
#include <cmath>

namespace siha {

GuidanceSystem::GuidanceSystem(const FlightConfig& cfg, const VisionConfig& vcfg)
    : flight_cfg_(cfg), vision_cfg_(vcfg),
      yaw_pid_(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_kd, -45.0, 45.0),
      pitch_pid_(cfg.pitch_kp, cfg.pitch_ki, cfg.pitch_kd, -20.0, 20.0),
      throttle_pid_(0.5, 0.01, 0.1, -5.0, 5.0)
{
    last_time_ = std::chrono::steady_clock::now();
}

GuidanceCommand GuidanceSystem::compute_tracking(
    int target_cx, int target_cy, int target_area,
    int frame_w, int frame_h,
    const Telemetry& own)
{
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) dt = 0.02;

    GuidanceCommand cmd;

    // Frame merkezi
    int cx = frame_w / 2;
    int cy = frame_h / 2;

    // Piksel hatası
    int error_x = target_cx - cx;  // pozitif = hedef sağda
    int error_y = target_cy - cy;  // pozitif = hedef aşağıda

    // Piksel hatasını açısal hataya dönüştür
    double yaw_error = pixel_to_angle(error_x, frame_w, h_fov_deg_);
    double pitch_error = -pixel_to_angle(error_y, frame_h, v_fov_deg_);  // ters

    // PID ile yaw düzeltmesi
    double yaw_correction = yaw_pid_.compute(yaw_error, dt);

    // PID ile pitch düzeltmesi
    double pitch_correction = pitch_pid_.compute(pitch_error, dt);

    // Hedef heading = mevcut heading + yaw düzeltmesi
    cmd.heading_deg = std::fmod(own.heading + yaw_correction + 360.0, 360.0);

    // Hedef pitch
    cmd.pitch_deg = pitch_correction;

    // Hedef irtifa (pitch'e göre ayarla)
    cmd.altitude_m = own.position.altitude + pitch_correction * 0.5;
    cmd.altitude_m = std::clamp(cmd.altitude_m,
                                 flight_cfg_.min_altitude,
                                 flight_cfg_.max_altitude);

    // Hız ayarı: hedef büyükse (yakınsa) yavaşla, küçükse hızlan
    double frame_area = static_cast<double>(frame_w * frame_h);
    double coverage = static_cast<double>(target_area) / frame_area;

    if (coverage > 0.15) {
        cmd.speed_mps = flight_cfg_.approach_speed;
    } else if (coverage > 0.05) {
        cmd.speed_mps = flight_cfg_.cruise_speed;
    } else {
        cmd.speed_mps = flight_cfg_.max_speed * 0.8;
    }

    cmd.yaw_rate = yaw_correction;
    cmd.is_valid = true;

    return cmd;
}

GuidanceCommand GuidanceSystem::compute_waypoint(
    const GeoPoint& target,
    const Telemetry& own,
    double speed)
{
    GuidanceCommand cmd;

    double bearing = bearing_between(own.position, target);
    double dist = distance_between(own.position, target);

    cmd.heading_deg = bearing;
    cmd.speed_mps = speed;
    cmd.altitude_m = target.altitude > 0 ? target.altitude : own.position.altitude;
    cmd.is_valid = true;

    return cmd;
}

GuidanceCommand GuidanceSystem::compute_evasion(
    double threat_bearing, double threat_distance,
    const Telemetry& own)
{
    GuidanceCommand cmd;

    // Tehdidin tam tersi yöne dön
    cmd.heading_deg = std::fmod(threat_bearing + 180.0, 360.0);

    // Mesafeye göre hız
    if (threat_distance < 50.0) {
        cmd.speed_mps = flight_cfg_.max_speed;
    } else {
        cmd.speed_mps = flight_cfg_.cruise_speed;
    }

    // İrtifa değiştir (tehdit aynı irtifadaysa)
    cmd.altitude_m = own.position.altitude + 15.0;
    cmd.altitude_m = std::clamp(cmd.altitude_m,
                                 flight_cfg_.min_altitude,
                                 flight_cfg_.max_altitude);

    cmd.is_valid = true;
    return cmd;
}

void GuidanceSystem::reset() {
    yaw_pid_.reset();
    pitch_pid_.reset();
    throttle_pid_.reset();
    last_time_ = std::chrono::steady_clock::now();
}

void GuidanceSystem::update_yaw_pid(double kp, double ki, double kd) {
    yaw_pid_.set_gains(kp, ki, kd);
}

void GuidanceSystem::update_pitch_pid(double kp, double ki, double kd) {
    pitch_pid_.set_gains(kp, ki, kd);
}

double GuidanceSystem::pixel_to_angle(int pixel_offset, int frame_size,
                                        double fov_deg) const
{
    return (static_cast<double>(pixel_offset) / frame_size) * fov_deg;
}

double GuidanceSystem::bearing_between(const GeoPoint& a, const GeoPoint& b) {
    double dlon = (b.longitude - a.longitude) * M_PI / 180.0;
    double lat1 = a.latitude * M_PI / 180.0;
    double lat2 = b.latitude * M_PI / 180.0;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * 180.0 / M_PI;
    return std::fmod(bearing + 360.0, 360.0);
}

double GuidanceSystem::distance_between(const GeoPoint& a, const GeoPoint& b) {
    double dlat = (b.latitude - a.latitude) * M_PI / 180.0;
    double dlon = (b.longitude - a.longitude) * M_PI / 180.0;
    double lat1 = a.latitude * M_PI / 180.0;
    double lat2 = b.latitude * M_PI / 180.0;

    double sin_dlat = std::sin(dlat / 2.0);
    double sin_dlon = std::sin(dlon / 2.0);
    double h = sin_dlat * sin_dlat +
               std::cos(lat1) * std::cos(lat2) * sin_dlon * sin_dlon;

    return 6371000.0 * 2.0 * std::asin(std::sqrt(h));  // metre
}

}  // namespace siha
