/**
 * @file guidance_system.cpp
 * @brief ProNav + PID Hibrit Güdüm
 *
 * ProNav: a_cmd = N × V_closing × dλ/dt
 *   - λ = LOS açısı (kameradan piksel → açı)
 *   - dλ/dt = LOS rate (frame'ler arası fark)
 *   - N = navigasyon sabiti (genelde 3-5)
 *
 * PID: ProNav çıktısına eklenen stabilizasyon katmanı
 */

#include "siha_autonomy/flight/guidance_system.hpp"
#include <cmath>
#include <algorithm>

namespace siha {

GuidanceSystem::GuidanceSystem(const FlightConfig& cfg, const VisionConfig& vcfg)
    : flight_cfg_(cfg), vision_cfg_(vcfg),
      yaw_pid_(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_kd, -30.0, 30.0),
      pitch_pid_(cfg.pitch_kp, cfg.pitch_ki, cfg.pitch_kd, -15.0, 15.0),
      throttle_pid_(0.5, 0.01, 0.1, -5.0, 5.0)
{
    last_time_ = std::chrono::steady_clock::now();
    pronav_.N = 4.0;
}

GuidanceCommand GuidanceSystem::compute_tracking(
    int target_cx, int target_cy, int target_area,
    int frame_w, int frame_h, const Telemetry& own)
{
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;
    if (dt <= 0.001 || dt > 1.0) dt = 0.05;

    GuidanceCommand cmd;
    int cx = frame_w / 2, cy = frame_h / 2;

    // ── Normalize LOS açıları (-1 .. +1) ──
    double los_yaw  = static_cast<double>(target_cx - cx) / (frame_w / 2.0);
    double los_pitch = static_cast<double>(target_cy - cy) / (frame_h / 2.0);

    // ── ProNav: LOS rate hesapla ──
    double pronav_yaw = 0, pronav_pitch = 0;
    if (pronav_.initialized) {
        double los_rate_yaw  = (los_yaw  - pronav_.prev_los_yaw)  / dt;
        double los_rate_pitch = (los_pitch - pronav_.prev_los_pitch) / dt;

        // a_cmd = N × V × dλ/dt
        double V = std::max(own.speed, 10.0);  // minimum 10 m/s
        pronav_yaw   = pronav_.N * V * los_rate_yaw * 0.1;   // ölçekleme
        pronav_pitch  = pronav_.N * V * los_rate_pitch * 0.05;

        // Sabit kanat limitleri
        pronav_yaw  = std::clamp(pronav_yaw, -25.0, 25.0);
        pronav_pitch = std::clamp(pronav_pitch, -10.0, 10.0);
    }
    pronav_.prev_los_yaw  = los_yaw;
    pronav_.prev_los_pitch = los_pitch;
    pronav_.initialized = true;

    // ── PID: Mevcut hata düzeltme ──
    double yaw_error  = pixel_to_angle(target_cx - cx, frame_w, h_fov_deg_);
    double pitch_error = -pixel_to_angle(target_cy - cy, frame_h, v_fov_deg_);
    double pid_yaw   = yaw_pid_.compute(yaw_error, dt);
    double pid_pitch = pitch_pid_.compute(pitch_error, dt);

    // ── Hibrit: ProNav (öngörücü) + PID (düzeltici) ──
    double total_yaw   = pronav_yaw  * 0.6 + pid_yaw  * 0.4;
    double total_pitch  = pronav_pitch * 0.6 + pid_pitch * 0.4;

    // ── Komut oluştur ──
    cmd.heading_deg = std::fmod(own.heading + total_yaw + 360.0, 360.0);
    cmd.pitch_deg = total_pitch;

    // İrtifa (pitch'e göre)
    cmd.altitude_m = own.position.altitude + total_pitch * 0.3;
    cmd.altitude_m = std::clamp(cmd.altitude_m, flight_cfg_.min_altitude, flight_cfg_.max_altitude);

    // Hız (mesafeye göre)
    double frame_area = static_cast<double>(frame_w * frame_h);
    double coverage = static_cast<double>(target_area) / frame_area;
    if (coverage > 0.15) cmd.speed_mps = flight_cfg_.approach_speed;
    else if (coverage > 0.05) cmd.speed_mps = flight_cfg_.cruise_speed;
    else cmd.speed_mps = flight_cfg_.max_speed * 0.8;

    cmd.yaw_rate = total_yaw;
    cmd.is_valid = true;
    return cmd;
}

GuidanceCommand GuidanceSystem::compute_waypoint(
    const GeoPoint& target, const Telemetry& own, double speed) {
    GuidanceCommand cmd;
    cmd.heading_deg = bearing_between(own.position, target);
    cmd.speed_mps = speed;
    cmd.altitude_m = target.altitude > 0 ? target.altitude : own.position.altitude;
    cmd.is_valid = true;
    return cmd;
}

GuidanceCommand GuidanceSystem::compute_evasion(
    double threat_bearing, double threat_distance, const Telemetry& own) {
    GuidanceCommand cmd;
    // Tehdidin 90° sağına dön (tam tersi değil — sabit kanat için daha verimli)
    cmd.heading_deg = std::fmod(threat_bearing + 90.0, 360.0);
    cmd.speed_mps = flight_cfg_.max_speed;
    cmd.altitude_m = own.position.altitude + 10.0;
    cmd.altitude_m = std::clamp(cmd.altitude_m, flight_cfg_.min_altitude, flight_cfg_.max_altitude);
    cmd.is_valid = true;
    return cmd;
}

void GuidanceSystem::reset() {
    yaw_pid_.reset(); pitch_pid_.reset(); throttle_pid_.reset();
    pronav_.initialized = false;
    last_time_ = std::chrono::steady_clock::now();
}

void GuidanceSystem::update_yaw_pid(double kp, double ki, double kd) { yaw_pid_.set_gains(kp,ki,kd); }
void GuidanceSystem::update_pitch_pid(double kp, double ki, double kd) { pitch_pid_.set_gains(kp,ki,kd); }

double GuidanceSystem::pixel_to_angle(int off, int sz, double fov) const {
    return (static_cast<double>(off) / sz) * fov;
}

double GuidanceSystem::bearing_between(const GeoPoint& a, const GeoPoint& b) {
    double dlon = (b.longitude-a.longitude)*M_PI/180.0;
    double la = a.latitude*M_PI/180.0, lb = b.latitude*M_PI/180.0;
    double x = std::sin(dlon)*std::cos(lb);
    double y = std::cos(la)*std::sin(lb) - std::sin(la)*std::cos(lb)*std::cos(dlon);
    return std::fmod(std::atan2(x,y)*180.0/M_PI + 360.0, 360.0);
}

double GuidanceSystem::distance_between(const GeoPoint& a, const GeoPoint& b) {
    double dlat=(b.latitude-a.latitude)*M_PI/180, dlon=(b.longitude-a.longitude)*M_PI/180;
    double la=a.latitude*M_PI/180, lb=b.latitude*M_PI/180;
    double h = std::sin(dlat/2)*std::sin(dlat/2) + std::cos(la)*std::cos(lb)*std::sin(dlon/2)*std::sin(dlon/2);
    return 6371000.0*2.0*std::asin(std::sqrt(h));
}

}
