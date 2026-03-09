/**
 * @file guidance_system.hpp
 * @brief ProNav + PID Hibrit Güdüm Sistemi
 *
 * Katman 1: ProNav (öngörücü takip — LOS rate tabanlı)
 * Katman 2: PID (stabilizasyon — hata düzeltme)
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include "siha_autonomy/flight/pid_controller.hpp"

namespace siha {

struct GuidanceCommand {
    double heading_deg = 0; double pitch_deg = 0;
    double speed_mps = 0;   double altitude_m = 0;
    double yaw_rate = 0;    bool is_valid = false;
};

/// ProNav state (frame'ler arası LOS takibi)
struct ProNavState {
    double prev_los_yaw   = 0;  // önceki LOS açısı (yaw)
    double prev_los_pitch = 0;  // önceki LOS açısı (pitch)
    bool   initialized    = false;
    double N = 4.0;             // navigasyon sabiti (3-5 arası)
};

class GuidanceSystem {
public:
    GuidanceSystem(const FlightConfig& cfg, const VisionConfig& vcfg);

    /**
     * @brief ProNav + PID hibrit takip güdümü.
     * Kameradaki hedef pozisyonundan komut üretir.
     * ProNav: LOS rate → öngörücü düzeltme
     * PID: kalan hatayı sıfırlama
     */
    GuidanceCommand compute_tracking(
        int target_cx, int target_cy, int target_area,
        int frame_w, int frame_h, const Telemetry& own);

    /// GPS noktasına güdüm
    GuidanceCommand compute_waypoint(
        const GeoPoint& target, const Telemetry& own, double speed);

    /// Kaçınma güdümü
    GuidanceCommand compute_evasion(
        double threat_bearing, double threat_distance, const Telemetry& own);

    /// ProNav parametresini ayarla
    void set_pronav_gain(double N) { pronav_.N = N; }

    void update_yaw_pid(double kp, double ki, double kd);
    void update_pitch_pid(double kp, double ki, double kd);
    void reset();

    static double bearing_between(const GeoPoint& a, const GeoPoint& b);
    static double distance_between(const GeoPoint& a, const GeoPoint& b);

private:
    double pixel_to_angle(int offset, int size, double fov) const;

    FlightConfig flight_cfg_;
    VisionConfig vision_cfg_;
    PIDController yaw_pid_, pitch_pid_, throttle_pid_;
    ProNavState pronav_;
    double h_fov_deg_ = 70.0, v_fov_deg_ = 55.0;
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace siha
