/**
 * @file guidance_system.hpp
 * @brief Güdüm Sistemi — Hedef takip ve manevra algoritmaları
 *
 * Görüntü işlemeden gelen hedef bilgisini, uçuş komutlarına dönüştürür.
 * Pure Pursuit, Proportional Navigation ve PID tabanlı hata düzeltme.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include "siha_autonomy/flight/pid_controller.hpp"
#include <opencv2/core.hpp>

namespace siha {

/// Güdüm komutu (uçuş kontrolcüsüne gönderilecek)
struct GuidanceCommand {
    double heading_deg  = 0.0;   // Hedef yön (derece)
    double pitch_deg    = 0.0;   // Dikilme açısı
    double speed_mps    = 0.0;   // Hedef hız
    double altitude_m   = 0.0;   // Hedef irtifa
    double yaw_rate     = 0.0;   // Yaw dönüş hızı (derece/sn)
    bool   is_valid     = false;
};

/**
 * @class GuidanceSystem
 * @brief Hedef takip güdüm algoritması.
 *
 * Kamera frame'indeki hedef pozisyonunu,
 * uçağın yapması gereken manevralara dönüştürür.
 */
class GuidanceSystem {
public:
    explicit GuidanceSystem(const FlightConfig& cfg, const VisionConfig& vcfg);

    /**
     * @brief Kameradaki hedef pozisyonundan güdüm komutu üret.
     * @param target_cx Hedef merkez X (piksel)
     * @param target_cy Hedef merkez Y (piksel)
     * @param target_area Hedef alanı (piksel²)
     * @param frame_w Frame genişliği
     * @param frame_h Frame yüksekliği
     * @param own_telemetry Kendi telemetrimiz
     * @return Güdüm komutu
     */
    GuidanceCommand compute_tracking(
        int target_cx, int target_cy, int target_area,
        int frame_w, int frame_h,
        const Telemetry& own_telemetry);

    /**
     * @brief GPS koordinatına doğru güdüm.
     * @param target Hedef GPS konumu
     * @param own Kendi konumumuz
     * @param speed Hedef hız
     * @return Güdüm komutu
     */
    GuidanceCommand compute_waypoint(
        const GeoPoint& target,
        const Telemetry& own,
        double speed);

    /**
     * @brief Rakip İHA'dan kaçınma güdümü.
     * @param threat_bearing Tehdit yönü (derece)
     * @param threat_distance Tehdit mesafesi (metre)
     * @param own Kendi telemetrimiz
     * @return Güdüm komutu
     */
    GuidanceCommand compute_evasion(
        double threat_bearing, double threat_distance,
        const Telemetry& own);

    /// PID kazançlarını güncelle
    void update_yaw_pid(double kp, double ki, double kd);
    void update_pitch_pid(double kp, double ki, double kd);

    /// PID'leri sıfırla
    void reset();

    /// İki GPS noktası arası bearing ve mesafe (public utility)
    static double bearing_between(const GeoPoint& a, const GeoPoint& b);
    static double distance_between(const GeoPoint& a, const GeoPoint& b);

private:
    /// Piksel hatasını açısal hataya dönüştür
    double pixel_to_angle(int pixel_offset, int frame_size, double fov_deg) const;

    FlightConfig flight_cfg_;
    VisionConfig vision_cfg_;

    PIDController yaw_pid_;
    PIDController pitch_pid_;
    PIDController throttle_pid_;

    // Tahmini kamera FOV (derece)
    double h_fov_deg_ = 70.0;
    double v_fov_deg_ = 55.0;

    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace siha
