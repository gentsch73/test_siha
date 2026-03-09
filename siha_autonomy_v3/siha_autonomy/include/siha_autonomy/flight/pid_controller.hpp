/**
 * @file pid_controller.hpp
 * @brief Genel Amaçlı PID Kontrolcü
 *
 * Yaw, pitch, altitude kontrol döngülerinde kullanılır.
 * Anti-windup, derivative filter ve output clamping desteği.
 */
#pragma once

#include <chrono>
#include <algorithm>

namespace siha {

class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double output_min = -1.0, double output_max = 1.0);

    /// Hata değerini ver, kontrol çıktısını al
    double compute(double error, double dt);

    /// Kazançları güncelle
    void set_gains(double kp, double ki, double kd);

    /// Çıktı sınırlarını ayarla
    void set_output_limits(double min_val, double max_val);

    /// İntegral bileşenini sıfırla
    void reset_integral();

    /// Tümünü sıfırla
    void reset();

    // Getters
    double kp() const { return kp_; }
    double ki() const { return ki_; }
    double kd() const { return kd_; }
    double last_output() const { return last_output_; }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double last_output_ = 0.0;
    bool first_call_ = true;

    // Anti-windup
    double integral_max_;
};

}  // namespace siha
