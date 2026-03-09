/**
 * @file pid_controller.cpp
 * @brief PID Kontrolcü implementasyonu
 */

#include "siha_autonomy/flight/pid_controller.hpp"
#include <cmath>

namespace siha {

PIDController::PIDController(double kp, double ki, double kd,
                              double output_min, double output_max)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(output_min), output_max_(output_max)
{
    // Anti-windup: integral sınırı = çıktı sınırının 2 katı
    integral_max_ = std::abs(output_max) * 2.0;
}

double PIDController::compute(double error, double dt) {
    if (dt <= 0.0) return last_output_;

    // Proportional
    double p_term = kp_ * error;

    // Integral (anti-windup ile)
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_max_, integral_max_);
    double i_term = ki_ * integral_;

    // Derivative (ilk çağrıda atla)
    double d_term = 0.0;
    if (!first_call_) {
        double derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    }
    first_call_ = false;
    prev_error_ = error;

    // Toplam çıktı
    double output = p_term + i_term + d_term;

    // Clamp
    output = std::clamp(output, output_min_, output_max_);

    last_output_ = output;
    return output;
}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void PIDController::set_output_limits(double min_val, double max_val) {
    output_min_ = min_val;
    output_max_ = max_val;
    integral_max_ = std::abs(max_val) * 2.0;
}

void PIDController::reset_integral() {
    integral_ = 0.0;
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    last_output_ = 0.0;
    first_call_ = true;
}

}  // namespace siha
