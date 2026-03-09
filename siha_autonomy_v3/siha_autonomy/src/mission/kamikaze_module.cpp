/**
 * @file kamikaze_module.cpp
 */
#include "siha_autonomy/mission/kamikaze_module.hpp"
#include <iostream>

namespace siha {

KamikazeModule::KamikazeModule(const KamikazeConfig& cfg, const VisionConfig& vcfg)
    : config_(cfg)
{
    qr_detector_ = std::make_unique<QRDetector>();
}

void KamikazeModule::start(const GeoPoint& qr_position) {
    target_pos_ = qr_position;
    phase_ = KamikazePhase::CLIMBING;
    qr_success_ = false;
    qr_content_.clear();
    std::cout << "[Kamikaze] Görev başlatıldı. Hedef: "
              << qr_position.latitude << ", " << qr_position.longitude << std::endl;
}

KamikazePhase KamikazeModule::update(const Telemetry& own, const cv::Mat& frame) {
    switch (phase_) {
        case KamikazePhase::CLIMBING:
            if (own.position.altitude >= config_.min_dive_altitude) {
                phase_ = KamikazePhase::ALIGNING;
            }
            break;

        case KamikazePhase::ALIGNING:
            // Hizalama tamamlandığında dalışa geç (MissionController handle eder)
            break;

        case KamikazePhase::DIVING:
            phase_ = KamikazePhase::READING_QR;
            dive_start_time_ = std::chrono::steady_clock::now();
            [[fallthrough]];

        case KamikazePhase::READING_QR:
            if (!frame.empty() && !qr_success_) {
                auto result = qr_detector_->detect_and_decode(frame);
                if (result.found) {
                    qr_success_ = true;
                    qr_content_ = result.content;
                    std::cout << "[Kamikaze] QR OKUNDU: " << qr_content_ << std::endl;
                }
            }
            // Güvenli irtifaya gelince çek-çık
            if (own.position.altitude <= config_.safe_pullup_alt) {
                phase_ = KamikazePhase::PULLING_UP;
            }
            break;

        case KamikazePhase::PULLING_UP:
            if (own.position.altitude >= config_.safe_pullup_alt + 20.0) {
                phase_ = KamikazePhase::COMPLETED;
            }
            break;

        default:
            break;
    }
    return phase_;
}

void KamikazeModule::reset() {
    phase_ = KamikazePhase::IDLE;
    qr_success_ = false;
    qr_content_.clear();
}

}  // namespace siha
