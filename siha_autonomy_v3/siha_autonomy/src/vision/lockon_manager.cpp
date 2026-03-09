/**
 * @file lockon_manager.cpp
 * @brief Kilitlenme Yöneticisi implementasyonu
 */

#include "siha_autonomy/vision/lockon_manager.hpp"
#include <cmath>

namespace siha {

LockonManager::LockonManager(const VisionConfig& cfg)
    : config_(cfg)
{
}

LockonInfo LockonManager::process(const TrackedTarget& target,
                                   int frame_w, int frame_h)
{
    // Kara listedeki hedefe kilitlenemezsin
    if (is_blacklisted(target.track_id)) {
        return process_no_target();
    }

    // Farklı hedefe geçiş — sayaçları sıfırla
    if (info_.target_id != target.track_id && info_.target_id >= 0) {
        reset();
    }
    info_.target_id = target.track_id;

    bool in_strike = is_in_strike_zone(target.bbox, frame_w, frame_h);
    bool size_ok   = meets_size_requirement(target.bbox, frame_w, frame_h);

    total_frames_in_attempt_++;

    if (in_strike && size_ok) {
        // Hedef vuruş alanında ve yeterli boyutta
        continuous_lock_frames_++;

        if (continuous_lock_frames_ == 1) {
            // Kilitlenme başlangıcı
            lock_start_time_ = std::chrono::steady_clock::now();
            info_.lock_start = lock_start_time_;
        }

        auto now = std::chrono::steady_clock::now();
        info_.lock_duration_s = std::chrono::duration<double>(
            now - lock_start_time_).count();

        // 4 saniyelik kilitlenme sayacını güncelle
        info_.lock_counter = static_cast<int>(std::floor(info_.lock_duration_s));

        info_.is_locked = (info_.lock_duration_s > 0.5);  // en az 0.5 sn
        info_.lock_rect = target.bbox;

        // Tolerans sayacını sıfırla (başarılı frame)
        tolerance_counter_ = 0;

    } else {
        // Hedef vuruş alanı dışında
        missed_frames_in_attempt_++;

        // Frame kaybı toleransı kontrolü (%5)
        double loss_ratio = 0.0;
        if (total_frames_in_attempt_ > 0) {
            loss_ratio = static_cast<double>(missed_frames_in_attempt_) /
                         total_frames_in_attempt_;
        }

        if (loss_ratio > config_.frame_loss_tolerance) {
            // Tolerans aşıldı → kilitlenme başarısız
            tolerance_counter_++;
        }

        if (tolerance_counter_ > MAX_TOLERANCE) {
            // Tamamen başarısız — sıfırla
            reset();
        }
    }

    info_.tolerance_counter = tolerance_counter_;
    info_.total_lost_frames = missed_frames_in_attempt_;

    return info_;
}

LockonInfo LockonManager::process_no_target() {
    missed_frames_in_attempt_++;
    tolerance_counter_++;

    if (tolerance_counter_ > MAX_TOLERANCE) {
        reset();
    }

    info_.tolerance_counter = tolerance_counter_;
    return info_;
}

void LockonManager::confirm_lockon(int target_id) {
    add_to_blacklist(target_id);
    last_locked_id_ = target_id;
    reset();
}

bool LockonManager::is_blacklisted(int target_id) const {
    return blacklist_.count(target_id) > 0;
}

void LockonManager::add_to_blacklist(int target_id) {
    blacklist_.insert(target_id);
}

void LockonManager::reset() {
    info_ = LockonInfo{};
    continuous_lock_frames_ = 0;
    total_frames_in_attempt_ = 0;
    missed_frames_in_attempt_ = 0;
    tolerance_counter_ = 0;
}

bool LockonManager::is_in_strike_zone(const BoundingBox& bbox,
                                        int frame_w, int frame_h) const
{
    auto sz = compute_strike_zone(frame_w, frame_h);

    // Hedef merkezi vuruş alanı içinde mi?
    int cx = bbox.center_x();
    int cy = bbox.center_y();

    return (cx >= sz.x1 && cx <= sz.x2 && cy >= sz.y1 && cy <= sz.y2);
}

bool LockonManager::meets_size_requirement(const BoundingBox& bbox,
                                             int frame_w, int frame_h) const
{
    double frame_area = static_cast<double>(frame_w * frame_h);
    double target_area = static_cast<double>(bbox.area());
    double coverage = target_area / frame_area;

    // %6 güvenli sınır (şartname %5)
    return coverage >= config_.min_target_pct;
}

LockonManager::StrikeZone LockonManager::compute_strike_zone(
    int frame_w, int frame_h) const
{
    // Vuruş alanı: merkeze göre yatay %50, dikey %80
    double margin_h = (1.0 - config_.strike_zone_h_pct) / 2.0;
    double margin_v = (1.0 - config_.strike_zone_v_pct) / 2.0;

    StrikeZone sz;
    sz.x1 = static_cast<int>(frame_w * margin_h);
    sz.y1 = static_cast<int>(frame_h * margin_v);
    sz.x2 = static_cast<int>(frame_w * (1.0 - margin_h));
    sz.y2 = static_cast<int>(frame_h * (1.0 - margin_v));

    return sz;
}

}  // namespace siha
