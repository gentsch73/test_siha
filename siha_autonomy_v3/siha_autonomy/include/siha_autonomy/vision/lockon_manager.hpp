/**
 * @file lockon_manager.hpp
 * @brief Kilitlenme Yöneticisi
 *
 * Şartname kurallarına tam uygun kilitlenme mantığı:
 *   - Hedef vuruş alanı içinde mi? (merkez %50 yatay, %80 dikey)
 *   - Hedef %5+ kaplıyor mu? (güvenli sınır %6)
 *   - 4 saniye kesintisiz kilitlenme
 *   - 1 saniyelik tolerans payı
 *   - %5 frame kaybı toleransı
 *   - Aynı hedefe art arda kilitlenememe kuralı
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <opencv2/core.hpp>
#include <chrono>
#include <unordered_set>

namespace siha {

/**
 * @class LockonManager
 * @brief Kilitlenme karar ve sayaç yöneticisi.
 */
class LockonManager {
public:
    explicit LockonManager(const VisionConfig& cfg);

    /**
     * @brief Her frame'de çağrılır. Hedefin kilitlenme koşullarını kontrol eder.
     * @param target Takip edilen hedef bilgisi
     * @param frame_w Frame genişliği
     * @param frame_h Frame yüksekliği
     * @return Güncel kilitlenme bilgisi
     */
    LockonInfo process(const TrackedTarget& target, int frame_w, int frame_h);

    /// Hedef yoksa veya kayıpsa çağrılır
    LockonInfo process_no_target();

    /// Kilitlenme başarılı olduğunda çağrılır (sayaçları sıfırla, kara listeye ekle)
    void confirm_lockon(int target_id);

    /// Mevcut kilitlenme durumu
    const LockonInfo& current_info() const { return info_; }

    /// Belirli bir hedef kara listede mi?
    bool is_blacklisted(int target_id) const;

    /// Kara listeye ekle
    void add_to_blacklist(int target_id);

    /// Tüm sayaçları sıfırla
    void reset();

    /// Son başarılı kilitlenme hedef ID'si
    int last_locked_target() const { return last_locked_id_; }

    /// Tolerans sabiti (public, MissionController erişimi için)
    static constexpr int MAX_TOLERANCE = 1;   // 1 saniyelik esneme

private:
    /// Hedef vuruş alanı içinde mi?
    bool is_in_strike_zone(const BoundingBox& bbox, int frame_w, int frame_h) const;

    /// Hedef minimum boyut şartını sağlıyor mu?
    bool meets_size_requirement(const BoundingBox& bbox, int frame_w, int frame_h) const;

    /// Vuruş alanı sınırlarını hesapla
    struct StrikeZone {
        int x1, y1, x2, y2;
    };
    StrikeZone compute_strike_zone(int frame_w, int frame_h) const;

    VisionConfig config_;
    LockonInfo info_;

    // Kilitlenme zamanlama
    std::chrono::steady_clock::time_point lock_start_time_;
    int continuous_lock_frames_ = 0;
    int total_frames_in_attempt_ = 0;
    int missed_frames_in_attempt_ = 0;

    // Kara liste (vurulan hedefler)
    std::unordered_set<int> blacklist_;
    int last_locked_id_ = -1;

    // Tolerans
    int tolerance_counter_ = 0;
};

}  // namespace siha
