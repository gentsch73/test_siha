/**
 * @file kamikaze_module.hpp
 * @brief Kamikaze Görevi — QR Kod Dalış Planlaması
 *
 * 100m+ irtifadan dalışa geçip yerdeki 2m×2m QR kodu
 * okuma ve sunucuya iletme görevini yönetir.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include "siha_autonomy/mission/qr_detector.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <memory>

namespace siha {

/// Kamikaze görev aşaması
enum class KamikazePhase : uint8_t {
    IDLE,
    CLIMBING,        // ≥100m irtifaya tırmanma
    ALIGNING,        // QR hedefine hizalanma
    DIVING,          // -45° dalış
    READING_QR,      // QR okuma (dalış sırasında)
    PULLING_UP,      // 55m'de dalıştan çıkış
    COMPLETED,       // Görev tamamlandı
    ABORTED          // İptal edildi
};

class KamikazeModule {
public:
    explicit KamikazeModule(const KamikazeConfig& cfg,
                             const VisionConfig& vcfg);

    /// Görevi başlat
    void start(const GeoPoint& qr_position);

    /// Her tick'te çağrılır
    KamikazePhase update(const Telemetry& own, const cv::Mat& frame);

    /// Mevcut faz
    KamikazePhase phase() const { return phase_; }

    /// QR kodu okundu mu?
    bool qr_read_success() const { return qr_success_; }

    /// Okunan QR içeriği
    std::string qr_content() const { return qr_content_; }

    /// Hedef dalış noktası (GPS)
    GeoPoint dive_target() const { return target_pos_; }

    /// Sıfırla
    void reset();

private:
    KamikazeConfig config_;
    KamikazePhase phase_ = KamikazePhase::IDLE;
    std::unique_ptr<QRDetector> qr_detector_;

    GeoPoint target_pos_;
    bool qr_success_ = false;
    std::string qr_content_;

    std::chrono::steady_clock::time_point dive_start_time_;
};

}  // namespace siha
