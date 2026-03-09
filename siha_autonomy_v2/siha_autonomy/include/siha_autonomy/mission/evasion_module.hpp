/**
 * @file evasion_module.hpp
 * @brief Kaçınma Modülü — Rakip İHA ve yasaklı bölge kaçınma
 *
 * Sunucudan gelen rakip telemetri + hava savunma bölgelerine
 * göre kaçınma manevrası üretir.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <vector>

namespace siha {

/// Tehdit değerlendirmesi
struct ThreatAssessment {
    int    source_id      = -1;
    double bearing_deg    = 0.0;   // tehdit yönü
    double distance_m     = 0.0;   // mesafe
    double closure_rate   = 0.0;   // yaklaşma hızı (m/s)
    double threat_level   = 0.0;   // 0.0 - 1.0
    bool   is_nfz         = false; // hava savunma bölgesi mi
};

class EvasionModule {
public:
    explicit EvasionModule(const SafetyConfig& cfg);

    /**
     * @brief Tehdit analizi yap ve kaçınma komutu üret.
     * @param own Kendi pozisyonumuz
     * @param competitors Rakip İHA'lar
     * @param nfz_list Yasaklı bölgeler
     * @return En acil tehdit değerlendirmesi
     */
    ThreatAssessment evaluate(
        const Telemetry& own,
        const std::vector<CompetitorUAV>& competitors,
        const std::vector<NoFlyZone>& nfz_list);

    /// Kaçınma manevrası gerekli mi?
    bool evasion_needed() const { return evasion_needed_; }

    /// Kaçınma yönü (derece, kendi heading'imize göre)
    double evasion_heading() const { return evasion_heading_; }

    /// Kaçınma irtifası
    double evasion_altitude() const { return evasion_alt_; }

    /// Sıfırla
    void reset();

private:
    double compute_threat_level(const Telemetry& own,
                                 const CompetitorUAV& comp) const;

    SafetyConfig config_;
    bool evasion_needed_ = false;
    double evasion_heading_ = 0.0;
    double evasion_alt_ = 0.0;
    ThreatAssessment worst_threat_;
};

}  // namespace siha
