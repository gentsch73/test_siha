/**
 * @file telemetry_manager.hpp
 * @brief Telemetri Yöneticisi — Tüm telemetri verilerinin merkezi toplanma noktası
 *
 * Otopilottan gelen veriler, sunucudan gelen rakip verileri
 * ve iç durum bilgilerini birleştirir.
 * ÖNEMLİ: Telemetri interpolasyonu YASAK — otopilottan gelen veri olduğu gibi iletilir.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <mutex>
#include <vector>

namespace siha {

class TelemetryManager {
public:
    TelemetryManager();

    /// Kendi telemetrimizi güncelle (otopilottan ham veri)
    void update_own(const Telemetry& telem);

    /// Kendi telemetrimizi oku
    Telemetry own() const;

    /// Rakip telemetrilerini güncelle (sunucudan)
    void update_competitors(const std::vector<CompetitorUAV>& comps);

    /// Rakip telemetrilerini oku
    std::vector<CompetitorUAV> competitors() const;

    /// HSS bölgelerini güncelle
    void update_nfz(const std::vector<NoFlyZone>& zones);

    /// HSS bölgelerini oku
    std::vector<NoFlyZone> nfz_zones() const;

    /// Sunucu saatini güncelle
    void update_server_time(const ServerTime& st);

    /// Sunucu saatini oku
    ServerTime server_time() const;

    /// Arena sınırını ayarla
    void set_boundary(const ArenaBoundary& boundary);

    /// Arena sınırını oku
    ArenaBoundary boundary() const;

private:
    mutable std::mutex mutex_;
    Telemetry own_;
    std::vector<CompetitorUAV> competitors_;
    std::vector<NoFlyZone> nfz_zones_;
    ServerTime server_time_;
    ArenaBoundary boundary_;
};

}  // namespace siha
