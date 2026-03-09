/**
 * @file safety_monitor.hpp
 * @brief Güvenlik İzleyicisi
 *
 * Her tick'te kontrol edilen güvenlik koşulları:
 *   - Yarışma sınırı aşımı
 *   - İrtifa limiti aşımı
 *   - Haberleşme kopması (>10 sn)
 *   - Batarya düşüklüğü
 *   - Hava savunma bölgesine giriş
 *   - Çarpışma riski (rakip yakınlığı)
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <vector>
#include <chrono>

namespace siha {

/// Güvenlik alarmı
struct SafetyAlert {
    enum class Type : uint8_t {
        NONE,
        BOUNDARY_VIOLATION,   // Sınır aşımı
        ALTITUDE_VIOLATION,   // İrtifa aşımı
        SIGNAL_LOST,          // Haberleşme kopması
        LOW_BATTERY,          // Düşük batarya
        CRITICAL_BATTERY,     // Kritik batarya
        NFZ_INTRUSION,        // Yasaklı bölgeye giriş
        COLLISION_RISK,       // Çarpışma riski
        GPS_LOST              // GPS kaybı
    };

    Type   type     = Type::NONE;
    double severity = 0.0;       // 0.0 - 1.0
    std::string message;

    /// Zorunlu müdahale gerekiyor mu?
    bool requires_immediate_action() const {
        return severity > 0.8;
    }
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(const SafetyConfig& cfg);

    /**
     * @brief Her tick'te çağrılır. Tüm güvenlik kontrollerini yapar.
     * @return En yüksek öncelikli alarm (NONE ise sorun yok)
     */
    SafetyAlert check(
        const Telemetry& own,
        const ArenaBoundary& boundary,
        const std::vector<CompetitorUAV>& competitors,
        const std::vector<NoFlyZone>& nfz_zones,
        bool server_connected);

    /// Yarışma sınırı içinde mi?
    bool is_in_boundary(const GeoPoint& pos, const ArenaBoundary& boundary) const;

    /// Yasaklı bölgede mi?
    bool is_in_nfz(const GeoPoint& pos, const std::vector<NoFlyZone>& zones) const;

    /// En yakın sınır mesafesi (metre)
    double distance_to_boundary(const GeoPoint& pos, const ArenaBoundary& boundary) const;

    /// Haberleşme koptu mu?
    bool is_signal_lost() const { return signal_lost_; }

    /// Güvenli dönüş noktası (sınır ihlalinde)
    GeoPoint safe_return_point(const GeoPoint& current,
                                const ArenaBoundary& boundary) const;

private:
    SafetyConfig config_;
    bool signal_lost_ = false;
    std::chrono::steady_clock::time_point last_server_contact_;

    /// Point-in-polygon testi
    bool point_in_polygon(double lat, double lon,
                           const std::vector<GeoPoint>& poly) const;
};

}  // namespace siha
