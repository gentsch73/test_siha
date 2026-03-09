/**
 * @file search_pattern.hpp
 * @brief Arama Paterni Modülü — Hedef arama uçuş desenleri
 *
 * Yarışma alanını tarayarak rakip İHA aramak için
 * çeşitli uçuş desenleri üretir.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <vector>

namespace siha {

enum class SearchType : uint8_t {
    EXPANDING_SQUARE,  // Genişleyen kare
    LAWNMOWER,         // Çim biçme (paralel çizgiler)
    SPIRAL,            // Spiral
    RANDOM_WAYPOINT    // Rastgele waypoint
};

class SearchPattern {
public:
    explicit SearchPattern(const ArenaBoundary& boundary);

    /// Arama desenini ayarla
    void set_pattern(SearchType type);

    /// Sonraki waypoint'i al
    GeoPoint next_waypoint();

    /// Mevcut waypoint
    GeoPoint current_waypoint() const;

    /// Waypoint'e ulaşıldı mı? (mesafe kontrolü)
    bool waypoint_reached(const GeoPoint& current_pos, double threshold_m = 30.0) const;

    /// Yeni başlangıç noktası ayarla
    void reset(const GeoPoint& start);

    /// Tüm waypoint'ler tamamlandı mı?
    bool is_complete() const;

    /// Kalan waypoint sayısı
    int remaining() const;

private:
    void generate_expanding_square(const GeoPoint& center);
    void generate_lawnmower();
    void generate_spiral(const GeoPoint& center);
    void generate_random_waypoints(int count = 20);

    ArenaBoundary boundary_;
    SearchType type_ = SearchType::EXPANDING_SQUARE;
    std::vector<GeoPoint> waypoints_;
    int current_idx_ = 0;
};

}  // namespace siha
