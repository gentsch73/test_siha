/**
 * @file safety_monitor.cpp
 * @brief Güvenlik İzleyicisi implementasyonu
 */

#include "siha_autonomy/safety/safety_monitor.hpp"
#include <cmath>
#include <algorithm>

namespace siha {

SafetyMonitor::SafetyMonitor(const SafetyConfig& cfg)
    : config_(cfg)
{
    last_server_contact_ = std::chrono::steady_clock::now();
}

SafetyAlert SafetyMonitor::check(
    const Telemetry& own,
    const ArenaBoundary& boundary,
    const std::vector<CompetitorUAV>& competitors,
    const std::vector<NoFlyZone>& nfz_zones,
    bool server_connected)
{
    SafetyAlert worst;
    worst.type = SafetyAlert::Type::NONE;
    worst.severity = 0.0;

    // Sunucu bağlantı takibi
    if (server_connected) {
        last_server_contact_ = std::chrono::steady_clock::now();
        signal_lost_ = false;
    }

    // ── 1. Sınır Kontrolü ──
    if (!boundary.vertices.empty()) {
        if (!is_in_boundary(own.position, boundary)) {
            worst.type = SafetyAlert::Type::BOUNDARY_VIOLATION;
            worst.severity = 1.0;
            worst.message = "SINIR AŞIMI! Yarışma alanı dışındasınız.";
            return worst;
        }

        // Sınıra yaklaşma uyarısı
        double dist = distance_to_boundary(own.position, boundary);
        if (dist < config_.boundary_margin_m) {
            worst.type = SafetyAlert::Type::BOUNDARY_VIOLATION;
            worst.severity = 0.7;
            worst.message = "Sınıra çok yakınsınız: " +
                            std::to_string(static_cast<int>(dist)) + "m";
        }
    }

    // ── 2. İrtifa Kontrolü ──
    if (own.position.altitude > boundary.max_altitude) {
        SafetyAlert alt;
        alt.type = SafetyAlert::Type::ALTITUDE_VIOLATION;
        alt.severity = 0.9;
        alt.message = "İRTİFA LİMİTİ AŞILDI: " +
                      std::to_string(static_cast<int>(own.position.altitude)) + "m";
        if (alt.severity > worst.severity) worst = alt;
    }

    // ── 3. Haberleşme Kopması ──
    auto elapsed = std::chrono::steady_clock::now() - last_server_contact_;
    double elapsed_s = std::chrono::duration<double>(elapsed).count();
    if (elapsed_s > config_.signal_loss_timeout) {
        signal_lost_ = true;
        SafetyAlert sig;
        sig.type = SafetyAlert::Type::SIGNAL_LOST;
        sig.severity = 0.85;
        sig.message = "Haberleşme koptu: " +
                      std::to_string(static_cast<int>(elapsed_s)) + " sn";
        if (sig.severity > worst.severity) worst = sig;
    }

    // ── 4. Batarya ──
    if (own.battery <= static_cast<int>(config_.emergency_battery)) {
        SafetyAlert bat;
        bat.type = SafetyAlert::Type::CRITICAL_BATTERY;
        bat.severity = 1.0;
        bat.message = "KRİTİK BATARYA: %" + std::to_string(own.battery);
        if (bat.severity > worst.severity) worst = bat;
    } else if (own.battery <= static_cast<int>(config_.rtl_battery_pct)) {
        SafetyAlert bat;
        bat.type = SafetyAlert::Type::LOW_BATTERY;
        bat.severity = 0.75;
        bat.message = "Düşük batarya: %" + std::to_string(own.battery);
        if (bat.severity > worst.severity) worst = bat;
    }

    // ── 5. Hava Savunma (No-Fly Zone) ──
    if (is_in_nfz(own.position, nfz_zones)) {
        SafetyAlert nfz;
        nfz.type = SafetyAlert::Type::NFZ_INTRUSION;
        nfz.severity = 0.95;
        nfz.message = "KIRMIZI BÖLGE GİRİŞİ! Kaçınma gerekli.";
        if (nfz.severity > worst.severity) worst = nfz;
    }

    // ── 6. Çarpışma Riski ──
    for (const auto& comp : competitors) {
        double dx = (comp.position.latitude - own.position.latitude) * 111320.0;
        double dy = (comp.position.longitude - own.position.longitude) *
                    111320.0 * std::cos(own.position.latitude * M_PI / 180.0);
        double dz = comp.position.altitude - own.position.altitude;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < config_.collision_radius_m) {
            SafetyAlert col;
            col.type = SafetyAlert::Type::COLLISION_RISK;
            col.severity = 0.9;
            col.message = "Çarpışma riski! Rakip " +
                          std::to_string(comp.team_id) +
                          " - " + std::to_string(static_cast<int>(dist)) + "m";
            if (col.severity > worst.severity) worst = col;
        }
    }

    return worst;
}

bool SafetyMonitor::is_in_boundary(const GeoPoint& pos,
                                     const ArenaBoundary& boundary) const
{
    if (boundary.vertices.empty()) return true;
    return point_in_polygon(pos.latitude, pos.longitude, boundary.vertices);
}

bool SafetyMonitor::is_in_nfz(const GeoPoint& pos,
                                const std::vector<NoFlyZone>& zones) const
{
    for (const auto& nfz : zones) {
        double dx = (pos.latitude - nfz.latitude) * 111320.0;
        double dy = (pos.longitude - nfz.longitude) *
                    111320.0 * std::cos(pos.latitude * M_PI / 180.0);
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < nfz.radius_m) return true;
    }
    return false;
}

double SafetyMonitor::distance_to_boundary(const GeoPoint& pos,
                                             const ArenaBoundary& boundary) const
{
    if (boundary.vertices.empty()) return 99999.0;

    double min_dist = 99999.0;
    int n = static_cast<int>(boundary.vertices.size());

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        const auto& a = boundary.vertices[i];
        const auto& b = boundary.vertices[j];

        // Noktanın kenar segmentine olan minimum mesafesi
        double ax = a.latitude * 111320.0;
        double ay = a.longitude * 111320.0 * std::cos(a.latitude * M_PI / 180.0);
        double bx = b.latitude * 111320.0;
        double by = b.longitude * 111320.0 * std::cos(b.latitude * M_PI / 180.0);
        double px = pos.latitude * 111320.0;
        double py = pos.longitude * 111320.0 * std::cos(pos.latitude * M_PI / 180.0);

        double dx = bx - ax, dy = by - ay;
        double len_sq = dx*dx + dy*dy;
        if (len_sq < 1e-10) continue;

        double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len_sq, 0.0, 1.0);
        double closest_x = ax + t * dx;
        double closest_y = ay + t * dy;

        double dist = std::sqrt(
            (px - closest_x) * (px - closest_x) +
            (py - closest_y) * (py - closest_y));
        min_dist = std::min(min_dist, dist);
    }

    return min_dist;
}

GeoPoint SafetyMonitor::safe_return_point(const GeoPoint& current,
                                            const ArenaBoundary& boundary) const
{
    if (boundary.vertices.empty()) return current;

    // Sınırın merkezine dön
    double sum_lat = 0, sum_lon = 0;
    for (const auto& v : boundary.vertices) {
        sum_lat += v.latitude;
        sum_lon += v.longitude;
    }

    GeoPoint center;
    center.latitude = sum_lat / boundary.vertices.size();
    center.longitude = sum_lon / boundary.vertices.size();
    center.altitude = current.altitude;

    return center;
}

bool SafetyMonitor::point_in_polygon(double lat, double lon,
                                       const std::vector<GeoPoint>& poly) const
{
    bool inside = false;
    int n = static_cast<int>(poly.size());
    int j = n - 1;

    for (int i = 0; i < n; i++) {
        double yi = poly[i].longitude, xi = poly[i].latitude;
        double yj = poly[j].longitude, xj = poly[j].latitude;

        bool intersect = ((yi > lon) != (yj > lon)) &&
                         (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-9) + xi);
        if (intersect) inside = !inside;
        j = i;
    }

    return inside;
}

}  // namespace siha
