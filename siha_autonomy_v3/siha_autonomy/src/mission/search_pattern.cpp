/**
 * @file search_pattern.cpp
 */
#include "siha_autonomy/mission/search_pattern.hpp"
#include <cmath>
#include <random>
#include <algorithm>

namespace siha {

SearchPattern::SearchPattern(const ArenaBoundary& boundary)
    : boundary_(boundary)
{
}

void SearchPattern::set_pattern(SearchType type) {
    type_ = type;
    current_idx_ = 0;
    waypoints_.clear();

    GeoPoint center;
    if (!boundary_.vertices.empty()) {
        double sum_lat = 0, sum_lon = 0;
        for (const auto& v : boundary_.vertices) {
            sum_lat += v.latitude;
            sum_lon += v.longitude;
        }
        center.latitude = sum_lat / boundary_.vertices.size();
        center.longitude = sum_lon / boundary_.vertices.size();
        center.altitude = 60.0;  // varsayılan cruise altitude
    }

    switch (type) {
        case SearchType::EXPANDING_SQUARE:
            generate_expanding_square(center);
            break;
        case SearchType::LAWNMOWER:
            generate_lawnmower();
            break;
        case SearchType::SPIRAL:
            generate_spiral(center);
            break;
        case SearchType::RANDOM_WAYPOINT:
            generate_random_waypoints();
            break;
    }
}

GeoPoint SearchPattern::next_waypoint() {
    if (waypoints_.empty()) return {};
    current_idx_ = (current_idx_ + 1) % static_cast<int>(waypoints_.size());
    return waypoints_[current_idx_];
}

GeoPoint SearchPattern::current_waypoint() const {
    if (waypoints_.empty() || current_idx_ >= static_cast<int>(waypoints_.size()))
        return {};
    return waypoints_[current_idx_];
}

bool SearchPattern::waypoint_reached(const GeoPoint& pos, double threshold_m) const {
    if (waypoints_.empty()) return true;
    auto& wp = waypoints_[current_idx_];

    double dy = (wp.latitude - pos.latitude) * 111320.0;
    double dx = (wp.longitude - pos.longitude) *
                111320.0 * std::cos(pos.latitude * M_PI / 180.0);
    double dist = std::sqrt(dx*dx + dy*dy);

    return dist < threshold_m;
}

void SearchPattern::reset(const GeoPoint& start) {
    current_idx_ = 0;
    // Başlangıca en yakın waypoint'i bul
    if (waypoints_.empty()) return;
    double min_dist = 1e9;
    for (int i = 0; i < static_cast<int>(waypoints_.size()); i++) {
        double dy = (waypoints_[i].latitude - start.latitude) * 111320.0;
        double dx = (waypoints_[i].longitude - start.longitude) *
                    111320.0 * std::cos(start.latitude * M_PI / 180.0);
        double dist = dx*dx + dy*dy;
        if (dist < min_dist) {
            min_dist = dist;
            current_idx_ = i;
        }
    }
}

bool SearchPattern::is_complete() const {
    return current_idx_ >= static_cast<int>(waypoints_.size()) - 1;
}

int SearchPattern::remaining() const {
    return std::max(0, static_cast<int>(waypoints_.size()) - current_idx_ - 1);
}

void SearchPattern::generate_expanding_square(const GeoPoint& center) {
    // Genişleyen kare: merkez → kuzey → doğu → güney → batı → daha büyük
    double step_m = 100.0;  // her adımda 100m genişle
    double dlat_per_m = 1.0 / 111320.0;
    double dlon_per_m = 1.0 / (111320.0 * std::cos(center.latitude * M_PI / 180.0));

    waypoints_.push_back(center);

    for (int ring = 1; ring <= 5; ring++) {
        double offset = step_m * ring;
        GeoPoint p;
        p.altitude = center.altitude;

        // Kuzey
        p.latitude = center.latitude + offset * dlat_per_m;
        p.longitude = center.longitude;
        waypoints_.push_back(p);

        // Kuzeydoğu → Doğu
        p.latitude = center.latitude + offset * dlat_per_m;
        p.longitude = center.longitude + offset * dlon_per_m;
        waypoints_.push_back(p);

        // Güneydoğu
        p.latitude = center.latitude - offset * dlat_per_m;
        p.longitude = center.longitude + offset * dlon_per_m;
        waypoints_.push_back(p);

        // Güneybatı
        p.latitude = center.latitude - offset * dlat_per_m;
        p.longitude = center.longitude - offset * dlon_per_m;
        waypoints_.push_back(p);

        // Kuzeybatı → kapanış
        p.latitude = center.latitude + offset * dlat_per_m;
        p.longitude = center.longitude - offset * dlon_per_m;
        waypoints_.push_back(p);
    }
}

void SearchPattern::generate_lawnmower() {
    if (boundary_.vertices.empty()) return;

    double min_lat = 1e9, max_lat = -1e9;
    double min_lon = 1e9, max_lon = -1e9;
    for (const auto& v : boundary_.vertices) {
        min_lat = std::min(min_lat, v.latitude);
        max_lat = std::max(max_lat, v.latitude);
        min_lon = std::min(min_lon, v.longitude);
        max_lon = std::max(max_lon, v.longitude);
    }

    double spacing_m = 150.0;
    double dlat = spacing_m / 111320.0;
    bool left_to_right = true;

    for (double lat = min_lat; lat <= max_lat; lat += dlat) {
        GeoPoint p;
        p.altitude = 60.0;
        p.latitude = lat;

        if (left_to_right) {
            p.longitude = min_lon; waypoints_.push_back(p);
            p.longitude = max_lon; waypoints_.push_back(p);
        } else {
            p.longitude = max_lon; waypoints_.push_back(p);
            p.longitude = min_lon; waypoints_.push_back(p);
        }
        left_to_right = !left_to_right;
    }
}

void SearchPattern::generate_spiral(const GeoPoint& center) {
    double dlat_per_m = 1.0 / 111320.0;
    double dlon_per_m = 1.0 / (111320.0 * std::cos(center.latitude * M_PI / 180.0));

    for (int i = 0; i < 60; i++) {
        double angle = i * 30.0 * M_PI / 180.0;  // her 30°
        double radius = 50.0 + i * 20.0;  // spiral genişliyor

        GeoPoint p;
        p.latitude = center.latitude + radius * std::cos(angle) * dlat_per_m;
        p.longitude = center.longitude + radius * std::sin(angle) * dlon_per_m;
        p.altitude = center.altitude;
        waypoints_.push_back(p);
    }
}

void SearchPattern::generate_random_waypoints(int count) {
    if (boundary_.vertices.empty()) return;

    std::random_device rd;
    std::mt19937 gen(rd());

    double min_lat = 1e9, max_lat = -1e9;
    double min_lon = 1e9, max_lon = -1e9;
    for (const auto& v : boundary_.vertices) {
        min_lat = std::min(min_lat, v.latitude);
        max_lat = std::max(max_lat, v.latitude);
        min_lon = std::min(min_lon, v.longitude);
        max_lon = std::max(max_lon, v.longitude);
    }

    std::uniform_real_distribution<double> lat_dist(min_lat, max_lat);
    std::uniform_real_distribution<double> lon_dist(min_lon, max_lon);

    for (int i = 0; i < count; i++) {
        GeoPoint p;
        p.latitude = lat_dist(gen);
        p.longitude = lon_dist(gen);
        p.altitude = 60.0;
        waypoints_.push_back(p);
    }
}

}  // namespace siha
