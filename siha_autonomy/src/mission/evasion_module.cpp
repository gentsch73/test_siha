/**
 * @file evasion_module.cpp
 */
#include "siha_autonomy/mission/evasion_module.hpp"
#include <cmath>
#include <algorithm>

namespace siha {

EvasionModule::EvasionModule(const SafetyConfig& cfg) : config_(cfg) {}

ThreatAssessment EvasionModule::evaluate(
    const Telemetry& own,
    const std::vector<CompetitorUAV>& competitors,
    const std::vector<NoFlyZone>& nfz_list)
{
    worst_threat_ = {};
    evasion_needed_ = false;

    // Rakip tehdit analizi
    for (const auto& comp : competitors) {
        double threat = compute_threat_level(own, comp);
        if (threat > worst_threat_.threat_level) {
            double dy = (comp.position.latitude - own.position.latitude) * 111320.0;
            double dx = (comp.position.longitude - own.position.longitude) *
                        111320.0 * std::cos(own.position.latitude * M_PI / 180.0);

            worst_threat_.source_id = comp.team_id;
            worst_threat_.bearing_deg = std::fmod(
                std::atan2(dx, dy) * 180.0 / M_PI + 360.0, 360.0);
            worst_threat_.distance_m = std::sqrt(dx*dx + dy*dy);
            worst_threat_.threat_level = threat;
            worst_threat_.is_nfz = false;
        }
    }

    // HSS tehdit analizi
    for (const auto& nfz : nfz_list) {
        double dy = (nfz.latitude - own.position.latitude) * 111320.0;
        double dx = (nfz.longitude - own.position.longitude) *
                    111320.0 * std::cos(own.position.latitude * M_PI / 180.0);
        double dist = std::sqrt(dx*dx + dy*dy);
        double threat = 0.0;

        if (dist < nfz.radius_m) {
            threat = 1.0;  // İçindeyiz!
        } else if (dist < nfz.radius_m + config_.boundary_margin_m) {
            threat = 1.0 - (dist - nfz.radius_m) / config_.boundary_margin_m;
        }

        if (threat > worst_threat_.threat_level) {
            worst_threat_.bearing_deg = std::fmod(
                std::atan2(dx, dy) * 180.0 / M_PI + 360.0, 360.0);
            worst_threat_.distance_m = dist;
            worst_threat_.threat_level = threat;
            worst_threat_.is_nfz = true;
        }
    }

    // Kaçınma kararı
    if (worst_threat_.threat_level > 0.5) {
        evasion_needed_ = true;
        // Tehdidin tersi yöne
        evasion_heading_ = std::fmod(worst_threat_.bearing_deg + 180.0, 360.0);
        // İrtifa değişimi
        evasion_alt_ = own.position.altitude + (worst_threat_.is_nfz ? 0.0 : 20.0);
    }

    return worst_threat_;
}

double EvasionModule::compute_threat_level(const Telemetry& own,
                                             const CompetitorUAV& comp) const
{
    double dy = (comp.position.latitude - own.position.latitude) * 111320.0;
    double dx = (comp.position.longitude - own.position.longitude) *
                111320.0 * std::cos(own.position.latitude * M_PI / 180.0);
    double dz = comp.position.altitude - own.position.altitude;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist > 200.0) return 0.0;
    if (dist < config_.collision_radius_m) return 1.0;

    // Mesafe tabanlı tehdit (0-1 arası)
    return 1.0 - (dist / 200.0);
}

void EvasionModule::reset() {
    evasion_needed_ = false;
    worst_threat_ = {};
}

}  // namespace siha
