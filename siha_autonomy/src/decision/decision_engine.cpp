#include "siha_autonomy/decision/decision_engine.hpp"
#include <algorithm>
#include <cmath>

namespace siha {

DecisionEngine::DecisionEngine(const DecisionConfig& cfg) : cfg_(cfg) {}

DecisionResult DecisionEngine::evaluate(
    const Telemetry& own,
    const std::vector<CompetitorUAV>& competitors,
    const std::vector<NoFlyZone>& nfz_zones,
    const std::unordered_set<int>& blacklist)
{
    DecisionResult result;

    // ── 1. Tehdit analizi (öncelik!) ──
    analyze_threats(own, competitors);
    result.total_threats = static_cast<int>(threats_.size());

    // En ciddi tehdit
    ThreatInfo worst;
    for (auto& t : threats_) {
        if (t.threat_level > worst.threat_level) worst = t;
    }

    if (worst.threat_level > 0.7) {
        // Ciddi tehdit → KAÇIN
        result.action = DecisionResult::Action::EVADE;
        result.worst_threat = worst;
        // Tehdidin tam tersine dön
        result.evade_heading = std::fmod(worst.bearing_deg + 180.0, 360.0);
        return result;
    }

    // ── 2. Hedef seçimi ──
    analyze_candidates(own, competitors, blacklist);
    result.total_candidates = static_cast<int>(candidates_.size());

    if (candidates_.empty()) {
        result.action = DecisionResult::Action::SEARCH;
        return result;
    }

    // En iyi adayı seç
    auto best_it = std::max_element(candidates_.begin(), candidates_.end(),
        [](const TargetCandidate& a, const TargetCandidate& b) { return a.score < b.score; });

    result.action = DecisionResult::Action::PURSUE;
    result.best_target = *best_it;

    // Hedef GPS konumu bul
    for (auto& comp : competitors) {
        if (comp.team_id == best_it->team_id) {
            result.target_position = comp.position;
            break;
        }
    }

    return result;
}

void DecisionEngine::analyze_threats(const Telemetry& own,
                                      const std::vector<CompetitorUAV>& comps)
{
    threats_.clear();

    for (auto& comp : comps) {
        double dy = (comp.position.latitude - own.position.latitude) * 111320.0;
        double dx = (comp.position.longitude - own.position.longitude) *
                    111320.0 * std::cos(own.position.latitude * M_PI / 180.0);
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist > cfg_.evade_range_m * 2) continue;  // çok uzak, tehdit değil

        // Rakibin bize göre yönü
        double bearing = std::fmod(std::atan2(dx, dy) * 180.0 / M_PI + 360.0, 360.0);

        // Rakip bizim arkamızdan mı geliyor?
        double angle_from_tail = std::abs(bearing - std::fmod(own.heading + 180.0, 360.0));
        if (angle_from_tail > 180.0) angle_from_tail = 360.0 - angle_from_tail;
        bool is_behind = (angle_from_tail < cfg_.behind_angle_deg / 2.0);

        // Yaklaşma hızı tahmini (heading'e göre)
        double comp_heading_rad = comp.heading * M_PI / 180.0;
        double comp_vx = comp.speed * std::sin(comp_heading_rad);
        double comp_vy = comp.speed * std::cos(comp_heading_rad);
        double own_heading_rad = own.heading * M_PI / 180.0;
        double own_vx = own.speed * std::sin(own_heading_rad);
        double own_vy = own.speed * std::cos(own_heading_rad);

        // Relative velocity along LOS
        double los_x = dx / std::max(dist, 1.0);
        double los_y = dy / std::max(dist, 1.0);
        double closure = -((comp_vx - own_vx) * los_x + (comp_vy - own_vy) * los_y);

        // Tehdit seviyesi
        double threat = 0.0;
        if (dist < cfg_.evade_range_m && closure > cfg_.evade_closure_ms && is_behind) {
            threat = (1.0 - dist / cfg_.evade_range_m) * 0.6 +
                     std::min(closure / 20.0, 1.0) * 0.4;
        }

        if (threat > 0.1) {
            ThreatInfo ti;
            ti.team_id = comp.team_id;
            ti.distance_m = dist;
            ti.bearing_deg = bearing;
            ti.closure_rate = closure;
            ti.threat_level = std::min(threat, 1.0);
            ti.is_behind = is_behind;
            threats_.push_back(ti);
        }
    }
}

void DecisionEngine::analyze_candidates(const Telemetry& own,
                                          const std::vector<CompetitorUAV>& comps,
                                          const std::unordered_set<int>& blacklist)
{
    candidates_.clear();

    for (auto& comp : comps) {
        if (blacklist.count(comp.team_id)) continue;

        double dy = (comp.position.latitude - own.position.latitude) * 111320.0;
        double dx = (comp.position.longitude - own.position.longitude) *
                    111320.0 * std::cos(own.position.latitude * M_PI / 180.0);
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist > cfg_.max_score_distance) continue;  // çok uzak
        if (dist < 5.0) continue;  // çok yakın (muhtemelen biz)

        // Hedefin bearing'i
        double bearing = std::fmod(std::atan2(dx, dy) * 180.0 / M_PI + 360.0, 360.0);

        // Burnumuza göre açı farkı
        double angle_off = bearing - own.heading;
        if (angle_off > 180.0) angle_off -= 360.0;
        if (angle_off < -180.0) angle_off += 360.0;

        TargetCandidate tc;
        tc.team_id = comp.team_id;
        tc.distance_m = dist;
        tc.bearing_deg = bearing;
        tc.angle_off_deg = angle_off;
        tc.maneuver_cost = compute_maneuver_cost(angle_off);
        tc.eta_s = dist / std::max(own.speed, 10.0);
        tc.is_blacklisted = false;
        tc.score = score_candidate(tc);
        candidates_.push_back(tc);
    }

    // Skora göre sırala (yüksek ilk)
    std::sort(candidates_.begin(), candidates_.end(),
        [](const TargetCandidate& a, const TargetCandidate& b) { return a.score > b.score; });
}

double DecisionEngine::score_candidate(const TargetCandidate& c) const {
    // Mesafe skoru: yakın = iyi (0-1)
    double dist_score = 1.0 - std::min(c.distance_m / cfg_.max_score_distance, 1.0);

    // Açı skoru: önümüzde = iyi (0-1)
    double angle_score = 1.0 - std::min(std::abs(c.angle_off_deg) / 180.0, 1.0);

    // Manevra skoru: düşük maliyet = iyi (0-1)
    double maneuver_score = 1.0 - c.maneuver_cost;

    // ETA skoru: kısa = iyi (0-1)
    double eta_score = 1.0 - std::min(c.eta_s / 30.0, 1.0);

    return cfg_.weight_distance  * dist_score +
           cfg_.weight_angle     * angle_score +
           cfg_.weight_maneuver  * maneuver_score +
           cfg_.weight_eta       * eta_score;
}

double DecisionEngine::compute_maneuver_cost(double angle_off_deg) const {
    // Sabit kanat dönüş maliyeti:
    // 0° (tam önde) → 0.0 (bedava)
    // 90° (yanda)   → 0.5 (orta)
    // 180° (arkada)  → 1.0 (pahalı — tam U-dönüşü)
    double abs_angle = std::abs(angle_off_deg);

    // Non-linear: küçük açılar ucuz, büyük açılar çok pahalı
    return std::pow(abs_angle / 180.0, 1.5);
}

}  // namespace siha
