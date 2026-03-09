/**
 * @file decision_engine.hpp
 * @brief Karar Motoru — Akıllı hedef seçimi ve kaçınma kararı
 *
 * Sunucudan gelen 15 rakip telemetrisini analiz eder:
 *   1) Her rakip için "kilitlenme skoru" hesaplar
 *   2) Arkadan yaklaşan tehditleri tespit eder
 *   3) Manevra maliyetini hesaplar (sabit kanat dönüş yarıçapı)
 *   4) En iyi hedefi seçer veya kaçınma komutu verir
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <vector>
#include <unordered_set>
#include <cmath>

namespace siha {

/// Aday hedef analizi
struct TargetCandidate {
    int    team_id       = 0;
    double distance_m    = 0.0;    // mesafe
    double bearing_deg   = 0.0;    // hedef yönü (mutlak)
    double angle_off_deg = 0.0;    // burnumuza göre açı farkı (0=tam önümüzde)
    double maneuver_cost = 0.0;    // dönüş maliyeti (0-1, düşük=kolay)
    double eta_s         = 0.0;    // tahmini varış süresi (saniye)
    double score         = 0.0;    // toplam skor (yüksek=iyi)
    bool   is_blacklisted = false;
};

/// Tehdit analizi
struct ThreatInfo {
    int    team_id       = 0;
    double distance_m    = 0.0;
    double bearing_deg   = 0.0;
    double closure_rate  = 0.0;    // yaklaşma hızı (pozitif=yaklaşıyor)
    double threat_level  = 0.0;    // 0-1
    bool   is_behind     = false;  // arkamızdan mı geliyor
};

/// Karar çıktısı
struct DecisionResult {
    enum class Action { SEARCH, PURSUE, EVADE, LOITER };
    Action action = Action::SEARCH;

    // PURSUE durumunda
    TargetCandidate best_target;
    GeoPoint target_position;

    // EVADE durumunda
    ThreatInfo worst_threat;
    double evade_heading = 0.0;

    // Debug
    int total_candidates = 0;
    int total_threats = 0;
};

struct DecisionConfig {
    double weight_distance  = 0.35;
    double weight_angle     = 0.30;
    double weight_maneuver  = 0.25;
    double weight_eta       = 0.10;

    double evade_range_m    = 60.0;    // bu mesafede kaçınma tetiklenir
    double evade_closure_ms = 5.0;     // yaklaşma hızı eşiği (m/s)
    double behind_angle_deg = 120.0;   // "arkamızda" sayılan açı

    double min_turn_radius_m = 30.0;   // sabit kanat minimum dönüş yarıçapı
    double max_score_distance = 500.0; // bu mesafeden uzaklar 0 puan alır
};

class DecisionEngine {
public:
    explicit DecisionEngine(const DecisionConfig& cfg = {});

    /// Her tick'te çağrılır — karar üretir
    DecisionResult evaluate(
        const Telemetry& own,
        const std::vector<CompetitorUAV>& competitors,
        const std::vector<NoFlyZone>& nfz_zones,
        const std::unordered_set<int>& blacklist);

    /// Tüm adayların listesi (debug)
    const std::vector<TargetCandidate>& candidates() const { return candidates_; }
    const std::vector<ThreatInfo>& threats() const { return threats_; }

private:
    double score_candidate(const TargetCandidate& c) const;
    double compute_maneuver_cost(double angle_off_deg) const;
    void analyze_threats(const Telemetry& own, const std::vector<CompetitorUAV>& comps);
    void analyze_candidates(const Telemetry& own, const std::vector<CompetitorUAV>& comps,
                             const std::unordered_set<int>& blacklist);

    DecisionConfig cfg_;
    std::vector<TargetCandidate> candidates_;
    std::vector<ThreatInfo> threats_;
};

}  // namespace siha
