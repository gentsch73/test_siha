/**
 * @file decision_engine.cpp
 * @brief Karar Motoru implementasyonu
 *
 * Hedef seçim algoritması:
 *   Skor = w_dist * dist_score + w_angle * angle_score
 *          + w_maneuver * maneuver_score + w_eta * eta_score
 *          - relock_penalty (aynı hedefe tekrar)
 *
 * Kaçınma algoritması:
 *   Arkadan (>120°) yaklaşan ve mesafesi <50m olan rakip tespit edilirse
 *   kaçınma manevrası tetiklenir.
 */

#include "siha_autonomy/decision/decision_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <algorithm>
#include <sstream>

namespace siha {

// ─────────────────────────────────────────────────
//  Geometri Yardımcıları
// ─────────────────────────────────────────────────

double DecisionEngine::haversine_m(double lat1, double lon1,
                                    double lat2, double lon2)
{
    constexpr double R = 6371000.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlam = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dphi / 2) * std::sin(dphi / 2)
             + std::cos(phi1) * std::cos(phi2)
             * std::sin(dlam / 2) * std::sin(dlam / 2);
    return 2.0 * R * std::asin(std::sqrt(a));
}

double DecisionEngine::bearing(double lat1, double lon1,
                                double lat2, double lon2)
{
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double x = std::sin(dlon) * std::cos(phi2);
    double y = std::cos(phi1) * std::sin(phi2)
             - std::sin(phi1) * std::cos(phi2) * std::cos(dlon);
    double brng = std::atan2(x, y) * 180.0 / M_PI;
    return std::fmod(brng + 360.0, 360.0);
}

double DecisionEngine::angle_diff(double a, double b)
{
    double diff = std::fmod(a - b + 180.0, 360.0) - 180.0;
    return diff;
}

// ─────────────────────────────────────────────────
//  DecisionEngine
// ─────────────────────────────────────────────────

DecisionEngine::DecisionEngine(const DecisionConfig& cfg)
    : config_(cfg)
{
}

void DecisionEngine::update_own_telemetry(const Telemetry& own)
{
    std::lock_guard<std::mutex> lock(mutex_);
    own_telem_ = own;
}

void DecisionEngine::update_competitors(const std::vector<CompetitorUAV>& competitors)
{
    std::lock_guard<std::mutex> lock(mutex_);
    competitors_ = competitors;
}

void DecisionEngine::on_lock_complete(int team_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    blacklist_.insert(team_id);
    evading_ = false;
}

void DecisionEngine::on_evade_complete()
{
    std::lock_guard<std::mutex> lock(mutex_);
    evading_ = false;
}

double DecisionEngine::score_candidate(const CompetitorUAV& rival) const
{
    // Kendi konumumuz (mutex dışarıda tutulur — çağıran kilitliyor)
    double own_lat = own_telem_.position.latitude;
    double own_lon = own_telem_.position.longitude;
    double own_alt = own_telem_.position.altitude;
    double own_hdg = own_telem_.heading;

    if (own_lat == 0.0 && own_lon == 0.0) return 0.0;

    double h_dist = haversine_m(own_lat, own_lon,
                                 rival.position.latitude,
                                 rival.position.longitude);
    double v_dist = std::abs(rival.position.altitude - own_alt);
    double dist3d = std::sqrt(h_dist * h_dist + v_dist * v_dist);

    // Çok uzaktaki hedefleri ele alma
    if (dist3d > config_.max_range_m) return -1000.0;

    // 1) Mesafe skoru: yakın = yüksek (0-40)
    double dist_norm  = std::min(dist3d / config_.max_range_m, 1.0);
    double dist_score = (1.0 - dist_norm) * 40.0;

    // 2) Açı skoru: mevcut heading'e yakın hedef daha kolay ulaşılır (-20 ... +30)
    double brng        = bearing(own_lat, own_lon,
                                  rival.position.latitude, rival.position.longitude);
    double hdg_diff    = angle_diff(brng, own_hdg);
    // Önde (hdg_diff ~ 0) → +30, arkada (hdg_diff ~ ±180) → -20
    double angle_score = 30.0 - (std::abs(hdg_diff) / 180.0) * 50.0;

    // 3) Manevra maliyeti: geniş dönüş pahalı (-20 ... 0)
    double bank_angle    = std::abs(hdg_diff);
    double maneuver_score = -20.0 * std::min(bank_angle / 90.0, 1.0);

    // 4) ETA skoru: hızlı erişim tercih edilir (0-10)
    double own_spd = std::max(own_telem_.speed, 1.0);
    double eta_s   = dist3d / own_spd;
    double eta_score = std::max(0.0, 10.0 - eta_s / 30.0);

    // Toplam skor
    double score = config_.weight_distance * dist_score
                 + config_.weight_angle    * angle_score
                 + config_.weight_maneuver * maneuver_score
                 + config_.weight_eta      * eta_score;

    // Kara listedeki hedefe ceza
    if (blacklist_.count(rival.team_id)) {
        score -= config_.relock_penalty;
    }

    return score;
}

bool DecisionEngine::is_threat(const CompetitorUAV& rival) const
{
    double own_lat = own_telem_.position.latitude;
    double own_lon = own_telem_.position.longitude;

    if (own_lat == 0.0 && own_lon == 0.0) return false;

    double dist = haversine_m(own_lat, own_lon,
                               rival.position.latitude, rival.position.longitude);
    if (dist > config_.evade_range_m) return false;

    // Rakip arkadan mı geliyor? (bizim heading'e göre arkası)
    double brng_to_us = bearing(rival.position.latitude, rival.position.longitude,
                                 own_lat, own_lon);
    double approach_angle = std::abs(angle_diff(brng_to_us, rival.heading));
    bool from_behind = approach_angle < (180.0 - config_.evade_angle_min);

    return from_behind && (rival.speed > config_.evade_speed_ms);
}

std::optional<TargetCandidate> DecisionEngine::evaluate()
{
    std::lock_guard<std::mutex> lock(mutex_);

    current_threats_.clear();

    if (competitors_.empty()) {
        current_target_ = std::nullopt;
        return std::nullopt;
    }

    // Tehdit tespiti
    for (const auto& rival : competitors_) {
        if (is_threat(rival)) {
            TargetCandidate tc;
            tc.team_id    = rival.team_id;
            tc.latitude   = rival.position.latitude;
            tc.longitude  = rival.position.longitude;
            tc.altitude   = rival.position.altitude;
            tc.heading    = rival.heading;
            tc.distance_m = haversine_m(
                own_telem_.position.latitude, own_telem_.position.longitude,
                rival.position.latitude, rival.position.longitude);
            tc.is_threat  = true;
            current_threats_.push_back(tc);
        }
    }

    // Yakın tehdit varsa kaçınma moduna geç
    if (!current_threats_.empty()) {
        evading_ = true;
        current_target_ = std::nullopt;
        return std::nullopt;
    }

    evading_ = false;

    // En yüksek skorlu hedefi seç
    double best_score = -999.0;
    std::optional<TargetCandidate> best;

    double own_lat = own_telem_.position.latitude;
    double own_lon = own_telem_.position.longitude;

    for (const auto& rival : competitors_) {
        double sc = score_candidate(rival);
        if (sc > best_score) {
            best_score = sc;

            TargetCandidate tc;
            tc.team_id    = rival.team_id;
            tc.latitude   = rival.position.latitude;
            tc.longitude  = rival.position.longitude;
            tc.altitude   = rival.position.altitude;
            tc.heading    = rival.heading;
            tc.score      = sc;
            tc.distance_m = haversine_m(own_lat, own_lon,
                                         rival.position.latitude,
                                         rival.position.longitude);
            tc.bearing_deg = bearing(own_lat, own_lon,
                                      rival.position.latitude,
                                      rival.position.longitude);
            best = tc;
        }
    }

    current_target_ = best;
    return best;
}

std::vector<TargetCandidate> DecisionEngine::current_threats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_threats_;
}

std::optional<TargetCandidate> DecisionEngine::current_target() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_target_;
}

// ─────────────────────────────────────────────────
//  DecisionNode
// ─────────────────────────────────────────────────

DecisionNode::DecisionNode(const DecisionConfig& cfg)
    : Node("decision_node"), engine_(cfg)
{
    using std::placeholders::_1;

    pub_target_  = create_publisher<std_msgs::msg::String>("/decision/target",  10);
    pub_threats_ = create_publisher<std_msgs::msg::String>("/decision/threats", 10);
    pub_evade_   = create_publisher<std_msgs::msg::String>("/decision/evade",   10);

    sub_rivals_ = create_subscription<std_msgs::msg::String>(
        "/sunucu_telemetri", 10,
        std::bind(&DecisionNode::on_rivals_telemetry, this, _1));

    sub_own_telem_ = create_subscription<std_msgs::msg::String>(
        "/telemetry/own", 10,
        std::bind(&DecisionNode::on_own_telemetry, this, _1));

    sub_tracker_ = create_subscription<std_msgs::msg::String>(
        "/tracker/events", 10,
        std::bind(&DecisionNode::on_tracker_event, this, _1));

    eval_timer_ = create_wall_timer(
        std::chrono::duration<double>(cfg.reselect_period_s),
        std::bind(&DecisionNode::evaluate_and_publish, this));

    RCLCPP_INFO(get_logger(), "Karar modülü (DecisionNode) başlatıldı");
}

// ─── JSON ayrıştırma (minimal, harici kütüphane gerektirmez) ────────────────

/// Basit JSON string değerini al
static double json_get_double(const std::string& json,
                               const std::string& key,
                               double default_val = 0.0)
{
    std::string search = "\"" + key + "\":";
    auto pos = json.find(search);
    if (pos == std::string::npos) return default_val;
    pos += search.size();
    // Boşluk atla
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
    try {
        size_t end;
        return std::stod(json.substr(pos), &end);
    } catch (...) { return default_val; }
}

static int json_get_int(const std::string& json,
                         const std::string& key,
                         int default_val = 0)
{
    return static_cast<int>(json_get_double(json, key, default_val));
}

void DecisionNode::on_rivals_telemetry(const std_msgs::msg::String::SharedPtr msg)
{
    // Beklenen format:
    // {"konumBilgileri": [{"takim_numarasi":1,"iha_enlem":41.51,...}, ...]}
    const std::string& data = msg->data;

    // "konumBilgileri" dizisini çıkar
    auto arr_start = data.find("[");
    auto arr_end   = data.rfind("]");
    if (arr_start == std::string::npos || arr_end == std::string::npos) return;

    std::string arr = data.substr(arr_start + 1, arr_end - arr_start - 1);

    std::vector<CompetitorUAV> rivals;
    // Her objeyi {…} olarak ayrıştır
    size_t pos = 0;
    while (pos < arr.size()) {
        auto obj_start = arr.find("{", pos);
        if (obj_start == std::string::npos) break;
        auto obj_end = arr.find("}", obj_start);
        if (obj_end == std::string::npos) break;

        std::string obj = arr.substr(obj_start, obj_end - obj_start + 1);

        CompetitorUAV rival;
        rival.team_id              = json_get_int(obj, "takim_numarasi");
        rival.position.latitude    = json_get_double(obj, "iha_enlem");
        rival.position.longitude   = json_get_double(obj, "iha_boylam");
        rival.position.altitude    = json_get_double(obj, "iha_irtifa");
        rival.heading              = json_get_double(obj, "iha_yonelme");

        if (rival.team_id > 0) rivals.push_back(rival);

        pos = obj_end + 1;
    }

    engine_.update_competitors(rivals);
}

void DecisionNode::on_own_telemetry(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& data = msg->data;

    Telemetry own;
    own.position.latitude  = json_get_double(data, "iha_enlem");
    own.position.longitude = json_get_double(data, "iha_boylam");
    own.position.altitude  = json_get_double(data, "iha_irtifa");
    own.heading            = json_get_double(data, "iha_yonelme");
    own.speed              = json_get_double(data, "iha_hiz");

    engine_.update_own_telemetry(own);
}

void DecisionNode::on_tracker_event(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& data = msg->data;
    if (data.rfind("LOCK_COMPLETE:", 0) == 0) {
        try {
            int team_id = std::stoi(data.substr(14));
            engine_.on_lock_complete(team_id);
        } catch (...) {}
    } else if (data == "EVADE_COMPLETE") {
        engine_.on_evade_complete();
    }
}

void DecisionNode::evaluate_and_publish()
{
    auto target = engine_.evaluate();
    auto threats = engine_.current_threats();

    // Hedef yayınla
    {
        auto out = std_msgs::msg::String{};
        out.data = target ? target_to_json(*target) : "{\"target\":null}";
        pub_target_->publish(out);
    }

    // Tehdit listesi yayınla
    {
        auto out = std_msgs::msg::String{};
        out.data = threats_to_json(threats);
        pub_threats_->publish(out);
    }

    // Kaçınma komutu
    if (engine_.is_evading() && !threats.empty()) {
        const auto& t = threats.front();
        std::ostringstream oss;
        oss << "{\"evade\":true"
            << ",\"threat_team\":" << t.team_id
            << ",\"distance_m\":" << t.distance_m
            << "}";
        auto out = std_msgs::msg::String{};
        out.data = oss.str();
        pub_evade_->publish(out);

        RCLCPP_WARN(get_logger(), "KAÇINMA: takim=%d dist=%.1fm",
                    t.team_id, t.distance_m);
    }
}

std::string DecisionNode::target_to_json(const TargetCandidate& t)
{
    std::ostringstream oss;
    oss << std::fixed;
    oss.precision(7);
    oss << "{\"team_id\":"   << t.team_id
        << ",\"latitude\":"  << t.latitude
        << ",\"longitude\":" << t.longitude
        << ",\"altitude\":"  << t.altitude;
    oss.precision(2);
    oss << ",\"distance_m\":" << t.distance_m
        << ",\"bearing_deg\":" << t.bearing_deg
        << ",\"score\":"       << t.score
        << "}";
    return oss.str();
}

std::string DecisionNode::threats_to_json(const std::vector<TargetCandidate>& threats)
{
    if (threats.empty()) return "{\"threats\":[]}";
    std::ostringstream oss;
    oss << "{\"threats\":[";
    for (size_t i = 0; i < threats.size(); ++i) {
        const auto& t = threats[i];
        if (i > 0) oss << ",";
        oss << "{\"team_id\":" << t.team_id
            << ",\"distance_m\":" << std::fixed << t.distance_m << "}";
    }
    oss << "]}";
    return oss.str();
}

}  // namespace siha
