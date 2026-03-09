/**
 * @file decision_engine.hpp
 * @brief Karar Motoru — Akıllı Hedef Seçimi & Kaçınma Algoritması
 *
 * /sunucu_telemetri topic'inden gelen rakip İHA konumlarını değerlendirerek:
 *   - Manevra maliyeti tabanlı hedef seçimi
 *   - Aynı hedefe arka arkaya kilitlenme yasağı (şartname)
 *   - Arkadan yaklaşan tehditlere kaçınma manevrası
 *
 * Çıktılar:
 *   /decision/target  — seçilen hedef (JSON)
 *   /decision/threats — tespit edilen tehditler (JSON)
 *   /decision/evade   — kaçınma komutu (JSON)
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <unordered_set>
#include <string>
#include <functional>
#include <mutex>
#include <optional>

namespace siha {

/// Değerlendirilen hedef bilgisi
struct TargetCandidate {
    int    team_id     = -1;
    double latitude    = 0.0;
    double longitude   = 0.0;
    double altitude    = 0.0;
    double heading     = 0.0;
    double distance_m  = 0.0;
    double bearing_deg = 0.0;
    double score       = 0.0;
    bool   is_threat   = false;
};

/**
 * @class DecisionEngine
 * @brief Merkezi karar motoru.
 *
 * TelemetryManager'dan aldığı rakip ve kendi konumunu kullanarak
 * her değerlendirme döngüsünde en uygun hedefi seçer.
 */
class DecisionEngine {
public:
    explicit DecisionEngine(const DecisionConfig& cfg = DecisionConfig{});

    /// Kendi telemetrimizi güncelle
    void update_own_telemetry(const Telemetry& own);

    /// Rakip listesini güncelle (TelemetryManager'dan)
    void update_competitors(const std::vector<CompetitorUAV>& competitors);

    /// Son kilitlenilen hedefi kaydet (kara liste için)
    void on_lock_complete(int team_id);

    /// Kaçınma tamamlandı — normal moda dön
    void on_evade_complete();

    /**
     * @brief Hedef değerlendirmesi çalıştır.
     * @return Seçilen hedef (boş ise uygun hedef yok)
     */
    std::optional<TargetCandidate> evaluate();

    /// Mevcut tehdit listesi
    std::vector<TargetCandidate> current_threats() const;

    /// Kaçınma manevrası aktif mi?
    bool is_evading() const { return evading_; }

    /// Mevcut seçili hedef
    std::optional<TargetCandidate> current_target() const;

    /// Konfigürasyonu güncelle
    void set_config(const DecisionConfig& cfg) { config_ = cfg; }

private:
    /// İki GPS noktası arasındaki mesafe (Haversine, metre)
    static double haversine_m(double lat1, double lon1,
                               double lat2, double lon2);

    /// Bir noktadan diğerine pusula açısı (0-360)
    static double bearing(double lat1, double lon1,
                           double lat2, double lon2);

    /// İki açı arasındaki fark [-180, 180]
    static double angle_diff(double a, double b);

    /// Tek bir rakip için skor hesapla
    double score_candidate(const CompetitorUAV& rival) const;

    /// Tehdit kontrolü — rakip arkadan yaklaşıyor mu?
    bool is_threat(const CompetitorUAV& rival) const;

    DecisionConfig config_;
    Telemetry own_telem_;
    std::vector<CompetitorUAV> competitors_;
    std::unordered_set<int> blacklist_;  // kilitlenilen hedefler

    std::optional<TargetCandidate> current_target_;
    std::vector<TargetCandidate>   current_threats_;
    bool evading_ = false;

    mutable std::mutex mutex_;
};

/**
 * @class DecisionNode
 * @brief ROS2 arayüzü olan karar modülü.
 *
 * Abonelikler:
 *   /sunucu_telemetri — rakip konum JSON
 *   /telemetry/own    — kendi telemetri
 *   /tracker/events   — kilitlenme olayları
 *
 * Yayınlar:
 *   /decision/target  — seçilen hedef JSON
 *   /decision/threats — tehdit listesi JSON
 *   /decision/evade   — kaçınma komutu JSON
 */
class DecisionNode : public rclcpp::Node {
public:
    explicit DecisionNode(const DecisionConfig& cfg = DecisionConfig{});

private:
    void on_rivals_telemetry(const std_msgs::msg::String::SharedPtr msg);
    void on_own_telemetry(const std_msgs::msg::String::SharedPtr msg);
    void on_tracker_event(const std_msgs::msg::String::SharedPtr msg);
    void evaluate_and_publish();

    /// JSON stringify yardımcıları
    static std::string target_to_json(const TargetCandidate& t);
    static std::string threats_to_json(const std::vector<TargetCandidate>& threats);

    DecisionEngine engine_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_threats_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_evade_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rivals_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_own_telem_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tracker_;

    rclcpp::TimerBase::SharedPtr eval_timer_;
};

}  // namespace siha
    int    team_id     = -1;
    double latitude    = 0.0;
    double longitude   = 0.0;
    double altitude    = 0.0;
    double heading     = 0.0;
    double distance_m  = 0.0;
    double bearing_deg = 0.0;
    double score       = 0.0;
    bool   is_threat   = false;
};

/**
 * @class DecisionEngine
 * @brief Merkezi karar motoru.
 *
 * TelemetryManager'dan aldığı rakip ve kendi konumunu kullanarak
 * her değerlendirme döngüsünde en uygun hedefi seçer.
 */
class DecisionEngine {
public:
    explicit DecisionEngine(const DecisionConfig& cfg = DecisionConfig{});

    /// Kendi telemetrimizi güncelle
    void update_own_telemetry(const Telemetry& own);

    /// Rakip listesini güncelle (TelemetryManager'dan)
    void update_competitors(const std::vector<CompetitorUAV>& competitors);

    /// Son kilitlenilen hedefi kaydet (kara liste için)
    void on_lock_complete(int team_id);

    /// Kaçınma tamamlandı — normal moda dön
    void on_evade_complete();

    /**
     * @brief Hedef değerlendirmesi çalıştır.
     * @return Seçilen hedef (boş ise uygun hedef yok)
     */
    std::optional<TargetCandidate> evaluate();

    /// Mevcut tehdit listesi
    std::vector<TargetCandidate> current_threats() const;

    /// Kaçınma manevrası aktif mi?
    bool is_evading() const { return evading_; }

    /// Mevcut seçili hedef
    std::optional<TargetCandidate> current_target() const;

    /// Konfigürasyonu güncelle
    void set_config(const DecisionConfig& cfg) { config_ = cfg; }

private:
    /// İki GPS noktası arasındaki mesafe (Haversine, metre)
    static double haversine_m(double lat1, double lon1,
                               double lat2, double lon2);

    /// Bir noktadan diğerine pusula açısı (0-360)
    static double bearing(double lat1, double lon1,
                           double lat2, double lon2);

    /// İki açı arasındaki fark [-180, 180]
    static double angle_diff(double a, double b);

    /// Tek bir rakip için skor hesapla
    double score_candidate(const CompetitorUAV& rival) const;

    /// Tehdit kontrolü — rakip arkadan yaklaşıyor mu?
    bool is_threat(const CompetitorUAV& rival) const;

    DecisionConfig config_;
    Telemetry own_telem_;
    std::vector<CompetitorUAV> competitors_;
    std::unordered_set<int> blacklist_;  // kilitlenilen hedefler

    std::optional<TargetCandidate> current_target_;
    std::vector<TargetCandidate>   current_threats_;
    bool evading_ = false;

    mutable std::mutex mutex_;
};

/**
 * @class DecisionNode
 * @brief ROS2 arayüzü olan karar modülü.
 *
 * Abonelikler:
 *   /sunucu_telemetri — rakip konum JSON
 *   /telemetry/own    — kendi telemetri
 *   /tracker/events   — kilitlenme olayları
 *
 * Yayınlar:
 *   /decision/target  — seçilen hedef JSON
 *   /decision/threats — tehdit listesi JSON
 *   /decision/evade   — kaçınma komutu JSON
 */
class DecisionNode : public rclcpp::Node {
public:
    explicit DecisionNode(const DecisionConfig& cfg = DecisionConfig{});

private:
    void on_rivals_telemetry(const std_msgs::msg::String::SharedPtr msg);
    void on_own_telemetry(const std_msgs::msg::String::SharedPtr msg);
    void on_tracker_event(const std_msgs::msg::String::SharedPtr msg);
    void evaluate_and_publish();

    /// JSON stringify yardımcıları
    static std::string target_to_json(const TargetCandidate& t);
    static std::string threats_to_json(const std::vector<TargetCandidate>& threats);

    DecisionEngine engine_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_threats_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_evade_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rivals_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_own_telem_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tracker_;

    rclcpp::TimerBase::SharedPtr eval_timer_;
};

}  // namespace siha
