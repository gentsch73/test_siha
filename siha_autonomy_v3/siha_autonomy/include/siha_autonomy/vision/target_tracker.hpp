/**
 * @file target_tracker.hpp
 * @brief Çoklu Hedef Takip Modülü
 *
 * YOLO'dan gelen tespitleri frame'ler arası ilişkilendirir.
 * Her hedefe benzersiz ID atar ve Kalman filtresi ile
 * pozisyon tahmini yapar.
 *
 * Algoritma: SORT (Simple Online and Realtime Tracking) varyantı
 * + IOU eşleştirme + Kalman tahmin
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <map>
#include <unordered_set>
#include <chrono>

namespace siha {

/**
 * @class KalmanTrack
 * @brief Tek bir hedef için Kalman filtresi tabanlı takip nesnesi.
 */
class KalmanTrack {
public:
    KalmanTrack(int id, const BoundingBox& initial_bbox);

    /// Kalman tahmin adımı (prediction)
    BoundingBox predict();

    /// Kalman güncelleme adımı (ölçüm ile düzeltme)
    void update(const BoundingBox& measurement);

    /// Takip ID'si
    int id() const { return id_; }

    /// Mevcut bbox
    BoundingBox bbox() const { return current_bbox_; }

    /// Hız (piksel/frame)
    void velocity(double& vx, double& vy) const;

    /// Kaç frame boyunca görüldü
    int hits() const { return hit_count_; }

    /// Kaç frame boyunca kayıp
    int misses() const { return miss_count_; }

    /// Kayıp sayacını artır
    void mark_missed();

    /// Takip aktif mi? (çok uzun kayıp → sil)
    bool is_alive(int max_miss = 15) const;

    /// Son görülme zamanı
    auto last_seen() const { return last_update_; }

private:
    int id_;
    cv::KalmanFilter kf_;
    BoundingBox current_bbox_;
    int hit_count_ = 1;
    int miss_count_ = 0;
    std::chrono::steady_clock::time_point last_update_;
};


/**
 * @class TargetTracker
 * @brief Çoklu hedef takip yöneticisi.
 *
 * Her frame'de:
 *   1) Mevcut track'lerin tahminlerini oluştur (predict)
 *   2) Yeni tespitlerle eşleştir (IOU + Hungarian)
 *   3) Eşleşenleri güncelle, eşleşmeyenlere yeni track aç
 *   4) Kayıp track'leri sil
 */
class TargetTracker {
public:
    explicit TargetTracker(const VisionConfig& cfg);

    /// Yeni frame tespitlerini güncelle
    std::vector<TrackedTarget> update(const std::vector<BoundingBox>& detections);

    /// Tüm aktif takip edilen hedefler
    std::vector<TrackedTarget> active_targets() const;

    /// Belirli ID'li hedef
    bool get_target(int track_id, TrackedTarget& out) const;

    /// En yakın / en büyük hedefi seç (kilitlenme adayı)
    TrackedTarget select_best_target(
        const std::unordered_set<int>& blacklist) const;

    /// Tüm track'leri temizle
    void reset();

    /// Aktif hedef sayısı
    int active_count() const;

private:
    /// IOU (Intersection over Union) hesapla
    float compute_iou(const BoundingBox& a, const BoundingBox& b) const;

    /// Greedy IOU eşleştirme
    void match_detections(
        const std::vector<BoundingBox>& detections,
        std::vector<std::pair<int, int>>& matched,       // (track_idx, det_idx)
        std::vector<int>& unmatched_tracks,
        std::vector<int>& unmatched_dets) const;

    VisionConfig config_;
    std::vector<KalmanTrack> tracks_;
    int next_id_ = 1;
    float iou_threshold_ = 0.3f;
};

}  // namespace siha
