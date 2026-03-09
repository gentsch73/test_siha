/**
 * @file target_tracker.cpp
 * @brief Çoklu Hedef Takip implementasyonu — SORT + Kalman
 */

#include "siha_autonomy/vision/target_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace siha {

// ─────────────────────────────────────────────────
//  KalmanTrack
// ─────────────────────────────────────────────────

KalmanTrack::KalmanTrack(int id, const BoundingBox& initial_bbox)
    : id_(id), current_bbox_(initial_bbox)
{
    // Kalman filtresi: 8 state (x,y,w,h, dx,dy,dw,dh), 4 measurement (x,y,w,h)
    kf_ = cv::KalmanFilter(8, 4, 0);

    // Geçiş matrisi (constant velocity model)
    kf_.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
    for (int i = 0; i < 4; i++) {
        kf_.transitionMatrix.at<float>(i, i + 4) = 1.0f;  // pos += vel * dt
    }

    // Ölçüm matrisi
    kf_.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
    for (int i = 0; i < 4; i++) {
        kf_.measurementMatrix.at<float>(i, i) = 1.0f;
    }

    // Gürültü kovaryansları
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-2));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(1e-1));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar(1.0));

    // İlk durum
    kf_.statePost.at<float>(0) = static_cast<float>(initial_bbox.center_x());
    kf_.statePost.at<float>(1) = static_cast<float>(initial_bbox.center_y());
    kf_.statePost.at<float>(2) = static_cast<float>(initial_bbox.width());
    kf_.statePost.at<float>(3) = static_cast<float>(initial_bbox.height());

    last_update_ = std::chrono::steady_clock::now();
}

BoundingBox KalmanTrack::predict() {
    cv::Mat pred = kf_.predict();

    float cx = pred.at<float>(0);
    float cy = pred.at<float>(1);
    float w  = std::max(1.0f, pred.at<float>(2));
    float h  = std::max(1.0f, pred.at<float>(3));

    BoundingBox bbox;
    bbox.x1 = static_cast<int>(cx - w / 2);
    bbox.y1 = static_cast<int>(cy - h / 2);
    bbox.x2 = static_cast<int>(cx + w / 2);
    bbox.y2 = static_cast<int>(cy + h / 2);
    bbox.confidence = current_bbox_.confidence;
    bbox.class_id = current_bbox_.class_id;

    return bbox;
}

void KalmanTrack::update(const BoundingBox& measurement) {
    cv::Mat meas(4, 1, CV_32F);
    meas.at<float>(0) = static_cast<float>(measurement.center_x());
    meas.at<float>(1) = static_cast<float>(measurement.center_y());
    meas.at<float>(2) = static_cast<float>(measurement.width());
    meas.at<float>(3) = static_cast<float>(measurement.height());

    kf_.correct(meas);

    current_bbox_ = measurement;
    hit_count_++;
    miss_count_ = 0;
    last_update_ = std::chrono::steady_clock::now();
}

void KalmanTrack::velocity(double& vx, double& vy) const {
    vx = static_cast<double>(kf_.statePost.at<float>(4));
    vy = static_cast<double>(kf_.statePost.at<float>(5));
}

void KalmanTrack::mark_missed() {
    miss_count_++;
}

bool KalmanTrack::is_alive(int max_miss) const {
    return miss_count_ <= max_miss;
}

// ─────────────────────────────────────────────────
//  TargetTracker
// ─────────────────────────────────────────────────

TargetTracker::TargetTracker(const VisionConfig& cfg)
    : config_(cfg)
{
}

std::vector<TrackedTarget> TargetTracker::update(
    const std::vector<BoundingBox>& detections)
{
    // 1) Tüm mevcut track'lerin tahminlerini oluştur
    for (auto& track : tracks_) {
        track.predict();
    }

    // 2) Eşleştirme
    std::vector<std::pair<int, int>> matched;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_dets;
    match_detections(detections, matched, unmatched_tracks, unmatched_dets);

    // 3) Eşleşen track'leri güncelle
    for (const auto& [t_idx, d_idx] : matched) {
        tracks_[t_idx].update(detections[d_idx]);
    }

    // 4) Eşleşmeyen track'leri kayıp olarak işaretle
    for (int t_idx : unmatched_tracks) {
        tracks_[t_idx].mark_missed();
    }

    // 5) Yeni tespitler için track oluştur
    for (int d_idx : unmatched_dets) {
        tracks_.emplace_back(next_id_++, detections[d_idx]);
    }

    // 6) Ölü track'leri sil
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [](const KalmanTrack& t) { return !t.is_alive(); }),
        tracks_.end());

    return active_targets();
}

std::vector<TrackedTarget> TargetTracker::active_targets() const {
    std::vector<TrackedTarget> result;
    for (const auto& track : tracks_) {
        if (track.hits() >= 3) {  // en az 3 frame görülmüş olmalı
            TrackedTarget tt;
            tt.track_id = track.id();
            tt.bbox = track.bbox();
            track.velocity(tt.velocity_x, tt.velocity_y);
            tt.frames_visible = track.hits();
            tt.frames_lost = track.misses();
            tt.last_seen = track.last_seen();
            result.push_back(tt);
        }
    }
    return result;
}

bool TargetTracker::get_target(int track_id, TrackedTarget& out) const {
    for (const auto& track : tracks_) {
        if (track.id() == track_id) {
            out.track_id = track.id();
            out.bbox = track.bbox();
            track.velocity(out.velocity_x, out.velocity_y);
            out.frames_visible = track.hits();
            out.frames_lost = track.misses();
            return true;
        }
    }
    return false;
}

TrackedTarget TargetTracker::select_best_target(
    const std::unordered_set<int>& blacklist) const
{
    TrackedTarget best;
    best.track_id = -1;
    int best_score = -1;

    for (const auto& track : tracks_) {
        if (track.hits() < 3) continue;
        if (blacklist.count(track.id())) continue;

        // Skor: alan * confidence * hit_count
        int score = track.bbox().area() * static_cast<int>(track.bbox().confidence * 100)
                    * track.hits();

        if (score > best_score) {
            best_score = score;
            best.track_id = track.id();
            best.bbox = track.bbox();
            track.velocity(best.velocity_x, best.velocity_y);
            best.frames_visible = track.hits();
        }
    }

    return best;
}

void TargetTracker::reset() {
    tracks_.clear();
    next_id_ = 1;
}

int TargetTracker::active_count() const {
    int count = 0;
    for (const auto& t : tracks_) {
        if (t.hits() >= 3) count++;
    }
    return count;
}

float TargetTracker::compute_iou(const BoundingBox& a, const BoundingBox& b) const {
    int x1 = std::max(a.x1, b.x1);
    int y1 = std::max(a.y1, b.y1);
    int x2 = std::min(a.x2, b.x2);
    int y2 = std::min(a.y2, b.y2);

    int inter_w = std::max(0, x2 - x1);
    int inter_h = std::max(0, y2 - y1);
    float inter_area = static_cast<float>(inter_w * inter_h);

    float union_area = static_cast<float>(a.area() + b.area()) - inter_area;

    if (union_area < 1.0f) return 0.0f;
    return inter_area / union_area;
}

void TargetTracker::match_detections(
    const std::vector<BoundingBox>& detections,
    std::vector<std::pair<int, int>>& matched,
    std::vector<int>& unmatched_tracks,
    std::vector<int>& unmatched_dets) const
{
    int num_tracks = static_cast<int>(tracks_.size());
    int num_dets = static_cast<int>(detections.size());

    if (num_tracks == 0) {
        for (int i = 0; i < num_dets; i++) unmatched_dets.push_back(i);
        return;
    }
    if (num_dets == 0) {
        for (int i = 0; i < num_tracks; i++) unmatched_tracks.push_back(i);
        return;
    }

    // IOU matrisi oluştur
    std::vector<std::vector<float>> iou_matrix(num_tracks,
                                                 std::vector<float>(num_dets, 0.0f));

    for (int t = 0; t < num_tracks; t++) {
        for (int d = 0; d < num_dets; d++) {
            iou_matrix[t][d] = compute_iou(tracks_[t].bbox(), detections[d]);
        }
    }

    // Greedy matching (basit ama hızlı)
    std::vector<bool> det_used(num_dets, false);
    std::vector<bool> track_used(num_tracks, false);

    // En yüksek IOU'dan başla
    while (true) {
        float best_iou = iou_threshold_;
        int best_t = -1, best_d = -1;

        for (int t = 0; t < num_tracks; t++) {
            if (track_used[t]) continue;
            for (int d = 0; d < num_dets; d++) {
                if (det_used[d]) continue;
                if (iou_matrix[t][d] > best_iou) {
                    best_iou = iou_matrix[t][d];
                    best_t = t;
                    best_d = d;
                }
            }
        }

        if (best_t < 0) break;

        matched.emplace_back(best_t, best_d);
        track_used[best_t] = true;
        det_used[best_d] = true;
    }

    for (int t = 0; t < num_tracks; t++) {
        if (!track_used[t]) unmatched_tracks.push_back(t);
    }
    for (int d = 0; d < num_dets; d++) {
        if (!det_used[d]) unmatched_dets.push_back(d);
    }
}

}  // namespace siha
