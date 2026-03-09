/**
 * @file video_pipeline.hpp
 * @brief Video İşleme Hattı — FFmpeg entegrasyonu
 *
 * Kameradan gelen ham görüntüyü:
 *   1) Gerekli formata dönüştürür (resize, color space)
 *   2) Frame kuyruğu yönetir (producer-consumer)
 *   3) FFmpeg subprocess ile encode eder
 *
 * Gazebo veya gerçek kamera fark etmez, bu modül
 * her kaynağı normalize eder.
 */
#pragma once

#include "siha_autonomy/vision/camera_interface.hpp"
#include "siha_autonomy/core/config.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>

namespace siha {

/**
 * @class FrameProcessor
 * @brief Ham frame → normalize edilmiş frame dönüşüm birimi.
 *
 * - Boyut standardizasyonu (640×480 veya konfigüre edilebilir)
 * - Renk uzayı dönüşümü (BGR ↔ RGB)
 * - Lens distortion düzeltme (opsiyonel)
 * - Histogram eşitleme (düşük ışık koşulları)
 */
class FrameProcessor {
public:
    explicit FrameProcessor(const VisionConfig& cfg);

    /// Ham frame'i işle. Sonucu output'a yazar.
    void process(const cv::Mat& input, cv::Mat& output);

    /// Lens kalibrasyon matrisi yükle
    void load_calibration(const std::string& calib_file);

    /// Histogram eşitleme aç/kapat
    void set_histogram_eq(bool enabled) { hist_eq_ = enabled; }

private:
    int target_w_, target_h_;
    bool hist_eq_ = false;
    bool has_calibration_ = false;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat map1_, map2_;
};


/**
 * @class VideoPipeline
 * @brief Kameradan frame okuma → işleme → dağıtım hattı.
 *
 * Arka plan thread'inde çalışır. Her yeni frame işlendiğinde
 * kayıtlı callback'ler çağrılır.
 */
class VideoPipeline {
public:
    using FrameCallback = std::function<void(const cv::Mat& frame,
                                              double timestamp_ms)>;

    explicit VideoPipeline(const VisionConfig& cfg, bool simulation_mode);
    ~VideoPipeline();

    /// Hattı başlat
    bool start(std::shared_ptr<ICameraSource> camera);

    /// Hattı durdur
    void stop();

    /// Çalışıyor mu?
    bool is_running() const { return running_.load(); }

    /// Frame callback kaydet (birden fazla olabilir: YOLO, Recorder, vb.)
    void add_frame_listener(FrameCallback cb);

    /// Son işlenmiş frame'i al (thread-safe)
    bool get_latest_frame(cv::Mat& frame) const;

    /// Mevcut FPS
    double current_fps() const { return current_fps_.load(); }

    /// Gazebo kamerasına frame push et (ROS2 callback'ten)
    void push_gazebo_frame(const cv::Mat& frame);

private:
    void capture_loop();    // Arka plan thread
    void distribute_frame(const cv::Mat& frame, double ts);

    VisionConfig config_;
    bool simulation_mode_;

    std::shared_ptr<ICameraSource> camera_;
    FrameProcessor processor_;

    std::vector<FrameCallback> listeners_;
    mutable std::mutex listeners_mutex_;

    cv::Mat latest_frame_;
    mutable std::mutex frame_mutex_;

    std::thread capture_thread_;
    std::atomic<bool> running_{false};
    std::atomic<double> current_fps_{0.0};

    // FPS ölçüm
    std::chrono::steady_clock::time_point fps_start_;
    int fps_frame_count_ = 0;
};

}  // namespace siha
