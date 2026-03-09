/**
 * @file video_recorder.hpp
 * @brief Video Kayıt Modülü
 *
 * Şartname gereksinimlerini karşılayan video kaydı:
 *   - H264 codec, MP4 format
 *   - Minimum 640×480, sabit FPS (≥15)
 *   - Sağ üst: sunucu saati (milisaniye hassasiyet)
 *   - Kilitlenme dörtgeni: kırmızı (#FF0000), max 3px
 *   - FFmpeg subprocess ile encoding
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <cstdio>

namespace siha {

/**
 * @class OverlayRenderer
 * @brief Frame üzerine kilitlenme dörtgeni ve zaman damgası çizen yardımcı.
 */
class OverlayRenderer {
public:
    OverlayRenderer(int rect_thickness = 3);

    /// Kilitlenme dörtgenini çiz
    void draw_lockon_rect(cv::Mat& frame, const BoundingBox& bbox);

    /// Vuruş alanı sınırlarını çiz (debug)
    void draw_strike_zone(cv::Mat& frame, int frame_w, int frame_h,
                           double h_pct, double v_pct);

    /// Sunucu saatini sağ üst köşeye yaz
    void draw_server_time(cv::Mat& frame, const ServerTime& st);

    /// Takım adı ve bilgi overlay
    void draw_info(cv::Mat& frame, const std::string& text, int y_offset = 30);

    /// FPS göster
    void draw_fps(cv::Mat& frame, double fps);

private:
    int thickness_;
    cv::Scalar lock_color_{0, 0, 255};  // BGR: kırmızı
};


/**
 * @class VideoRecorder
 * @brief FFmpeg tabanlı video kayıt motoru.
 *
 * Producer-consumer pattern: frame'ler kuyruğa eklenir,
 * arka plan thread'i encode eder.
 */
class VideoRecorder {
public:
    explicit VideoRecorder(const RecordingConfig& cfg);
    ~VideoRecorder();

    /// Kaydı başlat
    bool start(const std::string& filename = "");

    /// Kaydı durdur
    void stop();

    /// Kayıt yapılıyor mu?
    bool is_recording() const { return recording_.load(); }

    /// Frame ekle (overlay eklendikten sonra)
    void write_frame(const cv::Mat& frame);

    /// Çıktı dosya yolu
    std::string output_path() const { return output_path_; }

    /// Overlay yardımcısına erişim
    OverlayRenderer& overlay() { return overlay_; }

private:
    void encode_loop();   // Arka plan FFmpeg encode thread
    bool open_ffmpeg_pipe();
    void close_ffmpeg_pipe();

    RecordingConfig config_;
    OverlayRenderer overlay_;

    std::string output_path_;
    FILE* ffmpeg_pipe_ = nullptr;

    // Frame kuyruğu
    std::queue<cv::Mat> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::thread encode_thread_;
    std::atomic<bool> recording_{false};
};

}  // namespace siha
