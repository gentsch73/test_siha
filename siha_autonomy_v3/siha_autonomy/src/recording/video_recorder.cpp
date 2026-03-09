/**
 * @file video_recorder.cpp
 * @brief Video Kayıt Modülü implementasyonu — FFmpeg pipe encoding
 */

#include "siha_autonomy/recording/video_recorder.hpp"
#include <opencv2/highgui.hpp>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <chrono>

namespace siha {

// ─────────────────────────────────────────────────
//  OverlayRenderer
// ─────────────────────────────────────────────────

OverlayRenderer::OverlayRenderer(int rect_thickness)
    : thickness_(rect_thickness)
{
}

void OverlayRenderer::draw_lockon_rect(cv::Mat& frame, const BoundingBox& bbox) {
    cv::rectangle(frame,
        cv::Point(bbox.x1, bbox.y1),
        cv::Point(bbox.x2, bbox.y2),
        lock_color_,
        thickness_);
}

void OverlayRenderer::draw_strike_zone(cv::Mat& frame, int frame_w, int frame_h,
                                         double h_pct, double v_pct)
{
    double margin_h = (1.0 - h_pct) / 2.0;
    double margin_v = (1.0 - v_pct) / 2.0;

    int x1 = static_cast<int>(frame_w * margin_h);
    int y1 = static_cast<int>(frame_h * margin_v);
    int x2 = static_cast<int>(frame_w * (1.0 - margin_h));
    int y2 = static_cast<int>(frame_h * (1.0 - margin_v));

    cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                  cv::Scalar(0, 255, 255), 1);  // sarı, ince çizgi
}

void OverlayRenderer::draw_server_time(cv::Mat& frame, const ServerTime& st) {
    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << st.hour << ":"
        << std::setw(2) << st.minute << ":"
        << std::setw(2) << st.second << "."
        << std::setw(3) << st.millisecond;

    std::string text = oss.str();

    // Sağ üst köşeye yaz
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                          0.6, 2, &baseline);
    int x = frame.cols - text_size.width - 10;
    int y = text_size.height + 10;

    // Arka plan kutusu (okunabilirlik)
    cv::rectangle(frame,
        cv::Point(x - 5, y - text_size.height - 5),
        cv::Point(x + text_size.width + 5, y + baseline + 5),
        cv::Scalar(0, 0, 0), cv::FILLED);

    cv::putText(frame, text, cv::Point(x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
}

void OverlayRenderer::draw_info(cv::Mat& frame, const std::string& text,
                                  int y_offset)
{
    cv::putText(frame, text, cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
}

void OverlayRenderer::draw_fps(cv::Mat& frame, double fps) {
    std::ostringstream oss;
    oss << "FPS: " << std::fixed << std::setprecision(1) << fps;
    cv::putText(frame, oss.str(), cv::Point(10, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
}

// ─────────────────────────────────────────────────
//  VideoRecorder
// ─────────────────────────────────────────────────

VideoRecorder::VideoRecorder(const RecordingConfig& cfg)
    : config_(cfg), overlay_(3)
{
    std::filesystem::create_directories(cfg.output_dir);
}

VideoRecorder::~VideoRecorder() {
    stop();
}

bool VideoRecorder::start(const std::string& filename) {
    if (recording_.load()) return true;

    // Dosya adı oluştur
    if (filename.empty()) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << config_.output_dir << "siha_"
            << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
            << "." << config_.format;
        output_path_ = oss.str();
    } else {
        output_path_ = config_.output_dir + filename;
    }

    if (!open_ffmpeg_pipe()) {
        std::cerr << "[VideoRecorder] FFmpeg pipe açılamadı!" << std::endl;
        return false;
    }

    recording_ = true;
    encode_thread_ = std::thread(&VideoRecorder::encode_loop, this);

    std::cout << "[VideoRecorder] Kayıt başlatıldı: " << output_path_ << std::endl;
    return true;
}

void VideoRecorder::stop() {
    if (!recording_.load()) return;

    recording_ = false;
    queue_cv_.notify_all();

    if (encode_thread_.joinable()) {
        encode_thread_.join();
    }

    close_ffmpeg_pipe();
    std::cout << "[VideoRecorder] Kayıt durduruldu: " << output_path_ << std::endl;
}

void VideoRecorder::write_frame(const cv::Mat& frame) {
    if (!recording_.load()) return;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    // Kuyruk çok doluysa en eski frame'i at (backpressure)
    if (frame_queue_.size() > 60) {
        frame_queue_.pop();
    }
    frame_queue_.push(frame.clone());
    queue_cv_.notify_one();
}

void VideoRecorder::encode_loop() {
    while (recording_.load() || !frame_queue_.empty()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait_for(lock, std::chrono::milliseconds(100),
                [this]() { return !frame_queue_.empty() || !recording_.load(); });

            if (frame_queue_.empty()) continue;
            frame = frame_queue_.front();
            frame_queue_.pop();
        }

        // FFmpeg'e yaz
        if (ffmpeg_pipe_ && !frame.empty()) {
            cv::Mat bgr;
            if (frame.channels() == 3) {
                bgr = frame;
            } else {
                cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
            }

            // Boyut kontrolü
            if (bgr.cols != config_.video_width || bgr.rows != config_.video_height) {
                cv::resize(bgr, bgr, cv::Size(config_.video_width, config_.video_height));
            }

            fwrite(bgr.data, 1, bgr.total() * bgr.elemSize(), ffmpeg_pipe_);
        }
    }
}

bool VideoRecorder::open_ffmpeg_pipe() {
    std::ostringstream cmd;
    cmd << "ffmpeg -y"
        << " -f rawvideo"
        << " -vcodec rawvideo"
        << " -pix_fmt bgr24"
        << " -s " << config_.video_width << "x" << config_.video_height
        << " -r " << config_.video_fps
        << " -i -"
        << " -c:v " << config_.codec
        << " -preset ultrafast"
        << " -pix_fmt yuv420p"
        << " -movflags +faststart"
        << " " << output_path_
        << " 2>/dev/null";

    ffmpeg_pipe_ = popen(cmd.str().c_str(), "w");
    return ffmpeg_pipe_ != nullptr;
}

void VideoRecorder::close_ffmpeg_pipe() {
    if (ffmpeg_pipe_) {
        pclose(ffmpeg_pipe_);
        ffmpeg_pipe_ = nullptr;
    }
}

}  // namespace siha
