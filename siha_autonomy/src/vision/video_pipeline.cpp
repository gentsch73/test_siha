/**
 * @file video_pipeline.cpp
 * @brief Video İşleme Hattı implementasyonu
 */

#include "siha_autonomy/vision/video_pipeline.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace siha {

// ── FrameProcessor ──

FrameProcessor::FrameProcessor(const VisionConfig& cfg)
    : target_w_(cfg.frame_width), target_h_(cfg.frame_height)
{
}

void FrameProcessor::process(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) { output = cv::Mat(); return; }

    cv::Mat temp = input;

    // 1) Boyut standardizasyonu
    if (temp.cols != target_w_ || temp.rows != target_h_) {
        cv::resize(temp, temp, cv::Size(target_w_, target_h_));
    }

    // 2) Lens distortion düzeltme
    if (has_calibration_) {
        cv::Mat undistorted;
        cv::remap(temp, undistorted, map1_, map2_, cv::INTER_LINEAR);
        temp = undistorted;
    }

    // 3) Histogram eşitleme (opsiyonel, düşük ışık)
    if (hist_eq_ && temp.channels() == 3) {
        cv::Mat lab;
        cv::cvtColor(temp, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> channels;
        cv::split(lab, channels);
        cv::equalizeHist(channels[0], channels[0]);
        cv::merge(channels, lab);
        cv::cvtColor(lab, temp, cv::COLOR_Lab2BGR);
    }

    output = temp;
}

void FrameProcessor::load_calibration(const std::string& calib_file) {
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened()) return;

    fs["camera_matrix"] >> camera_matrix_;
    fs["dist_coeffs"] >> dist_coeffs_;
    fs.release();

    if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
        cv::initUndistortRectifyMap(
            camera_matrix_, dist_coeffs_, cv::Mat(),
            camera_matrix_, cv::Size(target_w_, target_h_),
            CV_16SC2, map1_, map2_);
        has_calibration_ = true;
    }
}

// ── VideoPipeline ──

VideoPipeline::VideoPipeline(const VisionConfig& cfg, bool sim)
    : config_(cfg), simulation_mode_(sim), processor_(cfg)
{
}

VideoPipeline::~VideoPipeline() {
    stop();
}

bool VideoPipeline::start(std::shared_ptr<ICameraSource> camera) {
    camera_ = std::move(camera);
    if (!camera_ || !camera_->open()) {
        std::cerr << "[VideoPipeline] Kamera açılamadı!" << std::endl;
        return false;
    }

    running_ = true;
    capture_thread_ = std::thread(&VideoPipeline::capture_loop, this);
    std::cout << "[VideoPipeline] Başlatıldı: " << camera_->source_name() << std::endl;
    return true;
}

void VideoPipeline::stop() {
    running_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
    if (camera_) camera_->close();
}

void VideoPipeline::add_frame_listener(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    listeners_.push_back(std::move(cb));
}

bool VideoPipeline::get_latest_frame(cv::Mat& frame) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) return false;
    frame = latest_frame_.clone();
    return true;
}

void VideoPipeline::push_gazebo_frame(const cv::Mat& frame) {
    if (auto* gz_cam = dynamic_cast<GazeboCamera*>(camera_.get())) {
        gz_cam->on_image_received(frame);
    }
}

void VideoPipeline::capture_loop() {
    fps_start_ = std::chrono::steady_clock::now();
    fps_frame_count_ = 0;

    while (running_.load()) {
        cv::Mat raw_frame;
        if (!camera_->read(raw_frame) || raw_frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Frame işle
        cv::Mat processed;
        processor_.process(raw_frame, processed);

        // Son frame'i güncelle
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = processed;
        }

        // FPS hesapla
        fps_frame_count_++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start_).count();
        if (elapsed >= 1.0) {
            current_fps_ = fps_frame_count_ / elapsed;
            fps_frame_count_ = 0;
            fps_start_ = now;
        }

        // Timestamp
        double ts = std::chrono::duration<double, std::milli>(
            now.time_since_epoch()).count();

        // Listener'lara dağıt
        distribute_frame(processed, ts);
    }
}

void VideoPipeline::distribute_frame(const cv::Mat& frame, double ts) {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    for (auto& cb : listeners_) {
        cb(frame, ts);
    }
}

// ── Kamera Implementasyonları ──

GazeboCamera::GazeboCamera(const std::string& topic, int w, int h)
    : topic_(topic), width_(w), height_(h)
{
}

GazeboCamera::~GazeboCamera() { close(); }

bool GazeboCamera::open() {
    opened_ = true;
    last_frame_time_ = std::chrono::steady_clock::now();
    return true;
}

void GazeboCamera::close() { opened_ = false; }
bool GazeboCamera::is_open() const { return opened_; }

bool GazeboCamera::read(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_new_frame_) return false;
    frame = latest_frame_.clone();
    has_new_frame_ = false;

    // FPS ölçüm
    frame_count_++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
    if (elapsed >= 1.0) {
        measured_fps_ = frame_count_ / elapsed;
        frame_count_ = 0;
        last_frame_time_ = now;
    }
    return true;
}

void GazeboCamera::on_image_received(const cv::Mat& img) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = img.clone();
    has_new_frame_ = true;
}

RealCamera::RealCamera(const std::string& device, int w, int h, int fps)
    : device_(device), width_(w), height_(h), target_fps_(fps)
{
}

RealCamera::~RealCamera() { close(); }

bool RealCamera::open() {
    cap_ = std::make_unique<cv::VideoCapture>();

    // GStreamer pipeline desteği
    if (device_.find("!") != std::string::npos ||
        device_.find("gst") != std::string::npos) {
        cap_->open(device_, cv::CAP_GSTREAMER);
    } else if (device_.find("/dev/") != std::string::npos) {
        // V4L2 device
        int dev_num = 0;
        if (device_.length() > 10) dev_num = device_.back() - '0';
        cap_->open(dev_num, cv::CAP_V4L2);
    } else {
        cap_->open(device_);
    }

    if (!cap_->isOpened()) return false;

    cap_->set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_->set(cv::CAP_PROP_FPS, target_fps_);

    return true;
}

void RealCamera::close() {
    if (cap_ && cap_->isOpened()) cap_->release();
}

bool RealCamera::is_open() const {
    return cap_ && cap_->isOpened();
}

bool RealCamera::read(cv::Mat& frame) {
    if (!cap_ || !cap_->isOpened()) return false;
    return cap_->read(frame);
}

// ── Factory ──

std::unique_ptr<ICameraSource> create_camera_source(
    bool simulation_mode,
    const std::string& ros_topic,
    const std::string& device,
    int width, int height, int fps)
{
    if (simulation_mode) {
        return std::make_unique<GazeboCamera>(ros_topic, width, height);
    } else {
        return std::make_unique<RealCamera>(device, width, height, fps);
    }
}

}  // namespace siha
