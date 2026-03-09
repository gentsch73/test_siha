/**
 * @file camera_interface.hpp
 * @brief Kamera Soyutlama Katmanı
 *
 * Gazebo simülasyon kamerası ve gerçek USB/CSI kamera
 * aynı arayüz üzerinden erişilir. Strategy pattern.
 */
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <memory>
#include <functional>
#include <mutex>
#include <chrono>

namespace siha {

/**
 * @class ICameraSource
 * @brief Tüm kamera kaynaklarının uygulaması gereken arayüz.
 */
class ICameraSource {
public:
    virtual ~ICameraSource() = default;

    /// Kamerayı aç / başlat
    virtual bool open() = 0;

    /// Kamerayı kapat
    virtual void close() = 0;

    /// Kamera açık mı?
    virtual bool is_open() const = 0;

    /// Yeni frame oku (blocking). Başarılıysa true döner.
    virtual bool read(cv::Mat& frame) = 0;

    /// Frame genişliği
    virtual int width() const = 0;

    /// Frame yüksekliği
    virtual int height() const = 0;

    /// Gerçek FPS
    virtual double fps() const = 0;

    /// Kaynak adı (debug için)
    virtual std::string source_name() const = 0;
};


/**
 * @class GazeboCamera
 * @brief Gazebo simülasyonundaki kamerayı ROS2 topic'inden okur.
 */
class GazeboCamera : public ICameraSource {
public:
    explicit GazeboCamera(const std::string& ros_topic,
                          int width = 640, int height = 480);
    ~GazeboCamera() override;

    bool open() override;
    void close() override;
    bool is_open() const override;
    bool read(cv::Mat& frame) override;
    int  width() const override  { return width_; }
    int  height() const override { return height_; }
    double fps() const override  { return measured_fps_; }
    std::string source_name() const override { return topic_; }

    /// ROS2 image callback'ini ayarla (Node dışından çağrılır)
    void on_image_received(const cv::Mat& img);

private:
    std::string topic_;
    int width_, height_;
    double measured_fps_ = 0.0;
    bool opened_ = false;

    cv::Mat latest_frame_;
    bool    has_new_frame_ = false;
    mutable std::mutex frame_mutex_;

    std::chrono::steady_clock::time_point last_frame_time_;
    int frame_count_ = 0;
};


/**
 * @class RealCamera
 * @brief Gerçek USB veya CSI kamerayı OpenCV VideoCapture ile okur.
 *
 * V4L2 backend kullanılır. GStreamer pipeline da desteklenir.
 */
class RealCamera : public ICameraSource {
public:
    /// device: "/dev/video0" veya GStreamer pipeline string
    explicit RealCamera(const std::string& device,
                        int width = 640, int height = 480, int fps = 30);
    ~RealCamera() override;

    bool open() override;
    void close() override;
    bool is_open() const override;
    bool read(cv::Mat& frame) override;
    int  width() const override  { return width_; }
    int  height() const override { return height_; }
    double fps() const override  { return target_fps_; }
    std::string source_name() const override { return device_; }

private:
    std::string device_;
    int width_, height_;
    double target_fps_;
    std::unique_ptr<cv::VideoCapture> cap_;
};


/**
 * @brief Factory: Simülasyon moduna göre kamera kaynağı oluştur
 */
std::unique_ptr<ICameraSource> create_camera_source(
    bool simulation_mode,
    const std::string& ros_topic,
    const std::string& device,
    int width, int height, int fps);

}  // namespace siha
