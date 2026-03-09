/**
 * @file qr_detector.hpp
 * @brief QR Kod Tespit ve Okuma Modülü
 *
 * Kamikaze dalışı sırasında yerdeki 2m×2m QR kodu tespit eder
 * ve içindeki şifreli metni okur.
 * OpenCV QRCodeDetector + fallback olarak zbar kullanır.
 */
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <string>
#include <vector>

namespace siha {

struct QRResult {
    bool     found     = false;
    std::string content;
    std::vector<cv::Point2f> corners;  // 4 köşe noktası
    cv::Point2f center;
};

class QRDetector {
public:
    QRDetector();

    /// Frame'de QR kod ara ve oku
    QRResult detect_and_decode(const cv::Mat& frame);

    /// Sadece tespit et (okumadan)
    bool detect_only(const cv::Mat& frame, std::vector<cv::Point2f>& corners);

    /// Son başarılı okuma
    std::string last_content() const { return last_content_; }

private:
    cv::QRCodeDetector detector_;
    std::string last_content_;
};

}  // namespace siha
