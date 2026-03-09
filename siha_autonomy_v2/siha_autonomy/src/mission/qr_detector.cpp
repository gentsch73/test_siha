/**
 * @file qr_detector.cpp
 */
#include "siha_autonomy/mission/qr_detector.hpp"
#include <opencv2/imgproc.hpp>

namespace siha {

QRDetector::QRDetector() = default;

QRResult QRDetector::detect_and_decode(const cv::Mat& frame) {
    QRResult result;
    if (frame.empty()) return result;

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    // Kontrast artırma (dalış sırasında daha net okuma)
    cv::equalizeHist(gray, gray);

    std::vector<cv::Point> points;
    std::string data = detector_.detectAndDecode(gray, points);

    if (!data.empty() && !points.empty()) {
        result.found = true;
        result.content = data;
        last_content_ = data;

        // Köşe noktalarını aktar
        for (const auto& p : points) {
            result.corners.emplace_back(
                static_cast<float>(p.x), static_cast<float>(p.y));
        }

        // Merkez hesapla
        float cx = 0, cy = 0;
        for (const auto& c : result.corners) {
            cx += c.x; cy += c.y;
        }
        if (!result.corners.empty()) {
            result.center.x = cx / result.corners.size();
            result.center.y = cy / result.corners.size();
        }
    }

    return result;
}

bool QRDetector::detect_only(const cv::Mat& frame,
                              std::vector<cv::Point2f>& corners) {
    if (frame.empty()) return false;
    // OpenCV detectMulti kullanılabilir
    return false;  // TODO
}

}  // namespace siha
