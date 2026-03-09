/**
 * @file yolo_detector.cpp
 * @brief YOLO Nesne Tespit Modülü implementasyonu
 *
 * OpenCV DNN backend ile ONNX model inference.
 * Letterbox preprocessing + NMS postprocessing.
 */

#include "siha_autonomy/vision/yolo_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace siha {

YoloDetector::YoloDetector(const VisionConfig& cfg)
    : config_(cfg),
      input_w_(cfg.input_size),
      input_h_(cfg.input_size),
      conf_thresh_(cfg.confidence_thresh),
      nms_thresh_(cfg.nms_thresh)
{
}

YoloDetector::~YoloDetector() = default;

bool YoloDetector::load_model() {
    try {
        net_ = cv::dnn::readNetFromONNX(config_.model_path);

        // Backend seçimi
        if (config_.use_tensorrt) {
            // TensorRT varsa kullan (NVIDIA Jetson)
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        } else {
            // CPU fallback
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }

        // FP16 desteği
        if (config_.use_fp16) {
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
        }

        model_loaded_ = true;
        std::cout << "[YoloDetector] Model yüklendi: " << config_.model_path << std::endl;
        return true;

    } catch (const cv::Exception& e) {
        std::cerr << "[YoloDetector] Model yükleme hatası: " << e.what() << std::endl;
        model_loaded_ = false;
        return false;
    }
}

bool YoloDetector::reload_model(const std::string& new_path) {
    config_.model_path = new_path;
    model_loaded_ = false;
    return load_model();
}

std::vector<BoundingBox> YoloDetector::detect(const cv::Mat& frame) {
    std::vector<BoundingBox> results;
    if (!model_loaded_ || frame.empty()) return results;

    auto t_start = std::chrono::high_resolution_clock::now();

    // 1) Preprocessing (letterbox resize)
    cv::Mat blob = preprocess(frame);

    // 2) Forward pass
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    // 3) Postprocessing
    if (!outputs.empty()) {
        results = postprocess(outputs[0], frame.cols, frame.rows);
    }

    // 4) NMS
    apply_nms(results);

    auto t_end = std::chrono::high_resolution_clock::now();
    last_infer_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return results;
}

cv::Mat YoloDetector::preprocess(const cv::Mat& frame) {
    int orig_w = frame.cols;
    int orig_h = frame.rows;

    // Letterbox resize: en-boy oranını koru
    float scale = std::min(
        static_cast<float>(input_w_) / orig_w,
        static_cast<float>(input_h_) / orig_h);

    int new_w = static_cast<int>(orig_w * scale);
    int new_h = static_cast<int>(orig_h * scale);

    int pad_x = (input_w_ - new_w) / 2;
    int pad_y = (input_h_ - new_h) / 2;

    letterbox_info_.scale = scale;
    letterbox_info_.pad_x = pad_x;
    letterbox_info_.pad_y = pad_y;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(new_w, new_h));

    // Gri padding ekle
    cv::Mat padded(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));

    // Blob oluştur: BGR → RGB, normalize (0-1), NCHW
    cv::Mat blob = cv::dnn::blobFromImage(
        padded, 1.0 / 255.0,
        cv::Size(input_w_, input_h_),
        cv::Scalar(0, 0, 0),
        true,   // swapRB (BGR → RGB)
        false   // crop
    );

    return blob;
}

std::vector<BoundingBox> YoloDetector::postprocess(
    const cv::Mat& output, int orig_w, int orig_h)
{
    std::vector<BoundingBox> results;

    // YOLOv8 çıktı formatı: [1, num_classes+4, num_detections]
    // Transpose gerekebilir: [1, 84, 8400] → [8400, 84]
    cv::Mat det = output;

    // Boyut kontrol
    int rows = det.size[2];  // detection sayısı
    int cols = det.size[1];  // 4 (bbox) + num_classes

    // Transpose
    cv::Mat det_t;
    cv::Mat det_2d = det.reshape(1, cols);  // [cols, rows]
    cv::transpose(det_2d, det_t);           // [rows, cols]

    for (int i = 0; i < det_t.rows; i++) {
        const float* row = det_t.ptr<float>(i);

        // Bbox: cx, cy, w, h
        float cx = row[0];
        float cy = row[1];
        float w  = row[2];
        float h  = row[3];

        // Class scores (4'ten sonrası)
        float max_score = 0.0f;
        int max_class = 0;
        for (int c = 4; c < det_t.cols; c++) {
            if (row[c] > max_score) {
                max_score = row[c];
                max_class = c - 4;
            }
        }

        if (max_score < conf_thresh_) continue;

        // Letterbox → orijinal koordinatlara dönüştür
        float x1 = (cx - w / 2.0f - letterbox_info_.pad_x) / letterbox_info_.scale;
        float y1 = (cy - h / 2.0f - letterbox_info_.pad_y) / letterbox_info_.scale;
        float x2 = (cx + w / 2.0f - letterbox_info_.pad_x) / letterbox_info_.scale;
        float y2 = (cy + h / 2.0f - letterbox_info_.pad_y) / letterbox_info_.scale;

        // Sınırları kontrol et
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(orig_w - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(orig_h - 1)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(orig_w - 1)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(orig_h - 1)));

        BoundingBox bbox;
        bbox.x1 = static_cast<int>(x1);
        bbox.y1 = static_cast<int>(y1);
        bbox.x2 = static_cast<int>(x2);
        bbox.y2 = static_cast<int>(y2);
        bbox.confidence = max_score;
        bbox.class_id = max_class;

        if (bbox.width() > 0 && bbox.height() > 0) {
            results.push_back(bbox);
        }
    }

    return results;
}

void YoloDetector::apply_nms(std::vector<BoundingBox>& boxes) {
    if (boxes.size() <= 1) return;

    // OpenCV NMS
    std::vector<cv::Rect> rects;
    std::vector<float> scores;

    for (const auto& b : boxes) {
        rects.emplace_back(b.x1, b.y1, b.width(), b.height());
        scores.push_back(b.confidence);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(rects, scores, conf_thresh_, nms_thresh_, indices);

    std::vector<BoundingBox> filtered;
    filtered.reserve(indices.size());
    for (int idx : indices) {
        filtered.push_back(boxes[idx]);
    }

    boxes = std::move(filtered);
}

void YoloDetector::set_confidence_threshold(float thresh) {
    conf_thresh_ = thresh;
}

void YoloDetector::set_nms_threshold(float thresh) {
    nms_thresh_ = thresh;
}

}  // namespace siha
