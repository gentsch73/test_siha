/**
 * @file yolo_detector.hpp
 * @brief YOLO Nesne Tespit Modülü
 *
 * ONNX Runtime veya TensorRT backend ile YOLO inference.
 * Tespit edilen nesnelerin bounding box, confidence ve class bilgilerini döner.
 *
 * Desteklenen modeller: YOLOv8, YOLOv11 (ONNX veya TensorRT engine)
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <memory>

namespace siha {

/**
 * @class YoloDetector
 * @brief YOLO tabanlı nesne tespit motoru.
 *
 * Kullanım:
 * @code
 *   YoloDetector det(config.vision);
 *   det.load_model();
 *   auto results = det.detect(frame);
 * @endcode
 */
class YoloDetector {
public:
    explicit YoloDetector(const VisionConfig& cfg);
    ~YoloDetector();

    /// Modeli yükle. Başarılıysa true.
    bool load_model();

    /// Modeli yeniden yükle (sıcak değişim)
    bool reload_model(const std::string& new_path);

    /// Tek bir frame üzerinde tespit yap
    std::vector<BoundingBox> detect(const cv::Mat& frame);

    /// Son inference süresi (ms)
    double last_inference_ms() const { return last_infer_ms_; }

    /// Model yüklü mü?
    bool is_loaded() const { return model_loaded_; }

    /// Confidence eşiğini dinamik ayarla
    void set_confidence_threshold(float thresh);

    /// NMS eşiğini dinamik ayarla
    void set_nms_threshold(float thresh);

private:
    /// Frame'i model giriş boyutuna ön-işle
    cv::Mat preprocess(const cv::Mat& frame);

    /// Model çıktısını parse et → BoundingBox listesi
    std::vector<BoundingBox> postprocess(const cv::Mat& output,
                                          int orig_w, int orig_h);

    /// NMS (Non-Maximum Suppression) uygula
    void apply_nms(std::vector<BoundingBox>& boxes);

    VisionConfig config_;
    bool model_loaded_ = false;
    double last_infer_ms_ = 0.0;

    // OpenCV DNN backend (ONNX)
    cv::dnn::Net net_;

    // Model parametreleri
    int input_w_, input_h_;
    float conf_thresh_, nms_thresh_;
    int num_classes_ = 1;  // İHA sınıfı

    // Letterbox padding bilgisi
    struct LetterboxInfo {
        float scale = 1.0f;
        int pad_x = 0, pad_y = 0;
    };
    LetterboxInfo letterbox_info_;
};

}  // namespace siha
