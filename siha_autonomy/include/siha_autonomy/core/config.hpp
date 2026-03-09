/**
 * @file config.hpp
 * @brief Saerotech SİHA Otonom Sistem — Global Konfigürasyon
 *
 * Tüm alt modüller bu dosyadaki yapıları kullanır.
 * Parametreler YAML dosyasından veya ROS2 parametrelerinden yüklenebilir.
 */
#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <chrono>

namespace siha {

// ─────────────────────────────────────────────────
//  Görev Durumları (Mission States)
// ─────────────────────────────────────────────────
enum class MissionType : uint8_t {
    SAVASAN_IHA  = 0,   // Savaşan İHA Görevi
    KAMIKAZE     = 1    // Kamikaze Görevi
};

enum class FlightPhase : uint8_t {
    IDLE             = 0,   // Yerde, motorlar kapalı
    PRE_ARM          = 1,   // Arm öncesi kontroller
    ARMED            = 2,   // Armlandı, kalkışa hazır
    TAKEOFF          = 3,   // Otonom kalkış
    CLIMB            = 4,   // Hedef irtifaya tırmanma
    MISSION_SELECT   = 5,   // Görev seçim modu
    SEARCH           = 6,   // Hedef arama
    TRACK            = 7,   // Hedef takip
    LOCKON           = 8,   // Kilitlenme
    EVADE            = 9,   // Rakipten kaçınma
    KAMIKAZE_CLIMB   = 10,  // Kamikaze tırmanma (≥100m)
    KAMIKAZE_ALIGN   = 11,  // QR hedefine hizalanma
    KAMIKAZE_DIVE    = 12,  // Dalış
    KAMIKAZE_PULLUP  = 13,  // Dalıştan çıkış
    LOITER           = 14,  // Serbest uçuş / bekleme
    RTL              = 15,  // Eve dönüş
    LAND             = 16,  // İniş
    BOUNDARY_RETURN  = 17,  // Sınır aşımı → güvenli bölge
    SIGNAL_LOST      = 18,  // Haberleşme kopması → çember
    EMERGENCY        = 19   // Acil durum
};

enum class TargetStatus : uint8_t {
    NO_TARGET        = 0,
    DETECTED         = 1,
    TRACKING         = 2,
    IN_STRIKE_ZONE   = 3,
    LOCKED           = 4,
    LOST             = 5
};

// ─────────────────────────────────────────────────
//  Veri Yapıları
// ─────────────────────────────────────────────────

/// GPS Koordinatı
struct GeoPoint {
    double latitude   = 0.0;
    double longitude  = 0.0;
    double altitude   = 0.0;  // metre (AGL)
};

/// İHA Telemetri Verisi
struct Telemetry {
    GeoPoint position;
    double heading    = 0.0;  // derece (0-360)
    double pitch      = 0.0;  // dikilme (derece)
    double roll       = 0.0;  // yatış  (derece)
    double speed      = 0.0;  // m/s
    int    battery    = 100;  // yüzde
    bool   is_armed   = false;
    bool   is_autonomous = false;
    std::chrono::steady_clock::time_point timestamp;
};

/// Bounding Box (Piksel Koordinatı)
struct BoundingBox {
    int x1 = 0, y1 = 0;  // sol üst
    int x2 = 0, y2 = 0;  // sağ alt
    float confidence = 0.0f;
    int class_id = -1;

    int width()   const { return x2 - x1; }
    int height()  const { return y2 - y1; }
    int center_x() const { return (x1 + x2) / 2; }
    int center_y() const { return (y1 + y2) / 2; }
    int area()     const { return width() * height(); }
};

/// Takip Edilen Hedef
struct TrackedTarget {
    int       track_id       = -1;
    BoundingBox bbox;
    double    velocity_x     = 0.0;  // piksel/sn
    double    velocity_y     = 0.0;
    int       frames_visible = 0;
    int       frames_lost    = 0;
    bool      is_blacklisted = false;
    std::chrono::steady_clock::time_point last_seen;
};

/// Kilitlenme Bilgisi
struct LockonInfo {
    bool   is_locked       = false;
    int    target_id       = -1;
    double lock_duration_s = 0.0;       // saniye
    int    lock_counter    = 0;         // sayaç (≥4 gerekli)
    int    tolerance_counter = 0;       // tolerans sayacı
    int    total_lost_frames = 0;       // kayıp kare sayısı
    BoundingBox lock_rect;              // kilitlenme dörtgeni
    std::chrono::steady_clock::time_point lock_start;
};

/// Sunucu Saati
struct ServerTime {
    int day         = 0;
    int hour        = 0;
    int minute      = 0;
    int second      = 0;
    int millisecond = 0;
};

/// Rakip İHA Bilgisi (Sunucudan gelen)
struct CompetitorUAV {
    int    team_id     = 0;
    GeoPoint position;
    double heading     = 0.0;
    double speed       = 0.0;
    bool   is_locking  = false;
};

/// Hava Savunma Sistemi (No-Fly Zone)
struct NoFlyZone {
    double latitude  = 0.0;
    double longitude = 0.0;
    double radius_m  = 0.0;   // yarıçap (metre)
};

/// Yarışma Sınırı (Polygon)
struct ArenaBoundary {
    std::vector<GeoPoint> vertices;
    double max_altitude = 150.0;  // metre
};

// ─────────────────────────────────────────────────
//  Konfigürasyon Parametreleri
// ─────────────────────────────────────────────────

struct VisionConfig {
    // Kamera
    int    frame_width       = 640;
    int    frame_height      = 480;
    int    target_fps        = 30;
    std::string camera_topic = "/camera/image_raw";  // Gazebo
    std::string camera_device = "/dev/video0";         // Gerçek

    // YOLO
    std::string model_path   = "yolov8n.onnx";
    float  confidence_thresh = 0.45f;
    float  nms_thresh        = 0.50f;
    int    input_size        = 640;
    bool   use_tensorrt      = false;
    bool   use_fp16          = true;

    // Vuruş Alanı (Şartnameye göre)
    double strike_zone_h_pct = 0.50;  // yatay %50
    double strike_zone_v_pct = 0.80;  // dikey %80
    double min_target_pct    = 0.06;  // minimum %6 kaplama (güvenli sınır, şartname %5)

    // Kilitlenme
    double lockon_duration_s      = 4.0;     // 4 saniye
    double lockon_tolerance_s     = 1.0;     // 1 saniye esneme
    double frame_loss_tolerance   = 0.05;    // %5 kayıp kare toleransı
    int    lockon_confirm_count   = 4;       // sayaç ≥4

    // Overlay
    int    rect_thickness         = 3;       // piksel
    int    rect_color_r           = 255;     // kırmızı
    int    rect_color_g           = 0;
    int    rect_color_b           = 0;
};

struct FlightConfig {
    // Kalkış / İniş
    double takeoff_altitude     = 30.0;    // metre
    double cruise_altitude      = 60.0;    // metre
    double max_altitude         = 145.0;   // metre (sınırın altında)
    double min_altitude         = 15.0;    // metre
    double landing_speed        = 2.0;     // m/s

    // Hız
    double cruise_speed         = 25.0;    // m/s
    double max_speed            = 45.0;    // m/s
    double search_speed         = 20.0;    // m/s
    double approach_speed       = 15.0;    // m/s

    // PID Kazançları (yaw/pitch/roll)
    double yaw_kp = 1.2,   yaw_ki = 0.01,  yaw_kd = 0.3;
    double pitch_kp = 0.8, pitch_ki = 0.005, pitch_kd = 0.2;

    // MAVLink
    std::string mavlink_url     = "udp:127.0.0.1:14550";
    int    mavlink_sysid        = 1;
    int    mavlink_compid       = 191;
};

struct KamikazeConfig {
    double min_dive_altitude    = 100.0;   // metre (şartname)
    double climb_altitude       = 150.0;   // metre (güvenli)
    double safe_pullup_alt      = 55.0;    // metre
    double dive_angle_deg       = -45.0;   // derece
    double qr_size_m            = 2.0;     // QR kod boyutu (metre)
};

struct CommConfig {
    std::string server_ip       = "192.168.1.100";
    int    server_port          = 8080;
    int    team_id              = 1;
    double send_rate_hz         = 2.0;     // 1-5 Hz arası
    double signal_loss_timeout  = 10.0;    // saniye
    std::string ftp_server      = "";
    std::string ftp_user        = "";
    std::string ftp_pass        = "";
};

struct SafetyConfig {
    double boundary_margin_m    = 50.0;    // sınıra minimum mesafe
    double altitude_margin_m    = 5.0;     // irtifa sınırına minimum pay
    double signal_loss_timeout  = 10.0;    // haberleşme kopma süresi
    double rtl_battery_pct      = 15.0;    // RTL batarya eşiği
    double emergency_battery    = 5.0;     // acil iniş batarya eşiği
    double collision_radius_m   = 20.0;    // çarpışma kaçınma yarıçapı
};

struct RecordingConfig {
    std::string output_dir      = "/tmp/siha_video/";
    int    video_fps            = 30;
    std::string codec           = "libx264";  // H264
    std::string format          = "mp4";
    int    video_width          = 640;
    int    video_height         = 480;
};

/// Ana Konfigürasyon (tüm alt yapıları barındırır)
struct SystemConfig {
    MissionType    mission_type  = MissionType::SAVASAN_IHA;
    VisionConfig   vision;
    FlightConfig   flight;
    KamikazeConfig kamikaze;
    CommConfig     comm;
    SafetyConfig   safety;
    RecordingConfig recording;
    bool           simulation_mode = true;   // Gazebo mu gerçek mi
};

}  // namespace siha
