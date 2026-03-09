/**
 * @file mission_controller.cpp
 * @brief Ana Görev Yöneticisi — Düzeltilmiş implementasyon
 *
 * Düzeltmeler:
 *   - Komutlar rate-limited (spam yok)
 *   - Her fazda timeout kontrolü
 *   - Arm başarısızlığında kullanıcı komutu bekleme
 *   - Telemetri doğrulaması (gerçek veri gelene kadar ilerleme)
 *   - /mission/command topic'i ile dışarıdan kontrol
 */

#include "siha_autonomy/core/mission_controller.hpp"
#include <chrono>
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace siha {

// ═══════════════════════════════════════════════
//  Yardımcı: Belirli aralıkla komut gönderme
// ═══════════════════════════════════════════════

static double seconds_since(const Clock::time_point& t) {
    return std::chrono::duration<double>(Clock::now() - t).count();
}

// ═══════════════════════════════════════════════
//  Constructor
// ═══════════════════════════════════════════════

MissionController::MissionController(const SystemConfig& cfg)
    : Node("mission_controller"), config_(cfg)
{
    RCLCPP_INFO(get_logger(), "═══ SAEROTECH OTONOM SİSTEM BAŞLATILIYOR ═══");

    // ── Alt modüller ──
    yolo_detector_   = std::make_unique<YoloDetector>(cfg.vision);
    target_tracker_  = std::make_unique<TargetTracker>(cfg.vision);
    lockon_manager_  = std::make_unique<LockonManager>(cfg.vision);
    flight_ctrl_     = std::make_unique<FlightController>(cfg.flight, this);
    guidance_        = std::make_unique<GuidanceSystem>(cfg.flight, cfg.vision);
    kamikaze_        = std::make_unique<KamikazeModule>(cfg.kamikaze, cfg.vision);
    evasion_         = std::make_unique<EvasionModule>(cfg.safety);
    server_comm_     = std::make_unique<ServerComm>(cfg.comm);
    telemetry_mgr_   = std::make_unique<TelemetryManager>();
    safety_monitor_  = std::make_unique<SafetyMonitor>(cfg.safety);
    video_recorder_  = std::make_unique<VideoRecorder>(cfg.recording);
    overlay_         = std::make_unique<OverlayRenderer>(cfg.vision.rect_thickness);

    // ── YOLO ──
    if (yolo_detector_->load_model()) {
        RCLCPP_INFO(get_logger(), "[YOLO] Model yüklendi: %s", cfg.vision.model_path.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "[YOLO] Model yüklenemedi: %s", cfg.vision.model_path.c_str());
    }

    // ── Gazebo Kamera Aboneliği ──
    sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
        cfg.vision.camera_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_camera_image(msg); });
    RCLCPP_INFO(get_logger(), "[Kamera] Abone: %s", cfg.vision.camera_topic.c_str());

    // ── NPC Telemetri Aboneliği ──
    sub_npc_ = create_subscription<std_msgs::msg::String>(
        "/sunucu_telemetri", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_npc_telemetry(msg); });
    RCLCPP_INFO(get_logger(), "[NPC] Abone: /sunucu_telemetri");

    // ── Kullanıcı Komut Aboneliği ──
    sub_user_cmd_ = create_subscription<std_msgs::msg::String>(
        "/mission/command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_user_command(msg); });
    RCLCPP_INFO(get_logger(), "[Komut] Dinleniyor: /mission/command");

    // ── Durum Yayıncısı ──
    pub_status_ = create_publisher<std_msgs::msg::String>("/mission/status", 10);

    // ── Sunucu ──
    server_comm_->connect();

    // ── Ana döngü (20 Hz) ──
    main_timer_ = create_wall_timer(50ms, [this]() { tick(); });

    // ── Telemetri gönderme (2 Hz) ──
    telemetry_timer_ = create_wall_timer(500ms, [this]() { send_telemetry(); });

    // ── Durum yayını (1 Hz) ──
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });

    phase_entry_time_ = Clock::now();
    last_cmd_time_ = Clock::now();
    camera_fps_start_ = Clock::now();

    RCLCPP_INFO(get_logger(), "═══ SİSTEM HAZIR. Faz: IDLE ═══");
    RCLCPP_INFO(get_logger(), "Komutlar: ros2 topic pub /mission/command std_msgs/String \"data: 'arm'\"");
}

MissionController::~MissionController() {
    video_recorder_->stop();
    server_comm_->disconnect();
    flight_ctrl_->disconnect();
}

// ═══════════════════════════════════════════════
//  ROS2 Callback'ler
// ═══════════════════════════════════════════════

void MissionController::on_camera_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::lock_guard<std::mutex> lock(camera_mutex_);
        latest_camera_frame_ = cv_ptr->image;
        has_new_frame_ = true;

        camera_frame_count_++;
        auto now = Clock::now();
        double elapsed = seconds_since(camera_fps_start_);
        if (elapsed >= 2.0) {
            camera_fps_ = camera_frame_count_ / elapsed;
            camera_frame_count_ = 0;
            camera_fps_start_ = now;
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Kamera] cv_bridge hatası: %s", e.what());
    }
}

void MissionController::on_npc_telemetry(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto& s = msg->data;
        std::vector<CompetitorUAV> competitors;

        size_t kb_pos = s.find("konumBilgileri");
        if (kb_pos == std::string::npos) return;
        size_t arr_start = s.find('[', kb_pos);
        if (arr_start == std::string::npos) return;

        size_t pos = arr_start;
        while (true) {
            size_t obj_start = s.find('{', pos);
            size_t arr_end = s.find(']', pos);
            if (obj_start == std::string::npos || 
                (arr_end != std::string::npos && obj_start > arr_end)) break;
            size_t obj_end = s.find('}', obj_start);
            if (obj_end == std::string::npos) break;

            std::string obj = s.substr(obj_start, obj_end - obj_start + 1);
            auto get_num = [&](const std::string& key) -> double {
                auto p = obj.find("\"" + key + "\"");
                if (p == std::string::npos) return 0.0;
                p = obj.find(':', p);
                if (p == std::string::npos) return 0.0;
                try { return std::stod(obj.substr(p + 1)); }
                catch (...) { return 0.0; }
            };

            CompetitorUAV comp;
            comp.team_id = static_cast<int>(get_num("takim_numarasi"));
            comp.position.latitude  = get_num("iha_enlem");
            comp.position.longitude = get_num("iha_boylam");
            comp.position.altitude  = get_num("iha_irtifa");
            comp.heading = get_num("iha_yonelme");
            comp.speed   = get_num("iha_hiz");
            if (comp.team_id > 0) competitors.push_back(comp);
            pos = obj_end + 1;
        }

        if (!competitors.empty()) {
            telemetry_mgr_->update_competitors(competitors);
        }
    } catch (...) {}
}

void MissionController::on_user_command(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;
    RCLCPP_INFO(get_logger(), "Kullanıcı komutu: '%s'", cmd.c_str());

    if (cmd == "arm" || cmd == "ARM") {
        user_arm_requested_ = true;
    } else if (cmd == "disarm" || cmd == "DISARM") {
        flight_ctrl_->disarm();
        transition_to(FlightPhase::IDLE);
    } else if (cmd == "takeoff") {
        if (phase_ == FlightPhase::IDLE || phase_ == FlightPhase::PRE_ARM) {
            user_arm_requested_ = true;
        }
    } else if (cmd == "rtl" || cmd == "RTL") {
        transition_to(FlightPhase::RTL);
    } else if (cmd == "land" || cmd == "LAND") {
        transition_to(FlightPhase::LAND);
    } else if (cmd == "abort" || cmd == "ABORT") {
        transition_to(FlightPhase::EMERGENCY);
    } else if (cmd == "search") {
        if (phase_ == FlightPhase::LOITER || phase_ == FlightPhase::CLIMB) {
            transition_to(FlightPhase::SEARCH);
        }
    } else if (cmd == "loiter") {
        transition_to(FlightPhase::LOITER);
    } else if (cmd == "kamikaze") {
        mission_type_ = MissionType::KAMIKAZE;
        transition_to(FlightPhase::KAMIKAZE_CLIMB);
    } else if (cmd == "status") {
        publish_status();
    } else {
        RCLCPP_WARN(get_logger(), "Bilinmeyen komut: '%s'", cmd.c_str());
        RCLCPP_INFO(get_logger(), "Geçerli komutlar: arm, disarm, takeoff, rtl, land, abort, search, loiter, kamikaze, status");
    }
}

void MissionController::publish_status() {
    static const char* phase_names[] = {
        "IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","MISSION_SELECT",
        "SEARCH","TRACK","LOCKON","EVADE","KAMIKAZE_CLIMB","KAMIKAZE_ALIGN",
        "KAMIKAZE_DIVE","KAMIKAZE_PULLUP","LOITER","RTL","LAND",
        "BOUNDARY_RETURN","SIGNAL_LOST","EMERGENCY"
    };

    auto telem = flight_ctrl_->get_telemetry();
    int phase_idx = static_cast<int>(phase_);
    const char* phase_name = (phase_idx >= 0 && phase_idx < 20) ? phase_names[phase_idx] : "?";

    std::ostringstream oss;
    oss << "{"
        << "\"phase\":\"" << phase_name << "\""
        << ",\"armed\":" << (flight_ctrl_->is_armed() ? "true" : "false")
        << ",\"lat\":" << telem.position.latitude
        << ",\"lon\":" << telem.position.longitude
        << ",\"alt\":" << telem.position.altitude
        << ",\"hdg\":" << telem.heading
        << ",\"spd\":" << telem.speed
        << ",\"bat\":" << telem.battery
        << ",\"gps\":" << flight_ctrl_->gps_fix_type()
        << ",\"cam_fps\":" << camera_fps_
        << ",\"lockons\":" << total_lockons_
        << ",\"phase_time\":" << seconds_since(phase_entry_time_)
        << "}";

    auto msg = std_msgs::msg::String();
    msg.data = oss.str();
    pub_status_->publish(msg);

    // Periyodik log (5 saniyede bir)
    static int log_counter = 0;
    if (++log_counter % 5 == 0) {
        RCLCPP_INFO(get_logger(), "[%s] Alt:%.1fm Hdg:%.0f Spd:%.1f Bat:%d%% GPS:%d Cam:%.0fFPS",
            phase_name, telem.position.altitude, telem.heading,
            telem.speed, telem.battery, flight_ctrl_->gps_fix_type(), camera_fps_);
    }
}

// ═══════════════════════════════════════════════
//  Komut gönderme (rate-limited)
// ═══════════════════════════════════════════════

bool MissionController::send_cmd_throttled(double interval_s) {
    if (seconds_since(last_cmd_time_) < interval_s) return false;
    last_cmd_time_ = Clock::now();
    return true;
}

// ═══════════════════════════════════════════════
//  Ana Döngü
// ═══════════════════════════════════════════════

void MissionController::tick() {
    check_safety_conditions();
    process_vision_frame();
    record_frame();

    switch (phase_) {
        case FlightPhase::IDLE:              handle_idle();              break;
        case FlightPhase::PRE_ARM:           handle_pre_arm();           break;
        case FlightPhase::ARMED:             handle_armed();             break;
        case FlightPhase::TAKEOFF:           handle_takeoff();           break;
        case FlightPhase::CLIMB:             handle_climb();             break;
        case FlightPhase::MISSION_SELECT:    handle_mission_select();    break;
        case FlightPhase::SEARCH:            handle_search();            break;
        case FlightPhase::TRACK:             handle_track();             break;
        case FlightPhase::LOCKON:            handle_lockon();            break;
        case FlightPhase::EVADE:             handle_evade();             break;
        case FlightPhase::KAMIKAZE_CLIMB:    handle_kamikaze_climb();    break;
        case FlightPhase::KAMIKAZE_ALIGN:    handle_kamikaze_align();    break;
        case FlightPhase::KAMIKAZE_DIVE:     handle_kamikaze_dive();     break;
        case FlightPhase::KAMIKAZE_PULLUP:   handle_kamikaze_pullup();   break;
        case FlightPhase::LOITER:            handle_loiter();            break;
        case FlightPhase::RTL:               handle_rtl();               break;
        case FlightPhase::LAND:              handle_land();              break;
        case FlightPhase::BOUNDARY_RETURN:   handle_boundary_return();   break;
        case FlightPhase::SIGNAL_LOST:       handle_signal_lost();       break;
        case FlightPhase::EMERGENCY:         handle_emergency();         break;
    }
}

// ═══════════════════════════════════════════════
//  Durum Geçişi
// ═══════════════════════════════════════════════

void MissionController::transition_to(FlightPhase new_phase) {
    if (phase_ == new_phase) return;

    static const char* names[] = {
        "IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","MISSION_SELECT",
        "SEARCH","TRACK","LOCKON","EVADE","KMZ_CLIMB","KMZ_ALIGN",
        "KMZ_DIVE","KMZ_PULLUP","LOITER","RTL","LAND",
        "BOUNDARY_RET","SIGNAL_LOST","EMERGENCY"
    };
    int fi = static_cast<int>(phase_), ti = static_cast<int>(new_phase);
    RCLCPP_INFO(get_logger(), "══ Faz: %s → %s ══",
        (fi >= 0 && fi < 20) ? names[fi] : "?",
        (ti >= 0 && ti < 20) ? names[ti] : "?");

    phase_ = new_phase;
    phase_entry_time_ = Clock::now();
    cmd_sent_in_phase_ = false;  // yeni fazda henüz komut gönderilmedi
    retry_count_ = 0;

    switch (new_phase) {
        case FlightPhase::SEARCH:
        case FlightPhase::TRACK:
            guidance_->reset();
            break;
        case FlightPhase::LOCKON:
            if (!video_recorder_->is_recording()) video_recorder_->start();
            break;
        case FlightPhase::RTL:
            flight_ctrl_->return_to_launch();
            break;
        case FlightPhase::LAND:
            flight_ctrl_->land();
            break;
        case FlightPhase::SIGNAL_LOST:
            flight_ctrl_->set_mode(ArduMode::LOITER);
            break;
        default: break;
    }
}

// ═══════════════════════════════════════════════
//  IDLE — Bağlantı bekle
// ═══════════════════════════════════════════════

void MissionController::handle_idle() {
    // Bağlantı kur (bir kere)
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->connect();
        cmd_sent_in_phase_ = true;
        RCLCPP_INFO(get_logger(), "mavlink_bridge bekleniyor...");
    }

    // Telemetri gelmeye başladı mı?
    auto telem = flight_ctrl_->get_telemetry();
    if (telem.position.latitude != 0.0) {
        RCLCPP_INFO(get_logger(), "Telemetri alınıyor. PRE_ARM'a geçiliyor.");
        transition_to(FlightPhase::PRE_ARM);
    }

    // 30 saniye telemetri gelmezse uyar
    if (seconds_since(phase_entry_time_) > 30.0 && !cmd_sent_in_phase_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "Telemetri yok! mavlink_bridge.py çalışıyor mu?");
    }
}

// ═══════════════════════════════════════════════
//  PRE_ARM — Sensör kontrolü + arm deneme
// ═══════════════════════════════════════════════

void MissionController::handle_pre_arm() {
    auto telem = flight_ctrl_->get_telemetry();
    int gps = flight_ctrl_->gps_fix_type();
    int bat = flight_ctrl_->battery_percent();

    // GPS hazır değilse bekle
    if (gps < 3) {
        if (send_cmd_throttled(5.0)) {
            RCLCPP_INFO(get_logger(), "GPS bekleniyor... (fix:%d, gerek:3+)", gps);
        }
        return;
    }

    // Batarya düşükse dur
    if (bat > 0 && bat < 15) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
            "Batarya çok düşük: %d%%. Arm yapılmayacak.", bat);
        return;
    }

    // Otomatik arm veya kullanıcı isteği
    if (user_arm_requested_) {
        user_arm_requested_ = false;
        RCLCPP_INFO(get_logger(), "ARM deneniyor... GPS:%d Bat:%d%% Lat:%.5f",
            gps, bat, telem.position.latitude);
        flight_ctrl_->set_mode(ArduMode::GUIDED);
        flight_ctrl_->arm();
        arm_attempt_time_ = Clock::now();
        retry_count_++;
        return;
    }

    // Arm denemesi yapıldıysa sonucunu kontrol et
    if (retry_count_ > 0) {
        double wait = seconds_since(arm_attempt_time_);

        // Arm başarılı mı?
        if (flight_ctrl_->is_armed()) {
            RCLCPP_INFO(get_logger(), "ARM BAŞARILI!");
            transition_to(FlightPhase::ARMED);
            return;
        }

        // 5 saniye içinde arm olmadıysa → başarısız
        if (wait > 5.0) {
            RCLCPP_WARN(get_logger(),
                "ARM başarısız (deneme %d). SITL'da 'arm throttle force' deneyin "
                "veya /mission/command'a 'arm' gönderin.", retry_count_);
            retry_count_ = 0;  // Yeni deneme için sıfırla
        }
        return;
    }

    // İlk girişte otomatik arm dene
    if (!cmd_sent_in_phase_) {
        cmd_sent_in_phase_ = true;
        user_arm_requested_ = true;  // İlk seferde otomatik dene
        RCLCPP_INFO(get_logger(), "Pre-arm kontrolleri OK. Otomatik arm deneniyor...");
    }

    // 20 saniyeden uzun süredir buradaysak ve arm olmadıysa kullanıcı bekliyoruz
    if (seconds_since(phase_entry_time_) > 20.0) {
        if (send_cmd_throttled(10.0)) {
            RCLCPP_INFO(get_logger(),
                "ARM bekleniyor. Komut: ros2 topic pub --once /mission/command "
                "std_msgs/String \"data: 'arm'\"");
        }
    }
}

// ═══════════════════════════════════════════════
//  ARMED — GUIDED moda geç, kalkışa hazırlan
// ═══════════════════════════════════════════════

void MissionController::handle_armed() {
    if (!flight_ctrl_->is_armed()) {
        // Arm kaybedildi
        RCLCPP_WARN(get_logger(), "Arm kaybedildi! PRE_ARM'a dönülüyor.");
        transition_to(FlightPhase::PRE_ARM);
        return;
    }

    // GUIDED moda geç (bir kere)
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->set_mode(ArduMode::GUIDED);
        cmd_sent_in_phase_ = true;
    }

    // 1 saniye bekle (mod geçişi için), sonra takeoff
    if (seconds_since(phase_entry_time_) > 1.0) {
        transition_to(FlightPhase::TAKEOFF);
    }
}

// ═══════════════════════════════════════════════
//  TAKEOFF — Kalkış komutu (BİR KERE) + irtifa takibi
// ═══════════════════════════════════════════════

void MissionController::handle_takeoff() {
    // Kalkış komutunu sadece BİR KERE gönder
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->takeoff(config_.flight.takeoff_altitude);
        cmd_sent_in_phase_ = true;
        RCLCPP_INFO(get_logger(), "Kalkış komutu gönderildi: %.0fm", config_.flight.takeoff_altitude);
        return;
    }

    auto telem = flight_ctrl_->get_telemetry();
    double target_alt = config_.flight.takeoff_altitude;
    double elapsed = seconds_since(phase_entry_time_);

    // İrtifa kontrolü (periyodik log)
    if (send_cmd_throttled(3.0)) {
        RCLCPP_INFO(get_logger(), "Tırmanma: %.1f / %.0fm (%.0f sn)",
            telem.position.altitude, target_alt, elapsed);
    }

    // Hedef irtifaya ulaştı mı?
    if (telem.position.altitude >= target_alt * 0.85) {
        RCLCPP_INFO(get_logger(), "Kalkış tamamlandı! İrtifa: %.1fm", telem.position.altitude);
        transition_to(FlightPhase::CLIMB);
        return;
    }

    // Timeout: 60 saniye içinde kalkamazsa → sorun var
    if (elapsed > 60.0) {
        RCLCPP_WARN(get_logger(), "Kalkış timeout (60sn)! İrtifa: %.1fm. Loiter'a geçiliyor.",
            telem.position.altitude);
        if (telem.position.altitude > 5.0) {
            transition_to(FlightPhase::LOITER);
        } else {
            RCLCPP_ERROR(get_logger(), "Yerden kalkamadı. ARM komutu bekleniyor.");
            transition_to(FlightPhase::PRE_ARM);
        }
    }

    // Arm kaybedildiyse geri dön
    if (!flight_ctrl_->is_armed() && elapsed > 3.0) {
        RCLCPP_WARN(get_logger(), "Kalkışta arm kaybedildi!");
        transition_to(FlightPhase::PRE_ARM);
    }
}

// ═══════════════════════════════════════════════
//  CLIMB — Operasyonel irtifaya tırman
// ═══════════════════════════════════════════════

void MissionController::handle_climb() {
    auto telem = flight_ctrl_->get_telemetry();
    double target = config_.flight.cruise_altitude;

    if (telem.position.altitude >= target * 0.90) {
        RCLCPP_INFO(get_logger(), "Cruise irtifası: %.1fm", telem.position.altitude);
        transition_to(FlightPhase::MISSION_SELECT);
        return;
    }

    // Tırmanma komutu (2 saniyede bir)
    if (send_cmd_throttled(2.0)) {
        flight_ctrl_->set_velocity_ned(0, 0, -3.0);
    }

    // Timeout
    if (seconds_since(phase_entry_time_) > 90.0) {
        RCLCPP_WARN(get_logger(), "Climb timeout. Mevcut irtifayla devam.");
        transition_to(FlightPhase::MISSION_SELECT);
    }
}

// ═══════════════════════════════════════════════
//  MISSION_SELECT
// ═══════════════════════════════════════════════

void MissionController::handle_mission_select() {
    RCLCPP_INFO(get_logger(), "Görev: %s",
        mission_type_ == MissionType::KAMIKAZE ? "KAMİKAZE" : "SAVAŞAN İHA");

    if (mission_type_ == MissionType::KAMIKAZE) {
        transition_to(FlightPhase::KAMIKAZE_CLIMB);
    } else {
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══════════════════════════════════════════════
//  SEARCH — Hedef ara
// ═══════════════════════════════════════════════

void MissionController::handle_search() {
    auto targets = target_tracker_->active_targets();
    if (!targets.empty()) {
        auto best = target_tracker_->select_best_target(blacklisted_targets_);
        if (best.track_id >= 0) {
            RCLCPP_INFO(get_logger(), "Hedef tespit! ID:%d conf:%.2f",
                        best.track_id, best.bbox.confidence);
            transition_to(FlightPhase::TRACK);
            return;
        }
    }

    // Arama uçuşu (2 saniyede bir komut)
    if (send_cmd_throttled(2.0)) {
        auto telem = flight_ctrl_->get_telemetry();
        flight_ctrl_->set_heading_and_speed(
            telem.heading, config_.flight.search_speed, config_.flight.cruise_altitude);
    }
}

// ═══════════════════════════════════════════════
//  TRACK — Hedef takip
// ═══════════════════════════════════════════════

void MissionController::handle_track() {
    auto targets = target_tracker_->active_targets();
    auto telem = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        // 3 saniye boyunca hedef yok → search'e dön
        if (seconds_since(phase_entry_time_) > 3.0 || 
            target_tracker_->active_count() == 0) {
            RCLCPP_WARN(get_logger(), "Hedef kayboldu → SEARCH");
            transition_to(FlightPhase::SEARCH);
        }
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) { transition_to(FlightPhase::SEARCH); return; }

    // Güdüm komutu (her tick güncelle — takip hassasiyeti gerek)
    auto cmd = guidance_->compute_tracking(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height, telem);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Kilitlenme kontrolü
    auto info = lockon_manager_->process(best,
        config_.vision.frame_width, config_.vision.frame_height);
    if (info.lock_duration_s > 0.5) {
        transition_to(FlightPhase::LOCKON);
    }
}

// ═══════════════════════════════════════════════
//  LOCKON — Kilitlenme
// ═══════════════════════════════════════════════

void MissionController::handle_lockon() {
    auto targets = target_tracker_->active_targets();
    auto telem = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        auto info = lockon_manager_->process_no_target();
        if (info.tolerance_counter > LockonManager::MAX_TOLERANCE) {
            RCLCPP_WARN(get_logger(), "Kilitlenme başarısız — tolerans aşıldı");
            lockon_manager_->reset();
            transition_to(FlightPhase::SEARCH);
        }
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) { transition_to(FlightPhase::SEARCH); return; }

    auto info = lockon_manager_->process(best,
        config_.vision.frame_width, config_.vision.frame_height);

    // Hedefi ortada tut
    auto cmd = guidance_->compute_tracking(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height, telem);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Periyodik log
    if (send_cmd_throttled(1.0)) {
        RCLCPP_INFO(get_logger(), "Kilitlenme: %.1f/4.0sn hedef:%d",
            info.lock_duration_s, best.track_id);
    }

    // Başarılı mı?
    if (info.lock_counter >= config_.vision.lockon_confirm_count) {
        RCLCPP_INFO(get_logger(),
            "══ KİLİTLENME BAŞARILI! Hedef:%d Süre:%.2fs ══",
            best.track_id, info.lock_duration_s);
        blacklist_target(best.track_id);
        lockon_manager_->confirm_lockon(best.track_id);
        total_lockons_++;

        LockonPacket pkt;
        pkt.team_id = config_.comm.team_id;
        pkt.lock_rect_x = info.lock_rect.x1;
        pkt.lock_rect_y = info.lock_rect.y1;
        pkt.lock_rect_w = info.lock_rect.width();
        pkt.lock_rect_h = info.lock_rect.height();
        server_comm_->send_lockon_packet(pkt);

        transition_to(FlightPhase::SEARCH);
    }

    // Timeout: 15 saniyede kilitlenme başarısızsa bırak
    if (seconds_since(phase_entry_time_) > 15.0) {
        RCLCPP_WARN(get_logger(), "Kilitlenme timeout (15sn)");
        lockon_manager_->reset();
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══════════════════════════════════════════════
//  EVADE — Kaçınma
// ═══════════════════════════════════════════════

void MissionController::handle_evade() {
    auto telem = flight_ctrl_->get_telemetry();
    auto comps = telemetry_mgr_->competitors();
    auto nfz   = telemetry_mgr_->nfz_zones();
    auto threat = evasion_->evaluate(telem, comps, nfz);

    if (!evasion_->evasion_needed()) {
        RCLCPP_INFO(get_logger(), "Tehdit geçti → SEARCH");
        transition_to(FlightPhase::SEARCH);
        return;
    }

    if (send_cmd_throttled(0.5)) {
        auto cmd = guidance_->compute_evasion(threat.bearing_deg, threat.distance_m, telem);
        if (cmd.is_valid) {
            flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
        }
    }

    // Timeout
    if (seconds_since(phase_entry_time_) > 30.0) {
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══════════════════════════════════════════════
//  KAMIKAZE fazları
// ═══════════════════════════════════════════════

void MissionController::handle_kamikaze_climb() {
    auto telem = flight_ctrl_->get_telemetry();
    if (telem.position.altitude >= config_.kamikaze.min_dive_altitude) {
        transition_to(FlightPhase::KAMIKAZE_ALIGN);
    } else if (send_cmd_throttled(2.0)) {
        flight_ctrl_->set_velocity_ned(0, 0, -4.0);
    }
}

void MissionController::handle_kamikaze_align() {
    auto telem = flight_ctrl_->get_telemetry();
    auto target = kamikaze_->dive_target();
    if (send_cmd_throttled(1.0)) {
        auto cmd = guidance_->compute_waypoint(target, telem, config_.flight.approach_speed);
        if (cmd.is_valid)
            flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
    double dist = GuidanceSystem::distance_between(telem.position, target);
    if (dist < 20.0) transition_to(FlightPhase::KAMIKAZE_DIVE);
}

void MissionController::handle_kamikaze_dive() {
    auto telem = flight_ctrl_->get_telemetry();
    cv::Mat frame;
    { std::lock_guard<std::mutex> lock(camera_mutex_);
      if (!latest_camera_frame_.empty()) frame = latest_camera_frame_.clone(); }
    kamikaze_->update(telem, frame);
    if (kamikaze_->qr_read_success()) {
        RCLCPP_INFO(get_logger(), "QR OKUNDU: %s", kamikaze_->qr_content().c_str());
        server_comm_->send_qr_data(kamikaze_->qr_content());
    }
    if (telem.position.altitude <= config_.kamikaze.safe_pullup_alt)
        transition_to(FlightPhase::KAMIKAZE_PULLUP);
    else if (send_cmd_throttled(0.5))
        flight_ctrl_->set_velocity_ned(
            telem.speed * std::cos(telem.heading * M_PI / 180.0),
            telem.speed * std::sin(telem.heading * M_PI / 180.0), 8.0);
}

void MissionController::handle_kamikaze_pullup() {
    auto telem = flight_ctrl_->get_telemetry();
    if (send_cmd_throttled(1.0)) flight_ctrl_->set_velocity_ned(0, 0, -5.0);
    if (telem.position.altitude >= config_.flight.cruise_altitude * 0.8) {
        kamikaze_->reset();
        mission_type_ = MissionType::SAVASAN_IHA;
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══════════════════════════════════════════════
//  LOITER / RTL / LAND / BOUNDARY / SIGNAL / EMERGENCY
// ═══════════════════════════════════════════════

void MissionController::handle_loiter() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->set_mode(ArduMode::LOITER);
        cmd_sent_in_phase_ = true;
        RCLCPP_INFO(get_logger(), "Loiter modu. Komut bekleniyor...");
    }
}

void MissionController::handle_rtl() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->return_to_launch();
        cmd_sent_in_phase_ = true;
    }
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.altitude < 5.0 && t.speed < 1.0)
        transition_to(FlightPhase::LAND);
}

void MissionController::handle_land() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->land();
        cmd_sent_in_phase_ = true;
    }
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.altitude < 0.5) {
        flight_ctrl_->disarm();
        video_recorder_->stop();
        RCLCPP_INFO(get_logger(), "İniş tamamlandı.");
        transition_to(FlightPhase::IDLE);
    }
}

void MissionController::handle_boundary_return() {
    auto telem = flight_ctrl_->get_telemetry();
    auto boundary = telemetry_mgr_->boundary();
    if (safety_monitor_->is_in_boundary(telem.position, boundary)) {
        transition_to(FlightPhase::SEARCH);
        return;
    }
    if (send_cmd_throttled(1.0)) {
        auto safe = safety_monitor_->safe_return_point(telem.position, boundary);
        auto cmd = guidance_->compute_waypoint(safe, telem, config_.flight.cruise_speed);
        if (cmd.is_valid)
            flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
}

void MissionController::handle_signal_lost() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->set_mode(ArduMode::LOITER);
        cmd_sent_in_phase_ = true;
        RCLCPP_WARN(get_logger(), "Haberleşme koptu! Çember atılıyor...");
    }
    if (server_comm_->is_connected() && !server_comm_->is_signal_lost()) {
        RCLCPP_INFO(get_logger(), "Haberleşme geri geldi!");
        transition_to(FlightPhase::SEARCH);
    }
    if (seconds_since(phase_entry_time_) > 30.0) {
        transition_to(FlightPhase::RTL);
    }
}

void MissionController::handle_emergency() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->set_mode(ArduMode::QLAND);
        cmd_sent_in_phase_ = true;
        RCLCPP_ERROR(get_logger(), "ACİL İNİŞ!");
    }
}

// ═══════════════════════════════════════════════
//  Görüntü İşleme
// ═══════════════════════════════════════════════

void MissionController::process_vision_frame() {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!has_new_frame_ || latest_camera_frame_.empty()) return;
        frame = latest_camera_frame_.clone();
        has_new_frame_ = false;
    }

    auto detections = yolo_detector_->detect(frame);
    target_tracker_->update(detections);
}

void MissionController::check_safety_conditions() {
    auto telem = flight_ctrl_->get_telemetry();
    telemetry_mgr_->update_own(telem);
    own_telemetry_ = telem;

    // Sadece uçuşta güvenlik kontrolü yap
    if (phase_ == FlightPhase::IDLE || phase_ == FlightPhase::PRE_ARM) return;

    auto boundary = telemetry_mgr_->boundary();
    auto comps = telemetry_mgr_->competitors();
    auto nfz = telemetry_mgr_->nfz_zones();
    auto alert = safety_monitor_->check(telem, boundary, comps, nfz,
                                         server_comm_->is_connected());
    if (alert.type == SafetyAlert::Type::NONE) return;

    switch (alert.type) {
        case SafetyAlert::Type::BOUNDARY_VIOLATION:
            if (phase_ != FlightPhase::BOUNDARY_RETURN && phase_ != FlightPhase::RTL &&
                phase_ != FlightPhase::LAND && phase_ != FlightPhase::EMERGENCY)
                transition_to(FlightPhase::BOUNDARY_RETURN);
            break;
        case SafetyAlert::Type::ALTITUDE_VIOLATION:
            if (send_cmd_throttled(2.0)) flight_ctrl_->set_velocity_ned(0, 0, 2.0);
            break;
        case SafetyAlert::Type::SIGNAL_LOST:
            if (phase_ != FlightPhase::SIGNAL_LOST && phase_ != FlightPhase::RTL)
                transition_to(FlightPhase::SIGNAL_LOST);
            break;
        case SafetyAlert::Type::CRITICAL_BATTERY:
            transition_to(FlightPhase::EMERGENCY);
            break;
        case SafetyAlert::Type::LOW_BATTERY:
            if (phase_ != FlightPhase::RTL && phase_ != FlightPhase::LAND)
                transition_to(FlightPhase::RTL);
            break;
        case SafetyAlert::Type::NFZ_INTRUSION:
        case SafetyAlert::Type::COLLISION_RISK:
            if (phase_ != FlightPhase::EVADE) transition_to(FlightPhase::EVADE);
            break;
        default: break;
    }
}

void MissionController::send_telemetry() {
    server_comm_->send_telemetry(telemetry_mgr_->own());
}

void MissionController::record_frame() {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (latest_camera_frame_.empty()) return;
        frame = latest_camera_frame_.clone();
    }
    overlay_->draw_server_time(frame, telemetry_mgr_->server_time());
    auto info = lockon_manager_->current_info();
    if (info.lock_duration_s > 0) overlay_->draw_lockon_rect(frame, info.lock_rect);
    overlay_->draw_fps(frame, camera_fps_);
    if (video_recorder_->is_recording()) video_recorder_->write_frame(frame);
}

void MissionController::set_mission_type(MissionType type) {
    mission_type_ = type;
    config_.mission_type = type;
}

bool MissionController::is_target_blacklisted(int id) {
    return blacklisted_targets_.count(id) > 0;
}

void MissionController::blacklist_target(int id) {
    blacklisted_targets_.insert(id);
    lockon_manager_->add_to_blacklist(id);
}

}  // namespace siha
