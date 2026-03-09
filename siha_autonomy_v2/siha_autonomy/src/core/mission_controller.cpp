/**
 * @file mission_controller.cpp
 * @brief Ana Görev Yöneticisi — Tam ROS2 entegrasyonlu implementasyon
 */

#include "siha_autonomy/core/mission_controller.hpp"
#include <chrono>
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;

namespace siha {

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
    flight_ctrl_     = std::make_unique<FlightController>(cfg.flight, this);  // this = ROS2 node
    guidance_        = std::make_unique<GuidanceSystem>(cfg.flight, cfg.vision);
    kamikaze_        = std::make_unique<KamikazeModule>(cfg.kamikaze, cfg.vision);
    evasion_         = std::make_unique<EvasionModule>(cfg.safety);
    server_comm_     = std::make_unique<ServerComm>(cfg.comm);
    telemetry_mgr_   = std::make_unique<TelemetryManager>();
    safety_monitor_  = std::make_unique<SafetyMonitor>(cfg.safety);
    video_recorder_  = std::make_unique<VideoRecorder>(cfg.recording);
    overlay_         = std::make_unique<OverlayRenderer>(cfg.vision.rect_thickness);

    // ── YOLO Model Yükle ──
    if (yolo_detector_->load_model()) {
        RCLCPP_INFO(get_logger(), "[YOLO] Model yüklendi: %s", cfg.vision.model_path.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "[YOLO] Model yüklenemedi: %s", cfg.vision.model_path.c_str());
    }

    // ── Gazebo Kamera Aboneliği ──
    sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
        cfg.vision.camera_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_camera_image(msg); });
    RCLCPP_INFO(get_logger(), "[Kamera] Abone olundu: %s", cfg.vision.camera_topic.c_str());

    // ── NPC Telemetri Aboneliği ──
    sub_npc_ = create_subscription<std_msgs::msg::String>(
        "/sunucu_telemetri", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_npc_telemetry(msg); });
    RCLCPP_INFO(get_logger(), "[NPC] Abone olundu: /sunucu_telemetri");

    // ── Sunucu bağlantısı ──
    server_comm_->connect();

    // ── Ana döngü (20 Hz) ──
    main_timer_ = create_wall_timer(50ms, [this]() { tick(); });

    // ── Telemetri gönderme (2 Hz) ──
    telemetry_timer_ = create_wall_timer(500ms, [this]() { send_telemetry(); });

    phase_entry_time_ = std::chrono::steady_clock::now();
    camera_fps_start_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "═══ SİSTEM HAZIR. Faz: IDLE ═══");
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

        // FPS ölçüm
        camera_frame_count_++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - camera_fps_start_).count();
        if (elapsed >= 2.0) {
            camera_fps_ = camera_frame_count_ / elapsed;
            camera_frame_count_ = 0;
            camera_fps_start_ = now;
            RCLCPP_DEBUG(get_logger(), "[Kamera] FPS: %.1f", camera_fps_);
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Kamera] cv_bridge hatası: %s", e.what());
    }
}

void MissionController::on_npc_telemetry(const std_msgs::msg::String::SharedPtr msg) {
    try {
        // Basit JSON parse (NPC verisini CompetitorUAV'a dönüştür)
        auto& s = msg->data;

        // sunucusaati ve konumBilgileri'ni bul
        std::vector<CompetitorUAV> competitors;

        // konumBilgileri dizisini parse et (basitleştirilmiş)
        size_t kb_pos = s.find("konumBilgileri");
        if (kb_pos == std::string::npos) return;

        size_t arr_start = s.find('[', kb_pos);
        if (arr_start == std::string::npos) return;

        // Her {...} bloğunu bul
        size_t pos = arr_start;
        while (true) {
            size_t obj_start = s.find('{', pos);
            size_t arr_end = s.find(']', pos);
            if (obj_start == std::string::npos || (arr_end != std::string::npos && obj_start > arr_end))
                break;
            size_t obj_end = s.find('}', obj_start);
            if (obj_end == std::string::npos) break;

            std::string obj = s.substr(obj_start, obj_end - obj_start + 1);

            auto get_num = [&](const std::string& key) -> double {
                auto p = obj.find("\"" + key + "\"");
                if (p == std::string::npos) return 0.0;
                p = obj.find(':', p);
                if (p == std::string::npos) return 0.0;
                return std::stod(obj.substr(p + 1));
            };

            CompetitorUAV comp;
            comp.team_id = static_cast<int>(get_num("takim_numarasi"));
            comp.position.latitude  = get_num("iha_enlem");
            comp.position.longitude = get_num("iha_boylam");
            comp.position.altitude  = get_num("iha_irtifa");
            comp.heading = get_num("iha_yonelme");
            comp.speed   = get_num("iha_hiz");
            competitors.push_back(comp);

            pos = obj_end + 1;
        }

        if (!competitors.empty()) {
            telemetry_mgr_->update_competitors(competitors);
        }
    } catch (...) {
        // Parse hatası
    }
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
    RCLCPP_INFO(get_logger(), "Faz geçişi: %d → %d",
                static_cast<int>(phase_), static_cast<int>(new_phase));
    phase_ = new_phase;
    phase_entry_time_ = std::chrono::steady_clock::now();

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
//  Faz İşleyicileri
// ═══════════════════════════════════════════════

void MissionController::handle_idle() {
    if (!flight_ctrl_->is_connected()) {
        flight_ctrl_->connect();
        return;
    }
    transition_to(FlightPhase::PRE_ARM);
}

void MissionController::handle_pre_arm() {
    auto telem = flight_ctrl_->get_telemetry();
    bool gps_ok  = flight_ctrl_->gps_fix_type() >= 3;
    bool batt_ok = flight_ctrl_->battery_percent() > 20;
    bool telem_ok = (telem.position.latitude != 0.0);  // telemetri geliyor mu

    if (!telem_ok) {
        // Henüz telemetri gelmemiş — bekle
        return;
    }

    if (gps_ok && batt_ok) {
        RCLCPP_INFO(get_logger(), "Pre-arm OK. GPS:%d Bat:%d%% Lat:%.5f",
                    flight_ctrl_->gps_fix_type(), flight_ctrl_->battery_percent(),
                    telem.position.latitude);
        flight_ctrl_->arm();
        transition_to(FlightPhase::ARMED);
    }
}

void MissionController::handle_armed() {
    if (flight_ctrl_->is_armed()) {
        flight_ctrl_->set_mode(ArduMode::GUIDED);
        transition_to(FlightPhase::TAKEOFF);
    }
}

void MissionController::handle_takeoff() {
    auto elapsed = std::chrono::steady_clock::now() - phase_entry_time_;
    if (elapsed < 1s) {
        flight_ctrl_->takeoff(config_.flight.takeoff_altitude);
        return;
    }

    auto telem = flight_ctrl_->get_telemetry();
    if (telem.position.altitude >= config_.flight.takeoff_altitude * 0.85) {
        RCLCPP_INFO(get_logger(), "Kalkış tamamlandı. İrtifa: %.1fm", telem.position.altitude);
        transition_to(FlightPhase::CLIMB);
    }
}

void MissionController::handle_climb() {
    auto telem = flight_ctrl_->get_telemetry();
    if (telem.position.altitude < config_.flight.cruise_altitude * 0.90) {
        flight_ctrl_->set_velocity_ned(0, 0, -3.0);
    } else {
        transition_to(FlightPhase::MISSION_SELECT);
    }
}

void MissionController::handle_mission_select() {
    if (mission_type_ == MissionType::KAMIKAZE) {
        transition_to(FlightPhase::KAMIKAZE_CLIMB);
    } else {
        transition_to(FlightPhase::SEARCH);
    }
}

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
    // Arama uçuşu — düz ilerle
    auto telem = flight_ctrl_->get_telemetry();
    flight_ctrl_->set_heading_and_speed(
        telem.heading, config_.flight.search_speed, config_.flight.cruise_altitude);
}

void MissionController::handle_track() {
    auto targets = target_tracker_->active_targets();
    auto telem = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        RCLCPP_WARN(get_logger(), "Hedef kayboldu → SEARCH");
        transition_to(FlightPhase::SEARCH);
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) { transition_to(FlightPhase::SEARCH); return; }

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

    if (info.lock_counter >= config_.vision.lockon_confirm_count) {
        RCLCPP_INFO(get_logger(), "══ KİLİTLENME BAŞARILI! Hedef:%d Süre:%.2fs ══",
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
}

void MissionController::handle_evade() {
    auto telem = flight_ctrl_->get_telemetry();
    auto comps = telemetry_mgr_->competitors();
    auto nfz   = telemetry_mgr_->nfz_zones();
    auto threat = evasion_->evaluate(telem, comps, nfz);

    if (!evasion_->evasion_needed()) {
        transition_to(FlightPhase::SEARCH);
        return;
    }
    auto cmd = guidance_->compute_evasion(threat.bearing_deg, threat.distance_m, telem);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
}

void MissionController::handle_kamikaze_climb() {
    auto telem = flight_ctrl_->get_telemetry();
    if (telem.position.altitude >= config_.kamikaze.min_dive_altitude) {
        transition_to(FlightPhase::KAMIKAZE_ALIGN);
    } else {
        flight_ctrl_->set_velocity_ned(0, 0, -4.0);
    }
}

void MissionController::handle_kamikaze_align() {
    auto telem = flight_ctrl_->get_telemetry();
    auto target = kamikaze_->dive_target();
    auto cmd = guidance_->compute_waypoint(target, telem, config_.flight.approach_speed);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
    double dist = GuidanceSystem::distance_between(telem.position, target);
    if (dist < 20.0) transition_to(FlightPhase::KAMIKAZE_DIVE);
}

void MissionController::handle_kamikaze_dive() {
    auto telem = flight_ctrl_->get_telemetry();
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!latest_camera_frame_.empty()) frame = latest_camera_frame_.clone();
    }
    auto phase = kamikaze_->update(telem, frame);
    if (kamikaze_->qr_read_success()) {
        RCLCPP_INFO(get_logger(), "QR OKUNDU: %s", kamikaze_->qr_content().c_str());
        server_comm_->send_qr_data(kamikaze_->qr_content());
    }
    if (telem.position.altitude <= config_.kamikaze.safe_pullup_alt) {
        transition_to(FlightPhase::KAMIKAZE_PULLUP);
    }
    flight_ctrl_->set_velocity_ned(
        telem.speed * std::cos(telem.heading * M_PI / 180.0),
        telem.speed * std::sin(telem.heading * M_PI / 180.0), 8.0);
}

void MissionController::handle_kamikaze_pullup() {
    auto telem = flight_ctrl_->get_telemetry();
    flight_ctrl_->set_velocity_ned(0, 0, -5.0);
    if (telem.position.altitude >= config_.flight.cruise_altitude * 0.8) {
        kamikaze_->reset();
        mission_type_ = MissionType::SAVASAN_IHA;
        transition_to(FlightPhase::SEARCH);
    }
}

void MissionController::handle_loiter()  { flight_ctrl_->set_mode(ArduMode::LOITER); }
void MissionController::handle_rtl()     {
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.altitude < 5.0 && t.speed < 1.0) transition_to(FlightPhase::LAND);
}
void MissionController::handle_land()    {
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.altitude < 0.5) {
        flight_ctrl_->disarm();
        video_recorder_->stop();
        transition_to(FlightPhase::IDLE);
    }
}

void MissionController::handle_boundary_return() {
    auto telem = flight_ctrl_->get_telemetry();
    auto boundary = telemetry_mgr_->boundary();
    auto safe = safety_monitor_->safe_return_point(telem.position, boundary);
    auto cmd = guidance_->compute_waypoint(safe, telem, config_.flight.cruise_speed);
    if (cmd.is_valid) flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    if (safety_monitor_->is_in_boundary(telem.position, boundary)) transition_to(FlightPhase::SEARCH);
}

void MissionController::handle_signal_lost() {
    flight_ctrl_->set_mode(ArduMode::LOITER);
    if (server_comm_->is_connected() && !server_comm_->is_signal_lost()) {
        transition_to(FlightPhase::SEARCH);
    }
    auto elapsed = std::chrono::steady_clock::now() - phase_entry_time_;
    if (elapsed > 30s) transition_to(FlightPhase::RTL);
}

void MissionController::handle_emergency() {
    flight_ctrl_->set_mode(ArduMode::QLAND);
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

    // YOLO inference
    auto detections = yolo_detector_->detect(frame);

    // Tracker güncelle
    auto tracked = target_tracker_->update(detections);

    if (!tracked.empty()) {
        auto best = target_tracker_->select_best_target(blacklisted_targets_);
        if (best.track_id >= 0) {
            lockon_manager_->process(best,
                config_.vision.frame_width, config_.vision.frame_height);
        }
    }
}

void MissionController::check_safety_conditions() {
    auto telem = flight_ctrl_->get_telemetry();
    telemetry_mgr_->update_own(telem);
    own_telemetry_ = telem;

    auto boundary = telemetry_mgr_->boundary();
    auto comps = telemetry_mgr_->competitors();
    auto nfz = telemetry_mgr_->nfz_zones();

    auto alert = safety_monitor_->check(telem, boundary, comps, nfz, server_comm_->is_connected());
    if (alert.type == SafetyAlert::Type::NONE) return;

    switch (alert.type) {
        case SafetyAlert::Type::BOUNDARY_VIOLATION:
            if (phase_ != FlightPhase::BOUNDARY_RETURN && phase_ != FlightPhase::RTL &&
                phase_ != FlightPhase::LAND && phase_ != FlightPhase::EMERGENCY) {
                RCLCPP_WARN(get_logger(), "⚠ SINIR AŞIMI!");
                transition_to(FlightPhase::BOUNDARY_RETURN);
            }
            break;
        case SafetyAlert::Type::ALTITUDE_VIOLATION:
            RCLCPP_WARN(get_logger(), "⚠ İRTİFA LİMİTİ!");
            flight_ctrl_->set_velocity_ned(0, 0, 2.0);
            break;
        case SafetyAlert::Type::SIGNAL_LOST:
            if (phase_ != FlightPhase::SIGNAL_LOST && phase_ != FlightPhase::RTL) {
                RCLCPP_WARN(get_logger(), "⚠ HABERLEŞME KOPTU!");
                transition_to(FlightPhase::SIGNAL_LOST);
            }
            break;
        case SafetyAlert::Type::CRITICAL_BATTERY:
            RCLCPP_ERROR(get_logger(), "🔴 KRİTİK BATARYA!");
            transition_to(FlightPhase::EMERGENCY);
            break;
        case SafetyAlert::Type::LOW_BATTERY:
            if (phase_ != FlightPhase::RTL && phase_ != FlightPhase::LAND) {
                transition_to(FlightPhase::RTL);
            }
            break;
        case SafetyAlert::Type::NFZ_INTRUSION:
            transition_to(FlightPhase::EVADE);
            break;
        case SafetyAlert::Type::COLLISION_RISK:
            if (phase_ != FlightPhase::EVADE) transition_to(FlightPhase::EVADE);
            break;
        default: break;
    }
}

void MissionController::send_telemetry() {
    auto telem = telemetry_mgr_->own();
    server_comm_->send_telemetry(telem);
}

void MissionController::record_frame() {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (latest_camera_frame_.empty()) return;
        frame = latest_camera_frame_.clone();
    }

    auto st = telemetry_mgr_->server_time();
    overlay_->draw_server_time(frame, st);

    auto lock_info = lockon_manager_->current_info();
    if (lock_info.lock_duration_s > 0) {
        overlay_->draw_lockon_rect(frame, lock_info.lock_rect);
    }

    overlay_->draw_fps(frame, camera_fps_);

    if (video_recorder_->is_recording()) {
        video_recorder_->write_frame(frame);
    }
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
