/**
 * @file mission_controller.cpp
 * @brief Ana Görev Yöneticisi implementasyonu
 */

#include "siha_autonomy/core/mission_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace siha {

// ─────────────────────────────────────────────────
//  Constructor / Destructor
// ─────────────────────────────────────────────────

MissionController::MissionController(const SystemConfig& cfg)
    : Node("mission_controller"), config_(cfg)
{
    RCLCPP_INFO(get_logger(), "=== SAEROTECH SİHA OTONOM SİSTEM BAŞLATILIYOR ===");

    // ── Alt Modüller Oluştur ──
    auto cam = create_camera_source(
        cfg.simulation_mode,
        cfg.vision.camera_topic,
        cfg.vision.camera_device,
        cfg.vision.frame_width,
        cfg.vision.frame_height,
        cfg.vision.target_fps);

    vision_pipeline_ = std::make_unique<VideoPipeline>(cfg.vision, cfg.simulation_mode);
    yolo_detector_   = std::make_unique<YoloDetector>(cfg.vision);
    target_tracker_  = std::make_unique<TargetTracker>(cfg.vision);
    lockon_manager_  = std::make_unique<LockonManager>(cfg.vision);
    flight_ctrl_     = std::make_unique<FlightController>(cfg.flight);
    guidance_        = std::make_unique<GuidanceSystem>(cfg.flight, cfg.vision);
    kamikaze_        = std::make_unique<KamikazeModule>(cfg.kamikaze, cfg.vision);
    evasion_         = std::make_unique<EvasionModule>(cfg.safety);
    server_comm_     = std::make_unique<ServerComm>(cfg.comm);
    telemetry_mgr_   = std::make_unique<TelemetryManager>();
    safety_monitor_  = std::make_unique<SafetyMonitor>(cfg.safety);
    video_recorder_  = std::make_unique<VideoRecorder>(cfg.recording);

    // ── YOLO Model Yükle ──
    if (!yolo_detector_->load_model()) {
        RCLCPP_ERROR(get_logger(), "YOLO modeli yüklenemedi!");
    }

    // ── Video Pipeline Başlat ──
    vision_pipeline_->add_frame_listener(
        [this](const cv::Mat& frame, double ts) {
            // Bu callback her frame'de çağrılır
            // Ancak ana işleme tick() içinde yapılır
        });
    vision_pipeline_->start(std::move(cam));

    // ── Sunucu Bağlantısı ──
    server_comm_->on_competitors_update(
        [this](const std::vector<CompetitorUAV>& comps) {
            telemetry_mgr_->update_competitors(comps);
        });
    server_comm_->on_nfz_update(
        [this](const std::vector<NoFlyZone>& zones) {
            telemetry_mgr_->update_nfz(zones);
        });

    // ── Heartbeat Kaybı Callback ──
    flight_ctrl_->on_heartbeat_lost([this]() {
        RCLCPP_WARN(get_logger(), "Otopilot heartbeat kaybedildi!");
        transition_to(FlightPhase::EMERGENCY);
    });

    // ── Ana Döngü Timer (50 Hz = 20ms) ──
    main_timer_ = create_wall_timer(20ms, [this]() { tick(); });

    // ── Telemetri Timer (2 Hz) ──
    telemetry_timer_ = create_wall_timer(500ms, [this]() { send_telemetry(); });

    phase_entry_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Sistem hazır. Faz: IDLE");
}

MissionController::~MissionController() {
    vision_pipeline_->stop();
    video_recorder_->stop();
    server_comm_->disconnect();
    flight_ctrl_->disconnect();
}

// ─────────────────────────────────────────────────
//  Ana Tick Döngüsü
// ─────────────────────────────────────────────────

void MissionController::tick() {
    // 1) Güvenlik kontrolleri (her zaman, en yüksek öncelik)
    check_safety_conditions();

    // 2) Görüntü işleme (frame al, YOLO çalıştır, tracker güncelle)
    process_vision_frame();

    // 3) Video kayıt
    record_frame();

    // 4) Mevcut faza göre işlem yap
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

// ─────────────────────────────────────────────────
//  Durum Geçişleri
// ─────────────────────────────────────────────────

void MissionController::transition_to(FlightPhase new_phase) {
    if (phase_ == new_phase) return;

    RCLCPP_INFO(get_logger(), "Faz geçişi: %d → %d",
                static_cast<int>(phase_), static_cast<int>(new_phase));

    phase_ = new_phase;
    phase_entry_time_ = std::chrono::steady_clock::now();

    // Faz geçişi yan etkileri
    switch (new_phase) {
        case FlightPhase::SEARCH:
            guidance_->reset();
            break;
        case FlightPhase::TRACK:
            guidance_->reset();
            break;
        case FlightPhase::LOCKON:
            // Kilitlenme başladığında video kaydı aktif olmalı
            if (!video_recorder_->is_recording()) {
                video_recorder_->start();
            }
            break;
        case FlightPhase::RTL:
            flight_ctrl_->return_to_launch();
            break;
        case FlightPhase::LAND:
            flight_ctrl_->land();
            break;
        case FlightPhase::SIGNAL_LOST:
            // Haberleşme kopmasında çember at
            flight_ctrl_->set_mode(ArduMode::LOITER);
            break;
        default:
            break;
    }
}

// ─────────────────────────────────────────────────
//  Faz İşleyicileri
// ─────────────────────────────────────────────────

void MissionController::handle_idle() {
    // Otopilota bağlan
    if (!flight_ctrl_->is_connected()) {
        flight_ctrl_->connect();
        return;
    }
    // Sunucuya bağlan
    if (!server_comm_->is_connected()) {
        server_comm_->connect();
    }
    // Her iki bağlantı da varsa pre-arm'a geç
    if (flight_ctrl_->is_connected()) {
        transition_to(FlightPhase::PRE_ARM);
    }
}

void MissionController::handle_pre_arm() {
    // Pre-arm kontrolleri
    auto telem = flight_ctrl_->get_telemetry();
    bool gps_ok = flight_ctrl_->gps_fix_type() >= 3;
    bool batt_ok = flight_ctrl_->battery_percent() > 30;
    bool yolo_ok = yolo_detector_->is_loaded();
    bool cam_ok  = vision_pipeline_->is_running();

    if (gps_ok && batt_ok && yolo_ok && cam_ok) {
        RCLCPP_INFO(get_logger(), "Pre-arm kontrolleri OK. Armlama...");
        if (flight_ctrl_->arm()) {
            transition_to(FlightPhase::ARMED);
        }
    }
}

void MissionController::handle_armed() {
    // Armlandı, kalkışa hazır
    // Otonom moda geç
    flight_ctrl_->set_mode(ArduMode::GUIDED);
    transition_to(FlightPhase::TAKEOFF);
}

void MissionController::handle_takeoff() {
    auto telem = flight_ctrl_->get_telemetry();

    // İlk çağrıda kalkış komutunu gönder
    auto elapsed = std::chrono::steady_clock::now() - phase_entry_time_;
    if (elapsed < 500ms) {
        flight_ctrl_->takeoff(config_.flight.takeoff_altitude);
        return;
    }

    // Hedef irtifaya ulaştı mı?
    if (telem.position.altitude >= config_.flight.takeoff_altitude * 0.90) {
        RCLCPP_INFO(get_logger(), "Kalkış tamamlandı. İrtifa: %.1f m",
                    telem.position.altitude);
        transition_to(FlightPhase::CLIMB);
    }
}

void MissionController::handle_climb() {
    auto telem = flight_ctrl_->get_telemetry();

    // Operasyonel irtifaya tırman
    double target_alt = config_.flight.cruise_altitude;
    if (telem.position.altitude < target_alt * 0.95) {
        flight_ctrl_->set_velocity_ned(0, 0, -3.0);  // yukarı tırman
    } else {
        transition_to(FlightPhase::MISSION_SELECT);
    }
}

void MissionController::handle_mission_select() {
    // QR konum bilgisi girildi mi?
    if (mission_type_ == MissionType::KAMIKAZE) {
        transition_to(FlightPhase::KAMIKAZE_CLIMB);
    } else {
        transition_to(FlightPhase::SEARCH);
    }
}

void MissionController::handle_search() {
    auto telem = flight_ctrl_->get_telemetry();
    auto targets = target_tracker_->active_targets();

    // Hedef tespit edildi mi?
    if (!targets.empty()) {
        auto best = target_tracker_->select_best_target(blacklisted_targets_);
        if (best.track_id >= 0 && !is_target_blacklisted(best.track_id)) {
            RCLCPP_INFO(get_logger(), "Hedef tespit edildi! ID: %d", best.track_id);
            transition_to(FlightPhase::TRACK);
            return;
        }
    }

    // Arama paterni uç
    if (search_pattern_) {
        if (search_pattern_->waypoint_reached(telem.position)) {
            auto next_wp = search_pattern_->next_waypoint();
            auto cmd = guidance_->compute_waypoint(next_wp, telem,
                                                    config_.flight.search_speed);
            if (cmd.is_valid) {
                flight_ctrl_->set_heading_and_speed(
                    cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
            }
        }
    } else {
        // Serbest uçuş — düz devam
        flight_ctrl_->set_heading_and_speed(
            telem.heading, config_.flight.search_speed,
            config_.flight.cruise_altitude);
    }
}

void MissionController::handle_track() {
    auto targets = target_tracker_->active_targets();
    auto telem = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        // Hedef kayboldu → kayıp algoritmasını başlat
        RCLCPP_WARN(get_logger(), "Hedef kayboldu. Arama moduna dönülüyor.");
        transition_to(FlightPhase::SEARCH);
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) {
        transition_to(FlightPhase::SEARCH);
        return;
    }

    // Hedefi ortala (PID güdüm)
    auto cmd = guidance_->compute_tracking(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height,
        telem);

    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(
            cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Hedef vuruş alanında mı? → Kilitlenme fazına geç
    auto lock_info = lockon_manager_->current_info();
    if (lock_info.is_locked || lock_info.lock_duration_s > 0) {
        transition_to(FlightPhase::LOCKON);
    }
}

void MissionController::handle_lockon() {
    auto targets = target_tracker_->active_targets();
    auto telem = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        // Hedef kayboldu → tolerans sayacını kontrol et
        auto info = lockon_manager_->process_no_target();
        if (info.tolerance_counter > LockonManager::MAX_TOLERANCE) {
            RCLCPP_WARN(get_logger(), "Kilitlenme başarısız. Tolerans aşıldı.");
            lockon_manager_->reset();
            transition_to(FlightPhase::SEARCH);
        }
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) {
        transition_to(FlightPhase::SEARCH);
        return;
    }

    // Kilitlenme mantığı
    auto info = lockon_manager_->process(
        best, config_.vision.frame_width, config_.vision.frame_height);

    // Hedefi ortada tut
    auto cmd = guidance_->compute_tracking(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height,
        telem);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(
            cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Kilitlenme başarılı mı? (sayaç ≥ 4)
    if (info.lock_counter >= config_.vision.lockon_confirm_count) {
        RCLCPP_INFO(get_logger(), "KİLİTLENME BAŞARILI! Hedef: %d, Süre: %.2f sn",
                    best.track_id, info.lock_duration_s);

        // Kara listeye ekle
        blacklist_target(best.track_id);
        lockon_manager_->confirm_lockon(best.track_id);
        total_lockons_++;

        // Sunucuya kilitlenme paketi gönder
        LockonPacket pkt;
        pkt.team_id = config_.comm.team_id;
        pkt.lock_rect_x = info.lock_rect.x1;
        pkt.lock_rect_y = info.lock_rect.y1;
        pkt.lock_rect_w = info.lock_rect.width();
        pkt.lock_rect_h = info.lock_rect.height();
        server_comm_->send_lockon_packet(pkt);

        // Aramaya geri dön
        transition_to(FlightPhase::SEARCH);
    }
}

void MissionController::handle_evade() {
    auto telem = flight_ctrl_->get_telemetry();
    auto comps = telemetry_mgr_->competitors();
    auto nfz   = telemetry_mgr_->nfz_zones();

    auto threat = evasion_->evaluate(telem, comps, nfz);

    if (!evasion_->evasion_needed()) {
        // Tehdit geçti → önceki göreve dön
        transition_to(FlightPhase::SEARCH);
        return;
    }

    // Kaçınma güdümü
    auto cmd = guidance_->compute_evasion(
        threat.bearing_deg, threat.distance_m, telem);
    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(
            cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
}

// ── Kamikaze Fazları ──

void MissionController::handle_kamikaze_climb() {
    auto telem = flight_ctrl_->get_telemetry();

    if (telem.position.altitude >= config_.kamikaze.min_dive_altitude) {
        RCLCPP_INFO(get_logger(), "Kamikaze irtifası OK: %.1f m",
                    telem.position.altitude);
        transition_to(FlightPhase::KAMIKAZE_ALIGN);
    } else {
        // Tırmanmaya devam
        double target_alt = config_.kamikaze.climb_altitude;
        flight_ctrl_->set_velocity_ned(0, 0, -4.0);
    }
}

void MissionController::handle_kamikaze_align() {
    auto telem = flight_ctrl_->get_telemetry();

    // QR hedef konumuna hizalan
    auto target = kamikaze_->dive_target();
    auto cmd = guidance_->compute_waypoint(target, telem, config_.flight.approach_speed);

    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Hedefin üzerinde mi?
    double dist = GuidanceSystem::distance_between(telem.position, target);
    if (dist < 20.0) {
        transition_to(FlightPhase::KAMIKAZE_DIVE);
    }
}

void MissionController::handle_kamikaze_dive() {
    auto telem = flight_ctrl_->get_telemetry();
    cv::Mat frame;
    vision_pipeline_->get_latest_frame(frame);

    auto phase = kamikaze_->update(telem, frame);

    if (phase == KamikazePhase::READING_QR && kamikaze_->qr_read_success()) {
        RCLCPP_INFO(get_logger(), "QR KOD OKUNDU: %s", kamikaze_->qr_content().c_str());
        server_comm_->send_qr_data(kamikaze_->qr_content());
    }

    // Güvenli irtifaya geldi mi?
    if (telem.position.altitude <= config_.kamikaze.safe_pullup_alt) {
        transition_to(FlightPhase::KAMIKAZE_PULLUP);
    }

    // Dalış komutu
    flight_ctrl_->set_velocity_ned(
        telem.speed * std::cos(telem.heading * M_PI / 180.0),
        telem.speed * std::sin(telem.heading * M_PI / 180.0),
        8.0);  // aşağı hız (m/s)
}

void MissionController::handle_kamikaze_pullup() {
    auto telem = flight_ctrl_->get_telemetry();

    // Güvenli yüksekliğe çık
    flight_ctrl_->set_velocity_ned(0, 0, -5.0);

    if (telem.position.altitude >= config_.flight.cruise_altitude * 0.8) {
        RCLCPP_INFO(get_logger(), "Kamikaze dalışı tamamlandı.");

        // QR konum bilgisini temizle
        kamikaze_->reset();

        // Savaşan İHA görevine geç
        mission_type_ = MissionType::SAVASAN_IHA;
        transition_to(FlightPhase::SEARCH);
    }
}

void MissionController::handle_loiter() {
    // Serbest uçuş — bekleme
    auto telem = flight_ctrl_->get_telemetry();
    flight_ctrl_->set_mode(ArduMode::LOITER);
}

void MissionController::handle_rtl() {
    auto telem = flight_ctrl_->get_telemetry();

    // Eve yaklaştı mı?
    if (telem.position.altitude < 5.0 && telem.speed < 1.0) {
        transition_to(FlightPhase::LAND);
    }
}

void MissionController::handle_land() {
    auto telem = flight_ctrl_->get_telemetry();

    if (telem.position.altitude < 0.5) {
        RCLCPP_INFO(get_logger(), "İniş tamamlandı.");
        flight_ctrl_->disarm();
        video_recorder_->stop();

        // Videoyu FTP'ye yükle
        if (!video_recorder_->output_path().empty()) {
            server_comm_->upload_video(video_recorder_->output_path());
        }

        transition_to(FlightPhase::IDLE);
    }
}

void MissionController::handle_boundary_return() {
    auto telem = flight_ctrl_->get_telemetry();
    auto boundary = telemetry_mgr_->boundary();

    auto safe_point = safety_monitor_->safe_return_point(telem.position, boundary);
    auto cmd = guidance_->compute_waypoint(safe_point, telem, config_.flight.cruise_speed);

    if (cmd.is_valid) {
        flight_ctrl_->set_heading_and_speed(
            cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }

    // Sınır içine döndü mü?
    if (safety_monitor_->is_in_boundary(telem.position, boundary)) {
        transition_to(FlightPhase::SEARCH);
    }
}

void MissionController::handle_signal_lost() {
    // 10 saniye haberleşme kopmuş → RTL + çember at
    auto telem = flight_ctrl_->get_telemetry();

    // Çember at (loiter)
    flight_ctrl_->set_mode(ArduMode::LOITER);

    // Haberleşme geri geldi mi?
    if (server_comm_->is_connected() && !server_comm_->is_signal_lost()) {
        RCLCPP_INFO(get_logger(), "Haberleşme yeniden sağlandı.");
        transition_to(FlightPhase::SEARCH);
    }

    // 30 saniyeden fazla kopuk → RTL
    auto elapsed = std::chrono::steady_clock::now() - phase_entry_time_;
    if (elapsed > 30s) {
        transition_to(FlightPhase::RTL);
    }
}

void MissionController::handle_emergency() {
    // Acil durum — hemen iniş
    flight_ctrl_->set_mode(ArduMode::QLAND);
}

// ─────────────────────────────────────────────────
//  Yardımcı Metotlar
// ─────────────────────────────────────────────────

void MissionController::process_vision_frame() {
    cv::Mat frame;
    if (!vision_pipeline_->get_latest_frame(frame) || frame.empty()) {
        return;
    }

    // YOLO inference
    auto detections = yolo_detector_->detect(frame);

    // Tracker güncelle
    auto tracked = target_tracker_->update(detections);

    // En iyi hedefi seç ve kilitlenme kontrolü yap
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

    auto boundary = telemetry_mgr_->boundary();
    auto comps = telemetry_mgr_->competitors();
    auto nfz = telemetry_mgr_->nfz_zones();

    auto alert = safety_monitor_->check(
        telem, boundary, comps, nfz, server_comm_->is_connected());

    if (alert.type == SafetyAlert::Type::NONE) return;

    // Aciliyet durumuna göre müdahale
    switch (alert.type) {
        case SafetyAlert::Type::BOUNDARY_VIOLATION:
            if (phase_ != FlightPhase::BOUNDARY_RETURN &&
                phase_ != FlightPhase::RTL &&
                phase_ != FlightPhase::LAND &&
                phase_ != FlightPhase::EMERGENCY) {
                RCLCPP_WARN(get_logger(), "SINIR AŞIMI! Güvenli bölgeye dönülüyor.");
                transition_to(FlightPhase::BOUNDARY_RETURN);
            }
            break;

        case SafetyAlert::Type::ALTITUDE_VIOLATION:
            RCLCPP_WARN(get_logger(), "İRTİFA LİMİTİ! İrtifa düşürülüyor.");
            flight_ctrl_->set_velocity_ned(0, 0, 2.0);  // aşağı in
            break;

        case SafetyAlert::Type::SIGNAL_LOST:
            if (phase_ != FlightPhase::SIGNAL_LOST &&
                phase_ != FlightPhase::RTL &&
                phase_ != FlightPhase::LAND) {
                RCLCPP_WARN(get_logger(), "HABERLEŞME KOPTU!");
                transition_to(FlightPhase::SIGNAL_LOST);
            }
            break;

        case SafetyAlert::Type::CRITICAL_BATTERY:
            RCLCPP_ERROR(get_logger(), "KRİTİK BATARYA! Acil iniş.");
            transition_to(FlightPhase::EMERGENCY);
            break;

        case SafetyAlert::Type::LOW_BATTERY:
            if (phase_ != FlightPhase::RTL && phase_ != FlightPhase::LAND) {
                RCLCPP_WARN(get_logger(), "Düşük batarya. RTL başlatılıyor.");
                transition_to(FlightPhase::RTL);
            }
            break;

        case SafetyAlert::Type::NFZ_INTRUSION:
            RCLCPP_WARN(get_logger(), "KIRMIZI BÖLGE! Kaçınma moduna geçiliyor.");
            transition_to(FlightPhase::EVADE);
            break;

        case SafetyAlert::Type::COLLISION_RISK:
            if (phase_ != FlightPhase::EVADE) {
                transition_to(FlightPhase::EVADE);
            }
            break;

        default:
            break;
    }
}

void MissionController::send_telemetry() {
    auto telem = telemetry_mgr_->own();
    server_comm_->send_telemetry(telem);
}

void MissionController::record_frame() {
    cv::Mat frame;
    if (!vision_pipeline_->get_latest_frame(frame) || frame.empty()) return;

    auto& overlay = video_recorder_->overlay();
    auto st = telemetry_mgr_->server_time();

    // Sunucu saatini çiz
    overlay.draw_server_time(frame, st);

    // Kilitlenme dörtgenini çiz (eğer aktifse)
    auto lock_info = lockon_manager_->current_info();
    if (lock_info.lock_duration_s > 0) {
        overlay.draw_lockon_rect(frame, lock_info.lock_rect);
    }

    // FPS
    overlay.draw_fps(frame, vision_pipeline_->current_fps());

    // Kaydet
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
