#include "siha_autonomy/core/mission_controller.hpp"
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace siha {

static double sec_since(const Clock::time_point& t) {
    return std::chrono::duration<double>(Clock::now()-t).count();
}

// ═══ Constructor ═══

MissionController::MissionController(const SystemConfig& cfg)
    : Node("mission_controller"), config_(cfg)
{
    RCLCPP_INFO(get_logger(), "═══ SAEROTECH OTONOM SİSTEM v4 ═══");

    yolo_detector_  = std::make_unique<YoloDetector>(cfg.vision);
    target_tracker_ = std::make_unique<TargetTracker>(cfg.vision);
    lockon_manager_ = std::make_unique<LockonManager>(cfg.vision);
    flight_ctrl_    = std::make_unique<FlightController>(cfg.flight, this);
    guidance_       = std::make_unique<GuidanceSystem>(cfg.flight, cfg.vision);
    decision_engine_ = std::make_unique<DecisionEngine>();
    kamikaze_       = std::make_unique<KamikazeModule>(cfg.kamikaze, cfg.vision);
    evasion_        = std::make_unique<EvasionModule>(cfg.safety);
    server_comm_    = std::make_unique<ServerComm>(cfg.comm);
    telemetry_mgr_  = std::make_unique<TelemetryManager>();
    safety_monitor_ = std::make_unique<SafetyMonitor>(cfg.safety);
    video_recorder_ = std::make_unique<VideoRecorder>(cfg.recording);
    overlay_        = std::make_unique<OverlayRenderer>(cfg.vision.rect_thickness);

    if (yolo_detector_->load_model())
        RCLCPP_INFO(get_logger(), "[YOLO] Model: %s", cfg.vision.model_path.c_str());
    else
        RCLCPP_ERROR(get_logger(), "[YOLO] Model yüklenemedi!");

    // Abonelikler (explicit tip — ROS2 Jazzy auto desteklemiyor)
    sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
        cfg.vision.camera_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg){ on_camera_image(msg); });
    sub_npc_ = create_subscription<std_msgs::msg::String>(
        "/sunucu_telemetri", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){ on_npc_telemetry(msg); });
    sub_user_cmd_ = create_subscription<std_msgs::msg::String>(
        "/mission/command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){ on_user_command(msg); });

    // Yayıncılar
    pub_status_     = create_publisher<std_msgs::msg::String>("/mission/status", 10);
    pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 10);

    server_comm_->connect();

    main_timer_      = create_wall_timer(50ms,  [this]{ tick(); });
    telemetry_timer_ = create_wall_timer(500ms, [this]{ send_telemetry(); });
    status_timer_    = create_wall_timer(1s,    [this]{ publish_status(); });

    phase_entry_time_ = last_cmd_time_ = camera_fps_start_ = Clock::now();
    RCLCPP_INFO(get_logger(), "═══ HAZIR. Faz: IDLE ═══");
    RCLCPP_INFO(get_logger(), "Komut: ros2 topic pub --once /mission/command std_msgs/String \"data: 'arm'\"");
}

MissionController::~MissionController() {
    video_recorder_->stop();
}

// ═══ Callbacks ═══

void MissionController::on_camera_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::lock_guard<std::mutex> lock(camera_mutex_);
        latest_camera_frame_ = cv_ptr->image;
        has_new_frame_ = true;
        camera_frame_count_++;
        double e = sec_since(camera_fps_start_);
        if (e >= 2.0) { camera_fps_ = camera_frame_count_/e; camera_frame_count_=0; camera_fps_start_=Clock::now(); }
    } catch (...) {}
}

void MissionController::on_npc_telemetry(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto& s = msg->data;
        std::vector<CompetitorUAV> comps;
        size_t pos = s.find("konumBilgileri");
        if (pos == std::string::npos) return;
        pos = s.find('[', pos);
        while (true) {
            size_t a = s.find('{', pos), b = s.find(']', pos);
            if (a == std::string::npos || (b != std::string::npos && a > b)) break;
            size_t c = s.find('}', a); if (c == std::string::npos) break;
            std::string obj = s.substr(a, c-a+1);
            auto gn = [&](const std::string& k) -> double {
                auto p=obj.find("\""+k+"\""); if(p==std::string::npos) return 0;
                p=obj.find(':',p); try{return std::stod(obj.substr(p+1));}catch(...){return 0;}
            };
            CompetitorUAV u;
            u.team_id = (int)gn("takim_numarasi");
            u.position.latitude = gn("iha_enlem");
            u.position.longitude = gn("iha_boylam");
            u.position.altitude = gn("iha_irtifa");
            u.heading = gn("iha_yonelme");
            u.speed = gn("iha_hiz");
            if (u.team_id > 0) comps.push_back(u);
            pos = c+1;
        }
        if (!comps.empty()) telemetry_mgr_->update_competitors(comps);
    } catch (...) {}
}

void MissionController::on_user_command(const std_msgs::msg::String::SharedPtr msg) {
    std::string c = msg->data;
    RCLCPP_INFO(get_logger(), "Komut: '%s'", c.c_str());
    if (c=="arm") user_arm_requested_=true;
    else if (c=="disarm") { flight_ctrl_->disarm(); transition_to(FlightPhase::IDLE); }
    else if (c=="rtl") transition_to(FlightPhase::RTL);
    else if (c=="land") transition_to(FlightPhase::LAND);
    else if (c=="abort") transition_to(FlightPhase::EMERGENCY);
    else if (c=="search") transition_to(FlightPhase::SEARCH);
    else if (c=="loiter") transition_to(FlightPhase::LOITER);
    else if (c=="guided") { flight_ctrl_->set_mode(ArduMode::GUIDED); RCLCPP_INFO(get_logger(),"GUIDED moda geçildi"); }
    else RCLCPP_WARN(get_logger(), "Bilinmeyen: %s (arm/disarm/rtl/land/abort/search/loiter/guided)", c.c_str());
}

void MissionController::publish_status() {
    static const char* N[] = {"IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","M_SEL",
        "SEARCH","TRACK","LOCKON","EVADE","KMZ_CLB","KMZ_ALN","KMZ_DIV","KMZ_PUL",
        "LOITER","RTL","LAND","BND_RET","SIG_LOST","EMERG"};
    auto t = flight_ctrl_->get_telemetry();
    int pi = (int)phase_;
    const char* pn = (pi>=0&&pi<20) ? N[pi] : "?";
    std::ostringstream o;
    o << "{\"phase\":\"" << pn << "\",\"armed\":" << (flight_ctrl_->is_armed()?"true":"false")
      << ",\"alt\":" << std::fixed << std::setprecision(1) << t.position.altitude
      << ",\"hdg\":" << (int)t.heading << ",\"spd\":" << t.speed
      << ",\"bat\":" << t.battery << ",\"gps\":" << flight_ctrl_->gps_fix_type()
      << ",\"cam\":" << (int)camera_fps_ << ",\"locks\":" << total_lockons_
      << ",\"npc\":" << telemetry_mgr_->competitors().size() << "}";
    auto m = std_msgs::msg::String(); m.data = o.str();
    pub_status_->publish(m);

    static int lc=0;
    if (++lc%3==0)
        RCLCPP_INFO(get_logger(), "[%s] Alt:%.0f Hdg:%d Spd:%.0f Bat:%d GPS:%d Cam:%dFPS NPC:%zu Lock:%d",
            pn, t.position.altitude, (int)t.heading, t.speed, t.battery,
            flight_ctrl_->gps_fix_type(), (int)camera_fps_,
            telemetry_mgr_->competitors().size(), total_lockons_);
}

bool MissionController::send_cmd_throttled(double s) {
    if (sec_since(last_cmd_time_)<s) return false;
    last_cmd_time_=Clock::now(); return true;
}

// ═══ Ana Döngü ═══

void MissionController::tick() {
    check_safety_conditions();
    process_vision_frame();
    publish_debug_image();
    record_frame();

    switch(phase_) {
        case FlightPhase::IDLE: handle_idle(); break;
        case FlightPhase::PRE_ARM: handle_pre_arm(); break;
        case FlightPhase::ARMED: handle_armed(); break;
        case FlightPhase::TAKEOFF: handle_takeoff(); break;
        case FlightPhase::CLIMB: handle_climb(); break;
        case FlightPhase::MISSION_SELECT: handle_mission_select(); break;
        case FlightPhase::SEARCH: handle_search(); break;
        case FlightPhase::TRACK: handle_track(); break;
        case FlightPhase::LOCKON: handle_lockon(); break;
        case FlightPhase::EVADE: handle_evade(); break;
        case FlightPhase::KAMIKAZE_CLIMB: handle_kamikaze_climb(); break;
        case FlightPhase::KAMIKAZE_ALIGN: handle_kamikaze_align(); break;
        case FlightPhase::KAMIKAZE_DIVE: handle_kamikaze_dive(); break;
        case FlightPhase::KAMIKAZE_PULLUP: handle_kamikaze_pullup(); break;
        case FlightPhase::LOITER: handle_loiter(); break;
        case FlightPhase::RTL: handle_rtl(); break;
        case FlightPhase::LAND: handle_land(); break;
        case FlightPhase::BOUNDARY_RETURN: handle_boundary_return(); break;
        case FlightPhase::SIGNAL_LOST: handle_signal_lost(); break;
        case FlightPhase::EMERGENCY: handle_emergency(); break;
    }
}

void MissionController::transition_to(FlightPhase p) {
    if (phase_==p) return;
    static const char* N[]={"IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","M_SEL",
        "SEARCH","TRACK","LOCKON","EVADE","KMZ_CLB","KMZ_ALN","KMZ_DIV","KMZ_PUL",
        "LOITER","RTL","LAND","BND_RET","SIG_LOST","EMERG"};
    RCLCPP_INFO(get_logger(),"══ %s → %s ══", N[(int)phase_], N[(int)p]);
    phase_=p; phase_entry_time_=Clock::now(); cmd_sent_in_phase_=false; retry_count_=0;
    switch(p) {
        case FlightPhase::SEARCH: case FlightPhase::TRACK: guidance_->reset(); break;
        case FlightPhase::LOCKON: if(!video_recorder_->is_recording()) video_recorder_->start(); break;
        case FlightPhase::RTL: flight_ctrl_->return_to_launch(); break;
        case FlightPhase::LAND: flight_ctrl_->land(); break;
        case FlightPhase::SIGNAL_LOST: flight_ctrl_->set_mode(ArduMode::LOITER); break;
        default: break;
    }
}

// ═══ IDLE ═══
void MissionController::handle_idle() {
    if (!cmd_sent_in_phase_) { flight_ctrl_->connect(); cmd_sent_in_phase_=true; }
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.latitude != 0.0) transition_to(FlightPhase::PRE_ARM);
}

// ═══ PRE_ARM ═══
void MissionController::handle_pre_arm() {
    auto t = flight_ctrl_->get_telemetry();
    int gps = flight_ctrl_->gps_fix_type();

    if (gps < 3) {
        if (send_cmd_throttled(5.0)) RCLCPP_INFO(get_logger(),"GPS bekleniyor (fix:%d)", gps);
        return;
    }

    if (user_arm_requested_) {
        user_arm_requested_ = false;
        RCLCPP_INFO(get_logger(),"ARM deneniyor... GPS:%d Lat:%.5f", gps, t.position.latitude);
        flight_ctrl_->arm();
        arm_attempt_time_ = Clock::now();
        retry_count_++;
        return;
    }

    if (retry_count_ > 0) {
        if (flight_ctrl_->is_armed()) {
            RCLCPP_INFO(get_logger(),"ARM BAŞARILI!"); transition_to(FlightPhase::ARMED); return;
        }
        if (sec_since(arm_attempt_time_) > 5.0) {
            RCLCPP_WARN(get_logger(),"ARM başarısız (#%d). 'arm' komutu beklenyor.", retry_count_);
            retry_count_ = 0;
        }
        return;
    }

    if (!cmd_sent_in_phase_) {
        cmd_sent_in_phase_ = true;
        user_arm_requested_ = true;
    }

    if (sec_since(phase_entry_time_) > 20.0 && send_cmd_throttled(10.0))
        RCLCPP_INFO(get_logger(),"ARM bekleniyor. /mission/command → 'arm'");
}

// ═══ ARMED ═══
void MissionController::handle_armed() {
    if (!flight_ctrl_->is_armed()) { transition_to(FlightPhase::PRE_ARM); return; }
    if (!cmd_sent_in_phase_) { cmd_sent_in_phase_=true; }
    if (sec_since(phase_entry_time_) > 0.5) transition_to(FlightPhase::TAKEOFF);
}

// ═══ TAKEOFF (ArduPlane: TAKEOFF modu) ═══
void MissionController::handle_takeoff() {
    if (!cmd_sent_in_phase_) {
        flight_ctrl_->takeoff(config_.flight.takeoff_altitude);
        cmd_sent_in_phase_ = true;
        RCLCPP_INFO(get_logger(),"Takeoff komutu gönderildi (%.0fm)", config_.flight.takeoff_altitude);
        return;
    }

    auto t = flight_ctrl_->get_telemetry();
    double elapsed = sec_since(phase_entry_time_);

    if (send_cmd_throttled(3.0))
        RCLCPP_INFO(get_logger(),"Tırmanma: %.1f/%.0fm (%0.fs)", t.position.altitude, config_.flight.takeoff_altitude, elapsed);

    if (t.position.altitude >= config_.flight.takeoff_altitude * 0.80) {
        RCLCPP_INFO(get_logger(),"Kalkış OK! İrtifa: %.1fm → GUIDED moda geçiliyor", t.position.altitude);
        flight_ctrl_->set_mode(ArduMode::GUIDED);
        transition_to(FlightPhase::CLIMB);
        return;
    }

    if (elapsed > 90.0) {
        RCLCPP_WARN(get_logger(),"Takeoff timeout!");
        if (t.position.altitude > 10.0) { flight_ctrl_->set_mode(ArduMode::GUIDED); transition_to(FlightPhase::LOITER); }
        else transition_to(FlightPhase::PRE_ARM);
    }
}

// ═══ CLIMB ═══
void MissionController::handle_climb() {
    auto t = flight_ctrl_->get_telemetry();
    if (t.position.altitude >= config_.flight.cruise_altitude * 0.85) {
        transition_to(FlightPhase::MISSION_SELECT); return;
    }
    // Cruise irtifasına tırman (GUIDED modda goto ile)
    if (send_cmd_throttled(3.0)) {
        flight_ctrl_->goto_position(t.position.latitude, t.position.longitude, config_.flight.cruise_altitude);
    }
    if (sec_since(phase_entry_time_) > 120.0) transition_to(FlightPhase::MISSION_SELECT);
}

// ═══ MISSION_SELECT ═══
void MissionController::handle_mission_select() {
    if (mission_type_ == MissionType::KAMIKAZE) transition_to(FlightPhase::KAMIKAZE_CLIMB);
    else transition_to(FlightPhase::SEARCH);
}

// ═══ SEARCH — DecisionEngine ile hedef seç ═══
void MissionController::handle_search() {
    // 1) Kamerada hedef varsa → TRACK
    auto vis_targets = target_tracker_->active_targets();
    if (!vis_targets.empty()) {
        auto best = target_tracker_->select_best_target(blacklisted_targets_);
        if (best.track_id >= 0) {
            RCLCPP_INFO(get_logger(),"Kamerada hedef! ID:%d → TRACK", best.track_id);
            transition_to(FlightPhase::TRACK); return;
        }
    }

    // 2) DecisionEngine ile analiz (tehdit + hedef seçimi)
    if (send_cmd_throttled(1.0)) {
        auto t = flight_ctrl_->get_telemetry();
        auto comps = telemetry_mgr_->competitors();
        auto nfz = telemetry_mgr_->nfz_zones();
        auto decision = decision_engine_->evaluate(t, comps, nfz, blacklisted_targets_);

        switch (decision.action) {
            case DecisionResult::Action::EVADE:
                RCLCPP_WARN(get_logger(),"Tehdit! Takım:%d mesafe:%.0fm → EVADE",
                    decision.worst_threat.team_id, decision.worst_threat.distance_m);
                transition_to(FlightPhase::EVADE);
                return;

            case DecisionResult::Action::PURSUE: {
                auto& tgt = decision.best_target;
                current_gps_target_ = tgt;  // Debug görüntü için kaydet
                RCLCPP_INFO(get_logger(),"Hedef: T%d mesafe:%.0fm açı:%.0f° skor:%.2f",
                    tgt.team_id, tgt.distance_m, tgt.angle_off_deg, tgt.score);
                auto cmd = guidance_->compute_waypoint(
                    decision.target_position, t, config_.flight.search_speed);
                if (cmd.is_valid)
                    flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
                break;
            }

            default:
                current_gps_target_ = {};  // hedef yok
                flight_ctrl_->set_heading_and_speed(
                    t.heading, config_.flight.search_speed, config_.flight.cruise_altitude);
                break;
        }
    }
}

// ═══ TRACK — ProNav güdüm ═══
void MissionController::handle_track() {
    auto targets = target_tracker_->active_targets();
    auto t = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        if (sec_since(phase_entry_time_) > 3.0) { transition_to(FlightPhase::SEARCH); }
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) { transition_to(FlightPhase::SEARCH); return; }

    // ProNav + PID hibrit güdüm
    auto cmd = guidance_->compute_tracking_pronav(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height, t);
    if (cmd.is_valid)
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);

    // Kilitlenme kontrolü
    auto info = lockon_manager_->process(best, config_.vision.frame_width, config_.vision.frame_height);
    if (info.lock_duration_s > 0.5) transition_to(FlightPhase::LOCKON);
}

// ═══ LOCKON ═══
void MissionController::handle_lockon() {
    auto targets = target_tracker_->active_targets();
    auto t = flight_ctrl_->get_telemetry();

    if (targets.empty()) {
        auto info = lockon_manager_->process_no_target();
        if (info.tolerance_counter > LockonManager::MAX_TOLERANCE) {
            lockon_manager_->reset(); transition_to(FlightPhase::SEARCH);
        }
        return;
    }

    auto best = target_tracker_->select_best_target(blacklisted_targets_);
    if (best.track_id < 0) { transition_to(FlightPhase::SEARCH); return; }

    auto info = lockon_manager_->process(best, config_.vision.frame_width, config_.vision.frame_height);

    auto cmd = guidance_->compute_tracking_pronav(
        best.bbox.center_x(), best.bbox.center_y(), best.bbox.area(),
        config_.vision.frame_width, config_.vision.frame_height, t);
    if (cmd.is_valid)
        flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);

    if (send_cmd_throttled(1.0))
        RCLCPP_INFO(get_logger(),"Kilitlenme: %.1f/4.0sn T:%d", info.lock_duration_s, best.track_id);

    if (info.lock_counter >= config_.vision.lockon_confirm_count) {
        RCLCPP_INFO(get_logger(),"══ KİLİTLENME BAŞARILI! T:%d %.2fs ══", best.track_id, info.lock_duration_s);
        blacklist_target(best.track_id);
        lockon_manager_->confirm_lockon(best.track_id);
        total_lockons_++;
        LockonPacket pkt; pkt.team_id=config_.comm.team_id;
        pkt.lock_rect_x=info.lock_rect.x1; pkt.lock_rect_y=info.lock_rect.y1;
        pkt.lock_rect_w=info.lock_rect.width(); pkt.lock_rect_h=info.lock_rect.height();
        server_comm_->send_lockon_packet(pkt);
        transition_to(FlightPhase::SEARCH);
    }

    if (sec_since(phase_entry_time_) > 15.0) {
        RCLCPP_WARN(get_logger(),"Lockon timeout"); lockon_manager_->reset();
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══ EVADE — DecisionEngine tehdit yönünden kaç ═══
void MissionController::handle_evade() {
    auto t = flight_ctrl_->get_telemetry();
    auto comps = telemetry_mgr_->competitors();
    auto nfz = telemetry_mgr_->nfz_zones();
    auto decision = decision_engine_->evaluate(t, comps, nfz, blacklisted_targets_);

    if (decision.action != DecisionResult::Action::EVADE) {
        RCLCPP_INFO(get_logger(),"Tehdit geçti → SEARCH");
        transition_to(FlightPhase::SEARCH); return;
    }

    if (send_cmd_throttled(0.5)) {
        auto cmd = guidance_->compute_evasion(
            decision.worst_threat.bearing_deg, decision.worst_threat.distance_m, t);
        if (cmd.is_valid)
            flight_ctrl_->set_heading_and_speed(cmd.heading_deg, cmd.speed_mps, cmd.altitude_m);
    }
    if (sec_since(phase_entry_time_) > 20.0) transition_to(FlightPhase::SEARCH);
}

// ═══ Kamikaze ═══
void MissionController::handle_kamikaze_climb() {
    auto t=flight_ctrl_->get_telemetry();
    if (t.position.altitude >= config_.kamikaze.min_dive_altitude) transition_to(FlightPhase::KAMIKAZE_ALIGN);
    else if (send_cmd_throttled(2.0)) flight_ctrl_->set_velocity_ned(0,0,-4.0);
}
void MissionController::handle_kamikaze_align() {
    auto t=flight_ctrl_->get_telemetry();
    if (send_cmd_throttled(1.0)) {
        auto cmd=guidance_->compute_waypoint(kamikaze_->dive_target(),t,config_.flight.approach_speed);
        if(cmd.is_valid) flight_ctrl_->set_heading_and_speed(cmd.heading_deg,cmd.speed_mps,cmd.altitude_m);
    }
    if (GuidanceSystem::distance_between(t.position, kamikaze_->dive_target()) < 20.0)
        transition_to(FlightPhase::KAMIKAZE_DIVE);
}
void MissionController::handle_kamikaze_dive() {
    auto t=flight_ctrl_->get_telemetry(); cv::Mat f;
    {std::lock_guard<std::mutex> l(camera_mutex_); if(!latest_camera_frame_.empty()) f=latest_camera_frame_.clone();}
    kamikaze_->update(t,f);
    if(kamikaze_->qr_read_success()) server_comm_->send_qr_data(kamikaze_->qr_content());
    if(t.position.altitude<=config_.kamikaze.safe_pullup_alt) transition_to(FlightPhase::KAMIKAZE_PULLUP);
    else if(send_cmd_throttled(0.5)) flight_ctrl_->set_velocity_ned(
        t.speed*std::cos(t.heading*M_PI/180), t.speed*std::sin(t.heading*M_PI/180), 8.0);
}
void MissionController::handle_kamikaze_pullup() {
    auto t=flight_ctrl_->get_telemetry();
    if(send_cmd_throttled(1.0)) flight_ctrl_->set_velocity_ned(0,0,-5.0);
    if(t.position.altitude>=config_.flight.cruise_altitude*0.8) {
        kamikaze_->reset(); mission_type_=MissionType::SAVASAN_IHA;
        transition_to(FlightPhase::SEARCH);
    }
}

// ═══ Diğer ═══
void MissionController::handle_loiter() {
    if(!cmd_sent_in_phase_) { flight_ctrl_->set_mode(ArduMode::LOITER); cmd_sent_in_phase_=true;
        RCLCPP_INFO(get_logger(),"Loiter. Komut bekleniyor..."); }
}
void MissionController::handle_rtl() {
    if(!cmd_sent_in_phase_) { flight_ctrl_->return_to_launch(); cmd_sent_in_phase_=true; }
    auto t=flight_ctrl_->get_telemetry();
    if(t.position.altitude<5.0 && t.speed<1.0) transition_to(FlightPhase::LAND);
}
void MissionController::handle_land() {
    if(!cmd_sent_in_phase_) { flight_ctrl_->land(); cmd_sent_in_phase_=true; }
    if(flight_ctrl_->get_telemetry().position.altitude<0.5) {
        flight_ctrl_->disarm(); video_recorder_->stop(); transition_to(FlightPhase::IDLE); }
}
void MissionController::handle_boundary_return() {
    auto t=flight_ctrl_->get_telemetry(); auto b=telemetry_mgr_->boundary();
    if(safety_monitor_->is_in_boundary(t.position,b)) { transition_to(FlightPhase::SEARCH); return; }
    if(send_cmd_throttled(1.0)) {
        auto s=safety_monitor_->safe_return_point(t.position,b);
        auto c=guidance_->compute_waypoint(s,t,config_.flight.cruise_speed);
        if(c.is_valid) flight_ctrl_->set_heading_and_speed(c.heading_deg,c.speed_mps,c.altitude_m);
    }
}
void MissionController::handle_signal_lost() {
    if(!cmd_sent_in_phase_) { flight_ctrl_->set_mode(ArduMode::LOITER); cmd_sent_in_phase_=true; }
    if(!server_comm_->is_signal_lost()) transition_to(FlightPhase::SEARCH);
    if(sec_since(phase_entry_time_)>30.0) transition_to(FlightPhase::RTL);
}
void MissionController::handle_emergency() {
    if(!cmd_sent_in_phase_) { flight_ctrl_->set_mode(ArduMode::QLAND); cmd_sent_in_phase_=true;
        RCLCPP_ERROR(get_logger(),"ACİL İNİŞ!"); }
}

// ═══ Görüntü İşleme ═══

void MissionController::process_vision_frame() {
    cv::Mat frame;
    { std::lock_guard<std::mutex> l(camera_mutex_);
      if(!has_new_frame_||latest_camera_frame_.empty()) return;
      frame=latest_camera_frame_.clone(); has_new_frame_=false; }
    auto dets = yolo_detector_->detect(frame);
    target_tracker_->update(dets);
}

void MissionController::publish_debug_image() {
    cv::Mat frame;
    { std::lock_guard<std::mutex> l(camera_mutex_);
      if(latest_camera_frame_.empty()) return;
      frame=latest_camera_frame_.clone(); }

    int fw=frame.cols, fh=frame.rows;

    // Vuruş alanı (sarı dikdörtgen)
    int sx1=fw*0.25, sy1=fh*0.10, sx2=fw*0.75, sy2=fh*0.90;
    cv::rectangle(frame, {sx1,sy1}, {sx2,sy2}, {0,255,255}, 1);

    // Crosshair
    cv::line(frame, {fw/2-15,fh/2}, {fw/2+15,fh/2}, {0,255,0}, 1);
    cv::line(frame, {fw/2,fh/2-15}, {fw/2,fh/2+15}, {0,255,0}, 1);

    // YOLO tespitleri (yeşil kutular)
    for (auto& t : target_tracker_->active_targets()) {
        auto& b = t.bbox;
        cv::Scalar col = t.is_blacklisted ? cv::Scalar(128,128,128) : cv::Scalar(0,255,0);
        cv::rectangle(frame, {b.x1,b.y1}, {b.x2,b.y2}, col, 2);
        std::string lbl = "T"+std::to_string(t.track_id)+" "+
            std::to_string((int)(b.confidence*100))+"%";
        cv::putText(frame, lbl, {b.x1,b.y1-5}, cv::FONT_HERSHEY_SIMPLEX, 0.4, col, 1);
    }

    // Kilitlenme dörtgeni (kırmızı)
    auto li = lockon_manager_->current_info();
    if (li.lock_duration_s > 0) {
        cv::rectangle(frame, {li.lock_rect.x1,li.lock_rect.y1},
            {li.lock_rect.x2,li.lock_rect.y2}, {0,0,255}, 3);
        std::string lt = "LOCK "+std::to_string((int)(li.lock_duration_s*10)/10.0).substr(0,3)+"s";
        cv::putText(frame, lt, {li.lock_rect.x1,li.lock_rect.y2+15},
            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255}, 2);
    }

    // Durum bilgisi
    static const char* PN[]={"IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","M_SEL",
        "SEARCH","TRACK","LOCKON","EVADE","KMZ","KMZ","KMZ","KMZ",
        "LOITER","RTL","LAND","BND","SIG","EMRG"};
    int pi=(int)phase_;
    cv::putText(frame, PN[pi<20?pi:0], {10,25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);
    cv::putText(frame, "FPS:"+std::to_string((int)camera_fps_), {10,fh-10},
        cv::FONT_HERSHEY_SIMPLEX, 0.4, {0,255,0}, 1);

    // GPS hedef bilgisi (search sırasında)
    if (current_gps_target_.team_id > 0 && phase_==FlightPhase::SEARCH) {
        std::string ti = "GPS→T"+std::to_string(current_gps_target_.team_id)+
            " "+std::to_string((int)current_gps_target_.distance_m)+"m "+
            std::to_string((int)current_gps_target_.angle_off_deg)+"°";
        cv::putText(frame, ti, {10,50}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {255,200,0}, 1);
    }

    // ROS2 yayınla
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    pub_debug_image_->publish(*msg);
}

// ═══ Güvenlik ═══
void MissionController::check_safety_conditions() {
    auto t=flight_ctrl_->get_telemetry();
    telemetry_mgr_->update_own(t); own_telemetry_=t;
    if(phase_==FlightPhase::IDLE||phase_==FlightPhase::PRE_ARM) return;
    auto al=safety_monitor_->check(t,telemetry_mgr_->boundary(),
        telemetry_mgr_->competitors(),telemetry_mgr_->nfz_zones(),server_comm_->is_connected());
    if(al.type==SafetyAlert::Type::NONE) return;
    switch(al.type) {
        case SafetyAlert::Type::BOUNDARY_VIOLATION:
            if(phase_!=FlightPhase::BOUNDARY_RETURN&&phase_!=FlightPhase::RTL&&phase_!=FlightPhase::EMERGENCY)
                transition_to(FlightPhase::BOUNDARY_RETURN); break;
        case SafetyAlert::Type::CRITICAL_BATTERY: transition_to(FlightPhase::EMERGENCY); break;
        case SafetyAlert::Type::LOW_BATTERY:
            if(phase_!=FlightPhase::RTL&&phase_!=FlightPhase::LAND) transition_to(FlightPhase::RTL); break;
        case SafetyAlert::Type::NFZ_INTRUSION: case SafetyAlert::Type::COLLISION_RISK:
            if(phase_!=FlightPhase::EVADE) transition_to(FlightPhase::EVADE); break;
        default: break;
    }
}

void MissionController::send_telemetry() { server_comm_->send_telemetry(telemetry_mgr_->own()); }
void MissionController::record_frame() {
    cv::Mat f; {std::lock_guard<std::mutex> l(camera_mutex_); if(latest_camera_frame_.empty()) return; f=latest_camera_frame_.clone();}
    overlay_->draw_server_time(f, telemetry_mgr_->server_time());
    auto li=lockon_manager_->current_info();
    if(li.lock_duration_s>0) overlay_->draw_lockon_rect(f,li.lock_rect);
    if(video_recorder_->is_recording()) video_recorder_->write_frame(f);
}
void MissionController::set_mission_type(MissionType t) { mission_type_=t; config_.mission_type=t; }
bool MissionController::is_target_blacklisted(int id) { return blacklisted_targets_.count(id)>0; }
void MissionController::blacklist_target(int id) { blacklisted_targets_.insert(id); lockon_manager_->add_to_blacklist(id); }

}
