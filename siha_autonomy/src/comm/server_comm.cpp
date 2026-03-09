/**
 * @file server_comm.cpp
 * @brief Yarışma Sunucusu Haberleşme stub implementasyonu
 */

#include "siha_autonomy/comm/server_comm.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>

// JSON için basit formatlama (gerçek projede nlohmann/json kullanın)

namespace siha {

ServerComm::ServerComm(const CommConfig& cfg) : config_(cfg) {
    last_comm_time_ = std::chrono::steady_clock::now();
}

ServerComm::~ServerComm() { disconnect(); }

bool ServerComm::connect() {
    // TODO: TCP socket aç
    // socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    // struct sockaddr_in serv_addr;
    // serv_addr.sin_family = AF_INET;
    // serv_addr.sin_port = htons(config_.server_port);
    // inet_pton(AF_INET, config_.server_ip.c_str(), &serv_addr.sin_addr);
    // connect(socket_fd_, ...);
    
    std::cout << "[ServerComm] Bağlanılıyor: " 
              << config_.server_ip << ":" << config_.server_port << std::endl;
    connected_ = true;
    running_ = true;
    // recv_thread_ = std::thread(&ServerComm::receive_loop, this);
    return true;
}

void ServerComm::disconnect() {
    running_ = false;
    connected_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    // if (socket_fd_ >= 0) close(socket_fd_);
}

bool ServerComm::send_telemetry(const Telemetry& telem) {
    if (!connected_) return false;
    std::string json = build_telemetry_json(telem);
    // TODO: send(socket_fd_, json.c_str(), json.size(), 0);
    last_comm_time_ = std::chrono::steady_clock::now();
    return true;
}

bool ServerComm::send_lockon_packet(const LockonPacket& packet) {
    if (!connected_) return false;
    std::cout << "[ServerComm] Kilitlenme paketi gönderildi. "
              << "Hedef: " << packet.target_team_id << std::endl;
    last_comm_time_ = std::chrono::steady_clock::now();
    return true;
}

bool ServerComm::send_qr_data(const std::string& qr_content) {
    if (!connected_) return false;
    std::cout << "[ServerComm] QR verisi gönderildi: " << qr_content << std::endl;
    return true;
}

ServerResponse ServerComm::last_response() const {
    std::lock_guard<std::mutex> lock(response_mutex_);
    return last_response_;
}

ServerTime ServerComm::server_time() const {
    std::lock_guard<std::mutex> lock(response_mutex_);
    return last_response_.server_time;
}

double ServerComm::time_since_last_comm() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_comm_time_).count();
}

bool ServerComm::is_signal_lost() const {
    return time_since_last_comm() > config_.signal_loss_timeout;
}

bool ServerComm::upload_video(const std::string& filepath) {
    // TODO: FTP upload implementasyonu
    std::cout << "[ServerComm] Video FTP yükleniyor: " << filepath << std::endl;
    return true;
}

std::string ServerComm::build_telemetry_json(const Telemetry& t) const {
    std::ostringstream oss;
    oss << "{"
        << "\"takim_numarasi\":" << config_.team_id << ","
        << "\"iha_enlem\":" << t.position.latitude << ","
        << "\"iha_boylam\":" << t.position.longitude << ","
        << "\"iha_irtifa\":" << t.position.altitude << ","
        << "\"iha_dikilme\":" << t.pitch << ","
        << "\"iha_yonelme\":" << static_cast<int>(t.heading) << ","
        << "\"iha_yatis\":" << t.roll << ","
        << "\"iha_hiz\":" << t.speed << ","
        << "\"iha_batarya\":" << t.battery << ","
        << "\"iha_otonom\":" << (t.is_autonomous ? 1 : 0)
        << "}";
    return oss.str();
}

void ServerComm::receive_loop() {
    while (running_.load()) {
        // TODO: Sunucudan veri oku ve parse et
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

}  // namespace siha
