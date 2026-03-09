/**
 * @file server_comm.hpp
 * @brief Yarışma Sunucusu Haberleşme Modülü
 *
 * Hakem sunucusuna kablolu ethernet üzerinden:
 *   - Telemetri gönderme (1-5 Hz)
 *   - Kilitlenme paketi gönderme
 *   - Sunucu saati senkronizasyonu
 *   - Rakip konum bilgisi alma
 *   - HSS (hava savunma) bölge bilgisi alma
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

namespace siha {

/// Kilitlenme paketi (sunucuya gönderilecek)
struct LockonPacket {
    int    team_id           = 0;
    int    target_team_id    = 0;
    double timestamp_ms      = 0.0;  // sunucu saati
    int    lock_rect_x       = 0;
    int    lock_rect_y       = 0;
    int    lock_rect_w       = 0;
    int    lock_rect_h       = 0;
    double target_center_x   = 0.0;
    double target_center_y   = 0.0;
};

/// Sunucu yanıtı
struct ServerResponse {
    ServerTime               server_time;
    std::vector<CompetitorUAV> competitors;
    std::vector<NoFlyZone>     nfz_zones;
    bool                      valid = false;
};

class ServerComm {
public:
    explicit ServerComm(const CommConfig& cfg);
    ~ServerComm();

    /// Sunucuya bağlan
    bool connect();

    /// Bağlantıyı kes
    void disconnect();

    /// Bağlı mı?
    bool is_connected() const { return connected_.load(); }

    /// Telemetri paketi gönder
    bool send_telemetry(const Telemetry& telem);

    /// Kilitlenme paketi gönder
    bool send_lockon_packet(const LockonPacket& packet);

    /// QR kod içeriğini gönder (kamikaze)
    bool send_qr_data(const std::string& qr_content);

    /// Son sunucu yanıtı
    ServerResponse last_response() const;

    /// Sunucu saati
    ServerTime server_time() const;

    /// Son başarılı haberleşmeden beri geçen süre (saniye)
    double time_since_last_comm() const;

    /// Haberleşme koptu mu? (>10 saniye)
    bool is_signal_lost() const;

    /// Video dosyasını FTP'ye yükle
    bool upload_video(const std::string& filepath);

    /// Callback: yeni rakip verisi geldiğinde
    using CompetitorCallback = std::function<void(const std::vector<CompetitorUAV>&)>;
    void on_competitors_update(CompetitorCallback cb) { competitor_cb_ = cb; }

    /// Callback: yeni HSS verisi geldiğinde
    using NFZCallback = std::function<void(const std::vector<NoFlyZone>&)>;
    void on_nfz_update(NFZCallback cb) { nfz_cb_ = cb; }

private:
    void receive_loop();           // Arka plan alım thread'i
    std::string build_telemetry_json(const Telemetry& t) const;
    ServerResponse parse_response(const std::string& json) const;

    CommConfig config_;
    std::atomic<bool> connected_{false};

    int socket_fd_ = -1;

    ServerResponse last_response_;
    mutable std::mutex response_mutex_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    std::chrono::steady_clock::time_point last_comm_time_;

    CompetitorCallback competitor_cb_;
    NFZCallback nfz_cb_;
};

}  // namespace siha
