/**
 * @file flight_controller.hpp
 * @brief Uçuş Kontrolcüsü — MAVLink/ArduPilot arayüzü
 *
 * Pixhawk otopilot ile haberleşme katmanı.
 * Otonom kalkış, iniş, mod değiştirme, waypoint gönderme,
 * hız/yön komutları.
 *
 * GUIDED mod üzerinden velocity/position komutları gönderir.
 */
#pragma once

#include "siha_autonomy/core/config.hpp"
#include <string>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>

namespace siha {

/// ArduPilot uçuş modları
enum class ArduMode : uint8_t {
    MANUAL   = 0,
    FBWA     = 5,
    FBWB     = 6,
    CRUISE   = 7,
    AUTO     = 10,
    RTL      = 11,
    LOITER   = 12,
    GUIDED   = 15,
    QLOITER  = 18,   // VTOL loiter
    QLAND    = 20,   // VTOL iniş
    QRTL     = 21    // VTOL RTL
};

/**
 * @class FlightController
 * @brief Otopilot haberleşme ve komut katmanı.
 */
class FlightController {
public:
    explicit FlightController(const FlightConfig& cfg);
    ~FlightController();

    /// MAVLink bağlantısını kur
    bool connect();

    /// Bağlantıyı kes
    void disconnect();

    /// Bağlı mı?
    bool is_connected() const { return connected_.load(); }

    // ── Temel Komutlar ──

    /// Aracı armla
    bool arm();

    /// Aracı disarm et
    bool disarm();

    /// Otonom kalkış (VTOL: QLOITER → takeoff)
    bool takeoff(double altitude_m);

    /// Otonom iniş
    bool land();

    /// Eve dönüş
    bool return_to_launch();

    /// Uçuş modunu değiştir
    bool set_mode(ArduMode mode);

    /// Mevcut mod
    ArduMode current_mode() const { return current_mode_; }

    // ── Navigasyon Komutları ──

    /// GPS konumuna git (GUIDED modu)
    bool goto_position(double lat, double lon, double alt);

    /// NED hız komutu gönder (m/s)
    /// vn: kuzey, ve: doğu, vd: aşağı (negatif = yukarı)
    bool set_velocity_ned(double vn, double ve, double vd);

    /// Yaw açısı ayarla (derece, 0=kuzey, CW pozitif)
    bool set_yaw(double heading_deg, double yaw_rate_deg_s = 0.0);

    /// Heading ve hız ile uçuş komutu
    bool set_heading_and_speed(double heading_deg, double speed_mps,
                                double altitude_m);

    // ── Telemetri Okuma ──

    /// Mevcut telemetriyi al
    Telemetry get_telemetry() const;

    /// Arm durumu
    bool is_armed() const;

    /// GPS fix durumu
    int gps_fix_type() const;

    /// Batarya yüzdesi
    int battery_percent() const;

    // ── Callback'ler ──

    /// Heartbeat kaybı callback
    void on_heartbeat_lost(std::function<void()> cb) { heartbeat_lost_cb_ = cb; }

private:
    void mavlink_receive_loop();   // Arka plan thread
    void send_heartbeat();          // Periyodik heartbeat

    FlightConfig config_;
    std::atomic<bool> connected_{false};
    ArduMode current_mode_ = ArduMode::MANUAL;

    Telemetry latest_telemetry_;
    mutable std::mutex telemetry_mutex_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    int mavlink_fd_ = -1;   // Socket fd
    std::function<void()> heartbeat_lost_cb_;

    std::chrono::steady_clock::time_point last_heartbeat_;
};

}  // namespace siha
