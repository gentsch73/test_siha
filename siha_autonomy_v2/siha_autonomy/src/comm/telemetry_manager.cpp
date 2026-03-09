/**
 * @file telemetry_manager.cpp
 */
#include "siha_autonomy/comm/telemetry_manager.hpp"

namespace siha {

TelemetryManager::TelemetryManager() = default;

void TelemetryManager::update_own(const Telemetry& t) {
    std::lock_guard<std::mutex> lock(mutex_);
    own_ = t;
}

Telemetry TelemetryManager::own() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return own_;
}

void TelemetryManager::update_competitors(const std::vector<CompetitorUAV>& c) {
    std::lock_guard<std::mutex> lock(mutex_);
    competitors_ = c;
}

std::vector<CompetitorUAV> TelemetryManager::competitors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return competitors_;
}

void TelemetryManager::update_nfz(const std::vector<NoFlyZone>& z) {
    std::lock_guard<std::mutex> lock(mutex_);
    nfz_zones_ = z;
}

std::vector<NoFlyZone> TelemetryManager::nfz_zones() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return nfz_zones_;
}

void TelemetryManager::update_server_time(const ServerTime& st) {
    std::lock_guard<std::mutex> lock(mutex_);
    server_time_ = st;
}

ServerTime TelemetryManager::server_time() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return server_time_;
}

void TelemetryManager::set_boundary(const ArenaBoundary& b) {
    std::lock_guard<std::mutex> lock(mutex_);
    boundary_ = b;
}

ArenaBoundary TelemetryManager::boundary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return boundary_;
}

}  // namespace siha
