#include "TelemetryManager.hpp"
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>

using namespace uav;
using namespace std::chrono;

TelemetryManager::TelemetryManager(const Config& cfg) : _cfg(cfg) {}

TelemetryManager::~TelemetryManager() {
    stop();
}

bool TelemetryManager::init() {
    if (!_cfg.telemetryUart) {
        std::cerr << "[TelemetryManager] Warning: no telemetry UART injected; telemetry disabled\n";
    }
    return true;
}

void TelemetryManager::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&TelemetryManager::runLoop, this);
}

void TelemetryManager::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void TelemetryManager::send(const TelemetryPacket& pkt) {
    // simple thread-safe queue
    static std::mutex qmut;
    static std::queue<TelemetryPacket> q;
    std::lock_guard<std::mutex> lk(qmut);
    q.push(pkt);
    // move queue to local in runLoop for sending
}

void TelemetryManager::setTelemetryCallback(TelemetryCallback cb) {
    _callback = std::move(cb);
}

void TelemetryManager::runLoop() {
    const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.sendHz));
    while (_running) {
        // For simplicity, build a small telemetry string and send it
        TelemetryPacket pkt;
        // In a real implementation, you'd pop from a queue; here we simulate periodic status
        pkt.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        // Example payload
        std::string payload = "TELEM," + std::to_string(pkt.timestampMs) + "\n";
        if (_cfg.telemetryUart) {
            _cfg.telemetryUart->write(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        } else {
            // print to stdout for debugging
            std::cout << "[Telemetry] " << payload;
        }
        if (_callback) _callback(pkt);
        std::this_thread::sleep_for(period);
    }
}
