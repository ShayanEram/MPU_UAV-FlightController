#include "BatteryManager.hpp"
#include <chrono>
#include <iostream>

using namespace uav;
using namespace std::chrono;

BatteryManager::BatteryManager(const Config& cfg) : _cfg(cfg) {}

BatteryManager::~BatteryManager() {
    stop();
}

bool BatteryManager::init() {
    if (!_cfg.adc) {
        std::cerr << "[BatteryManager] Warning: no ADC adapter injected; using simulated battery\n";
    }
    return true;
}

void BatteryManager::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&BatteryManager::runLoop, this);
}

void BatteryManager::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void BatteryManager::setBatteryCallback(BatteryCallback cb) {
    _callback = std::move(cb);
}

bool BatteryManager::readOnce(BatteryData& out) {
    if (_cfg.adc) {
        float v = 0.0f, a = 0.0f;
        if (_cfg.adc->readVoltage(v)) out.voltage = v;
        if (_cfg.adc->readCurrent(a)) out.current = a;
    } else {
        // simulated
        out.voltage = 11.1f;
        out.current = 1.2f;
    }
    // simple SOC estimate
    out.remainingPct = std::max(0.0f, std::min(100.0f, (out.voltage - 9.0f) / (12.6f - 9.0f) * 100.0f));
    out.isLow = out.remainingPct < _cfg.lowThresholdPct;
    out.isCritical = out.remainingPct < _cfg.criticalThresholdPct;
    out.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    return true;
}

void BatteryManager::runLoop() {
    const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.loopHz));
    while (_running) {
        BatteryData b;
        readOnce(b);
        if (_callback) _callback(b);
        std::this_thread::sleep_for(period);
    }
}
