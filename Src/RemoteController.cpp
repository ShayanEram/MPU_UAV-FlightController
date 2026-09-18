#include "RemoteController.hpp"
#include <chrono>
#include <iostream>

using namespace uav;
using namespace std::chrono;

RemoteController::RemoteController(const Config& cfg) : _cfg(cfg) {}

RemoteController::~RemoteController() {
    stop();
}

bool RemoteController::init() {
    if (!_cfg.rcInput) {
        std::cerr << "[RemoteController] Warning: no RC adapter injected; using simulated RC\n";
    }
    return true;
}

void RemoteController::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&RemoteController::runLoop, this);
}

void RemoteController::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void RemoteController::setRemoteCallback(RemoteCallback cb) {
    _callback = std::move(cb);
}

bool RemoteController::readOnce(RemoteData& out) {
    if (_cfg.rcInput) {
        std::vector<float> channels;
        if (!_cfg.rcInput->readChannels(channels)) return false;
        out = mapChannelsToRemote(channels);
    } else {
        // simulated neutral RC
        out.throttle = 0.0f;
        out.roll = 0.0f;
        out.pitch = 0.0f;
        out.yaw = 0.0f;
        out.modeSwitch = false;
        out.killSwitch = false;
        out.isConnected = true;
    }
    out.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    return true;
}

void RemoteController::runLoop() {
    const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.loopHz));
    while (_running) {
        RemoteData r;
        if (readOnce(r) && _callback) _callback(r);
        std::this_thread::sleep_for(period);
    }
}

RemoteData RemoteController::mapChannelsToRemote(const std::vector<float>& channels) {
    RemoteData r;
    // Expect channels: throttle, aileron, elevator, rudder, switches...
    if (channels.size() >= 4) {
        r.throttle = channels[0];
        r.roll = channels[1];
        r.pitch = channels[2];
        r.yaw = channels[3];
        r.isConnected = true;
    } else {
        r.isConnected = false;
    }
    // map additional channels to switches if present
    if (channels.size() >= 5) r.modeSwitch = channels[4] > 0.5f;
    if (channels.size() >= 6) r.killSwitch = channels[5] > 0.5f;
    return r;
}
