#include "FlightController.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace uav;
using namespace std::chrono;

FlightController::FlightController(const Config& cfg) : _cfg(cfg) {}

FlightController::~FlightController() {
    stop();
}

bool FlightController::init() {
    // initialize internal state
    _lastMotor = MotorData{};
    return true;
}

void FlightController::start() {
    if (_running.exchange(true)) return;
    // start a control thread if desired; here we rely on external calls to onSensorUpdate
    // but we can also run a periodic control loop
    std::thread([this]() {
        const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.loopHz));
        while (_running) {
            controlLoopIteration();
            std::this_thread::sleep_for(period);
        }
    }).detach();
}

void FlightController::stop() {
    _running = false;
}

void FlightController::onSensorUpdate(const SensorData& s) {
    _lastSensor = s;
}

void FlightController::onBatteryUpdate(const BatteryData& b) {
    _lastBattery = b;
}

void FlightController::onRemoteUpdate(const RemoteData& r) {
    _lastRemote = r;
}

void FlightController::setMotorOutputCallback(MotorCallback cb) {
    _motorCb = std::move(cb);
}

MotorData FlightController::getMotorData() const {
    return _lastMotor;
}

void FlightController::controlLoopIteration() {
    // Very simple stabilization + throttle mapping example
    // In real system, use proper PID controllers and sensor fusion (AHRS)
    MotorData out;
    // Basic throttle passthrough from RC
    out.motorThrottle = _lastRemote.throttle;

    // Simple servo mixing: map roll/pitch commands to servos
    // Assume remote roll/pitch in -1..1
    out.servoAileron = _lastRemote.roll;
    out.servoElevator = _lastRemote.pitch;
    out.servoRudder = _lastRemote.yaw;

    // Failsafe: if RC disconnected or battery critical, cut throttle
    if (!_lastRemote.isConnected || _lastBattery.isCritical) {
        out.motorThrottle = 0.0f;
    }

    out.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    _lastMotor = out;

    if (_motorCb) _motorCb(out);
}
