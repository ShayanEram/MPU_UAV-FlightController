#include "MotorController.hpp"
#include <algorithm>
#include <iostream>
#include <thread>

using namespace uav;

MotorController::MotorController(const Config& cfg) : _cfg(cfg) {
    _lastApplyTime = std::chrono::steady_clock::now();
}

MotorController::~MotorController() = default;

bool MotorController::init() {
    if (!_cfg.pwm) {
        std::cerr << "[MotorController] Error: no PWM adapter injected\n";
        return false;
    }
    // Ensure outputs are at safe neutral on init
    setFailsafe();
    return true;
}

bool MotorController::arm() {
    // require explicit conditions if configured (caller should check RC switch, etc.)
    _armed = true;
    // ensure motor starts at zero throttle
    _lastThrottle = 0.0f;
    // apply zero throttle immediately
    MotorData m;
    m.motorThrottle = 0.0f;
    m.servoAileron = 0.0f;
    m.servoElevator = 0.0f;
    m.servoRudder = 0.0f;
    apply(m);
    std::cerr << "[MotorController] Armed\n";
    return true;
}

bool MotorController::disarm() {
    _armed = false;
    setFailsafe();
    std::cerr << "[MotorController] Disarmed\n";
    return true;
}

bool MotorController::isArmed() const {
    return _armed.load();
}

uint16_t MotorController::throttleToPulse(float t) const {
    float tt = std::max(0.0f, std::min(1.0f, t));
    return static_cast<uint16_t>(_cfg.motorMinUs + tt * (_cfg.motorMaxUs - _cfg.motorMinUs));
}

uint16_t MotorController::servoToPulse(float s) const {
    float ss = std::max(-1.0f, std::min(1.0f, s));
    uint16_t mid = static_cast<uint16_t>((_cfg.servoMinUs + _cfg.servoMaxUs) / 2);
    uint16_t halfRange = static_cast<uint16_t>((_cfg.servoMaxUs - _cfg.servoMinUs) / 2);
    return static_cast<uint16_t>(mid + ss * halfRange);
}

bool MotorController::apply(const MotorData& out) {
    std::lock_guard<std::mutex> lk(_applyMutex);
    if (!_cfg.pwm) return false;

    // Throttle ramping
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - _lastApplyTime).count();
    _lastApplyTime = now;

    float desiredThrottle = out.motorThrottle;
    desiredThrottle = std::max(0.0f, std::min(1.0f, desiredThrottle));

    // If not armed, force throttle to zero
    if (!_armed.load()) desiredThrottle = 0.0f;

    // Ramp limit
    float maxDelta = _cfg.throttleRampRate * dt;
    float last = _lastThrottle.load();
    float delta = desiredThrottle - last;
    if (std::fabs(delta) > maxDelta) {
        desiredThrottle = last + (delta > 0 ? maxDelta : -maxDelta);
    }
    _lastThrottle = desiredThrottle;

    // Map to pulses
    uint16_t motorPulse = throttleToPulse(desiredThrottle);
    uint16_t aPulse = servoToPulse(out.servoAileron);
    uint16_t ePulse = servoToPulse(out.servoElevator);
    uint16_t rPulse = servoToPulse(out.servoRudder);

    // Clamp pulses
    auto clamp = [](uint16_t v, uint16_t lo, uint16_t hi)->uint16_t {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    };

    motorPulse = clamp(motorPulse, _cfg.motorMinUs, _cfg.motorMaxUs);
    aPulse = clamp(aPulse, _cfg.servoMinUs, _cfg.servoMaxUs);
    ePulse = clamp(ePulse, _cfg.servoMinUs, _cfg.servoMaxUs);
    rPulse = clamp(rPulse, _cfg.servoMinUs, _cfg.servoMaxUs);

    // Apply pulses
    if (!_cfg.pwm->setPulseWidth(_cfg.motorChannel, motorPulse)) return false;
    if (!_cfg.pwm->setPulseWidth(_cfg.aileronChannel, aPulse)) return false;
    if (!_cfg.pwm->setPulseWidth(_cfg.elevatorChannel, ePulse)) return false;
    if (!_cfg.pwm->setPulseWidth(_cfg.rudderChannel, rPulse)) return false;

    return true;
}

void MotorController::setFailsafe() {
    std::lock_guard<std::mutex> lk(_applyMutex);
    if (!_cfg.pwm) return;
    // motor zero throttle and neutral servos
    uint16_t motorPulse = throttleToPulse(0.0f);
    uint16_t neutral = servoToPulse(0.0f);
    _cfg.pwm->setPulseWidth(_cfg.motorChannel, motorPulse);
    _cfg.pwm->setPulseWidth(_cfg.aileronChannel, neutral);
    _cfg.pwm->setPulseWidth(_cfg.elevatorChannel, neutral);
    _cfg.pwm->setPulseWidth(_cfg.rudderChannel, neutral);
    _lastThrottle = 0.0f;
}
