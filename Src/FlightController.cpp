#include "FlightController.hpp"

#include <spdlog/spdlog.h>

FlightController::FlightController(Config& cfg) : m_cfg(std::move(cfg)) {}

FlightController::~FlightController() {
    spdlog::debug("FlightController stopped!");
}
//------------------------------------------------------------------------------------
bool FlightController::Initialize() {
    // initialize internal state
    m_last_motor = MotorData{};
    return true;
}
//------------------------------------------------------------------------------------
void FlightController::StepFC() { // Very simple stabilization + throttle mapping example
    // In real system, use proper PID controllers and sensor fusion (AHRS)
    MotorData out;
    // Basic throttle passthrough from RC
    out.m_motor_throttle = m_last_remote.m_throttle;

    // Simple servo mixing: map roll/pitch commands to servos
    // Assume remote roll/pitch in -1..1
    out.m_servo_aileron  = m_last_remote.m_roll;
    out.m_servo_elevator = m_last_remote.m_pitch;
    out.m_servo_rudder   = m_last_remote.m_yaw;

    // Failsafe: if RC disconnected or battery critical, cut throttle
    if (!m_last_remote.m_is_connected || m_last_battery.m_is_critical) {
        out.m_motor_throttle = 0.0f;
    }

    out.m_timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    m_last_motor       = out;

    if (m_motor_cb) {
        m_motor_cb(out);
    }
}
//------------------------------------------------------------------------------------
void FlightController::OnSensorUpdate(const SensorData& s) {
    m_last_sensor = s;
}

void FlightController::OnBatteryUpdate(const BatteryData& b) {
    m_last_battery = b;
}

void FlightController::OnRemoteUpdate(const RemoteData& r) {
    m_last_remote = r;
}

void FlightController::SetMotorOutputCallback(MotorCallback cb) {
    m_motor_cb = std::move(cb);
}

MotorData FlightController::GetMotorData() const {
    return m_last_motor;
}