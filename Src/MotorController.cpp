#include "MotorController.hpp"

#include <spdlog/spdlog.h>

MotorController::MotorController(Config& cfg) : m_cfg(std::move(cfg)) {
    m_last_apply_time = std::chrono::steady_clock::now();
}

MotorController::~MotorController() {
    spdlog::debug("MotorController stopped!");
}
//------------------------------------------------------------------------------------
bool MotorController::Initialize() {
    if (!m_cfg.m_pwm) {
        spdlog::error("[MotorController] Error: no PWM adapter injected");
        return false;
    }
    // Ensure outputs are at safe neutral on init
    SetFailsafe();
    return true;
}
//------------------------------------------------------------------------------------
bool MotorController::StepMC(const MotorData& out) {
    std::scoped_lock lk(m_apply_mutex);
    if (!m_cfg.m_pwm) {
        return false;
    }

    // Throttle ramping
    auto  now         = std::chrono::steady_clock::now();
    float dt          = std::chrono::duration<float>(now - m_last_apply_time).count();
    m_last_apply_time = now;

    float desired_throttle = out.motor_throttle;
    desired_throttle       = std::max(0.0f, std::min(1.0f, desired_throttle));

    // If not armed, force throttle to zero
    if (!m_armed.load()) {
        desired_throttle = 0.0F;
    }

    // Ramp limit
    float max_delta = m_cfg.m_throttle_ramp_rate * dt;
    float last      = m_last_throttle.load();
    float delta     = desired_throttle - last;
    if (std::fabs(delta) > max_delta) {
        desired_throttle = last + (delta > 0 ? max_delta : -max_delta);
    }
    m_last_throttle = desired_throttle;

    // Map to pulses
    uint16_t motor_pulse = ThrottleToPulse(desired_throttle);
    uint16_t a_pulse     = ServoToPulse(out.servo_aileron);
    uint16_t e_pulse     = ServoToPulse(out.servo_elevator);
    uint16_t r_pulse     = ServoToPulse(out.servo_rudder);

    // Clamp pulses
    auto clamp = [](uint16_t v, uint16_t lo, uint16_t hi) -> uint16_t {
        if (v < lo) {
            return lo;
        }
        if (v > hi) {
            return hi;
        }
        return v;
    };

    motor_pulse = clamp(motor_pulse, m_cfg.m_motor_min_us, m_cfg.m_motor_max_us);
    a_pulse     = clamp(a_pulse, m_cfg.m_servo_min_us, m_cfg.m_servo_max_us);
    e_pulse     = clamp(e_pulse, m_cfg.m_servo_min_us, m_cfg.m_servo_max_us);
    r_pulse     = clamp(r_pulse, m_cfg.m_servo_min_us, m_cfg.m_servo_max_us);

    // Apply pulses
    if (!m_cfg.m_pwm->SetPulseWidth(m_cfg.m_motor_channel, motor_pulse)) {
        return false;
    }
    if (!m_cfg.m_pwm->SetPulseWidth(m_cfg.m_aileron_channel, a_pulse)) {
        return false;
    }
    if (!m_cfg.m_pwm->SetPulseWidth(m_cfg.m_elevator_channel, e_pulse)) {
        return false;
    }
    if (!m_cfg.m_pwm->SetPulseWidth(m_cfg.m_rudder_channel, r_pulse)) {
        return false;
    }

    return true;
}
//------------------------------------------------------------------------------------
bool MotorController::Arm() {
    // require explicit conditions if configured (caller should check RC switch, etc.)
    m_armed = true;
    // ensure motor starts at zero throttle
    m_last_throttle = 0.0F;
    // apply zero throttle immediately
    MotorData m;
    m.motor_throttle = 0.0F;
    m.servo_aileron  = 0.0F;
    m.servo_elevator = 0.0F;
    m.servo_rudder   = 0.0F;
    StepMC(m);
    spdlog::debug("[MotorController] Armed");
    return true;
}

bool MotorController::Disarm() {
    m_armed = false;
    SetFailsafe();
    spdlog::debug("[MotorController] Disarmed");
    return true;
}

bool MotorController::IsArmed() const {
    return m_armed.load();
}

uint16_t MotorController::ThrottleToPulse(float t) const {
    float tt = std::max(0.0F, std::min(1.0F, t));
    return static_cast<uint16_t>(static_cast<float>(m_cfg.m_motor_min_us) +
                                 (tt * (static_cast<float>(m_cfg.m_motor_max_us) - static_cast<float>(m_cfg.m_motor_min_us))));
}

uint16_t MotorController::ServoToPulse(float s) const {
    float ss         = std::max(-1.0F, std::min(1.0F, s));
    auto  mid        = static_cast<uint16_t>((m_cfg.m_servo_min_us + m_cfg.m_servo_max_us) / 2);
    auto  half_range = static_cast<uint16_t>((m_cfg.m_servo_max_us - m_cfg.m_servo_min_us) / 2);
    return static_cast<uint16_t>(static_cast<float>(mid) + (ss * static_cast<float>(half_range)));
}

void MotorController::SetFailsafe() {
    std::scoped_lock lk(m_apply_mutex);
    if (!m_cfg.m_pwm) {
        return;
    }
    // motor zero throttle and neutral servos
    uint16_t m_motor_pulse = ThrottleToPulse(0.0F);
    uint16_t neutral       = ServoToPulse(0.0F);
    m_cfg.m_pwm->SetPulseWidth(m_cfg.m_motor_channel, m_motor_pulse);
    m_cfg.m_pwm->SetPulseWidth(m_cfg.m_aileron_channel, neutral);
    m_cfg.m_pwm->SetPulseWidth(m_cfg.m_elevator_channel, neutral);
    m_cfg.m_pwm->SetPulseWidth(m_cfg.m_rudder_channel, neutral);
    m_last_throttle = 0.0F;
}