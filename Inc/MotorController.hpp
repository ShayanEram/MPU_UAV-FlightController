#pragma once
/**
 * @file MotorController.hpp
 * @brief Header file for the MotorController class.
 * Manages speed and direction of motors.
 */

#include <fcntl.h>

#include <atomic>
#include <chrono>
#include <memory>

#include "Hardware_Interface/HardwareAbstractions.hpp"
#include "InterData.hpp"

class MotorController {
  public:
    struct Config {
        std::shared_ptr<HW::Pwm> m_pwm;
        size_t                   m_motor_channel{0};
        size_t                   m_aileron_channel{1};
        size_t                   m_elevator_channel{2};
        size_t                   m_rudder_channel{3};

        uint16_t m_motor_min_us{MOTOR_MIN_TIME};
        uint16_t m_motor_max_us{MOTOR_MAX_TIME};
        uint16_t m_servo_min_us{SERVO_MIN_TIME};
        uint16_t m_servo_max_us{SERVO_MAX_TIME};

        // Arming policy
        bool  m_require_arm_switch{true}; // require explicit arm
        float m_throttle_ramp_rate{1.0F}; // units per second (0..1)
    };

    explicit MotorController(Config& cfg);
    ~MotorController();

    explicit MotorController(const MotorController& rhs)   = delete;
    explicit MotorController(MotorController&& rhs)        = delete;
    MotorController& operator=(const MotorController& rhs) = delete;
    MotorController& operator=(MotorController&& rhs)      = delete;

    bool Initialize();

    // Arm/disarm API
    bool               Arm();    // returns true if armed
    bool               Disarm(); // returns true if disarmed
    [[nodiscard]] bool IsArmed() const;

    // set immediate failsafe (force outputs)
    void SetFailsafe();

    // apply motor/servo outputs (values normalized -1..1 for servos, 0..1 for throttle)
    bool StepMC(const MotorData& out);

  private:
    [[nodiscard]] uint16_t ThrottleToPulse(float t) const;
    [[nodiscard]] uint16_t ServoToPulse(float s) const;

    Config                                m_cfg;
    std::mutex                            m_apply_mutex;
    std::atomic<bool>                     m_armed{false};
    std::atomic<float>                    m_last_throttle{0.0F};
    std::chrono::steady_clock::time_point m_last_apply_time;

    static constexpr auto MOTOR_MIN_TIME = 1000;
    static constexpr auto MOTOR_MAX_TIME = 2000;
    static constexpr auto SERVO_MIN_TIME = 1000;
    static constexpr auto SERVO_MAX_TIME = 2000;
};