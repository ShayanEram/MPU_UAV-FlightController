#pragma once
/**
 * @file MotorController.hpp
 * @brief Header file for the MotorController class.
 * Manages speed and direction of motors.
 */

#include <fcntl.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

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

        uint16_t m_motor_min_us{1000};
        uint16_t m_motor_max_us{2000};
        uint16_t m_servo_min_us{1000};
        uint16_t m_servo_max_us{2000};

        // Arming policy
        bool  m_require_arm_switch{true}; // require explicit arm
        float m_throttle_ramp_rate{1.0f}; // units per second (0..1)
    };

    explicit MotorController(const Config& cfg);
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

    // apply motor/servo outputs (values normalized -1..1 for servos, 0..1 for throttle)
    bool Apply(const MotorData& out);

    // set immediate failsafe (force outputs)
    void SetFailsafe();

  private:
    void Step();

    [[nodiscard]] uint16_t ThrottleToPulse(float t) const;
    [[nodiscard]] uint16_t ServoToPulse(float s) const;

    Config                                m_cfg;
    std::atomic<bool>                     m_armed{false};
    std::atomic<float>                    m_last_throttle{0.0f};
    std::chrono::steady_clock::time_point m_last_apply_time;
};