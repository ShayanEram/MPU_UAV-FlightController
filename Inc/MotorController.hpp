#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <chrono>

namespace uav {

class MotorController {
public:
    struct Config {
        std::shared_ptr<uav::hw::PwmOutput> pwm;
        size_t motorChannel{0};
        size_t aileronChannel{1};
        size_t elevatorChannel{2};
        size_t rudderChannel{3};

        uint16_t motorMinUs{1000};
        uint16_t motorMaxUs{2000};
        uint16_t servoMinUs{1000};
        uint16_t servoMaxUs{2000};

        // Arming policy
        bool requireArmSwitch{true}; // require explicit arm
        float throttleRampRate{1.0f}; // units per second (0..1)
    };

    explicit MotorController(const Config& cfg);
    ~MotorController();

    bool init();

    // Arm/disarm API
    bool arm();    // returns true if armed
    bool disarm(); // returns true if disarmed
    bool isArmed() const;

    // apply motor/servo outputs (values normalized -1..1 for servos, 0..1 for throttle)
    bool apply(const MotorData& out);

    // set immediate failsafe (force outputs)
    void setFailsafe();

private:
    uint16_t throttleToPulse(float t) const;
    uint16_t servoToPulse(float s) const;

    Config _cfg;
    std::atomic<bool> _armed{false};
    std::atomic<float> _lastThrottle{0.0f};
    std::mutex _applyMutex;
    std::chrono::steady_clock::time_point _lastApplyTime;
};

} // namespace uav
