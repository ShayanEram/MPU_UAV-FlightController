/**
 * @file MotorController.hpp
 * @brief Header file for the MotorController class.
 * Manages speed and direction of motors.
 */

#ifndef MOTORCONTROLLER_HPP
#define MOTORCONTROLLER_HPP

#include <fcntl.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "InterData.hpp"

class MotorController {
  public:
    explicit MotorController(SharedResource<MotorData>& motorData);
    ~MotorController();
    void start();
    void stop();

  protected:
    enum class PitchDirection { FORWARD, BACKWARD };
    enum class RollDirection { LEFT, RIGHT };
    enum class YawDirection { LEFT, RIGHT, NONE };
    enum class MotorStatus { DISABLED, ENABLED, FAILSAFE };
    enum class MotorID { MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4 };

    struct MotorDirection {
        YawDirection   yaw;
        PitchDirection pitch;
        RollDirection  roll;
    };

    void setMotorDirection(const MotorDirection& direction);
    int  setMotorSpeed(const MotorID& id, float throttle);
    void stopMotors();
    int  getMotorStatus(const MotorID& id) const;
    void handleFailSafe();
    void enableMotors();
    void setMPwmPeriod(int period);
    void writeHardware(const std::string& path, int value);

    const std::string PWM_PERIOD_PATH     = "/sys/class/pwm/pwmchip0/pwm0/period";
    const std::string PWM_DUTY_CYCLE_PATH = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
    const std::string PWM_ENABLE_PATH     = "/sys/class/pwm/pwmchip0/pwm0/enable";
    const std::string I2C_DEVICE_PATH     = "/dev/i2c-1";

    int   _currentSpeed;
    float _pitchSpeed;
    float _rollSpeed;
    float _yawSpeed;

    static constexpr int MAX_SPEED = 255;
    static constexpr int MIN_SPEED = 0;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    SharedResource<MotorData>& _motorData;
};

#endif // MOTORCONTROLLER_HPP