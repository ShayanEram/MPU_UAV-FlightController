#pragma once
/**
 * @file FlightController.hpp
 * @brief Header file for the FlightController class.
 * The brain of the aircraft, responsible for stabilization, navigation, and autonomous flight.
 */

#include "InterData.hpp"

class FlightController {
  public:
    struct Config {
        // tuning parameters, control loop frequency, failsafe thresholds
        float    m_kp_roll{1.0F}, m_ki_roll{0.0F}, m_kd_roll{0.0F};
        float    m_kp_pitch{1.0F}, m_ki_pitch{0.0F}, m_kd_pitch{0.0F};
        float    m_kp_yaw{1.0F}, m_ki_yaw{0.0F}, m_kd_yaw{0.0F};
        uint32_t m_loop_hz{REFRESH_RATE_HZ};
    };

    explicit FlightController(Config& cfg);
    ~FlightController();

    explicit FlightController(const FlightController& rhs)   = delete;
    explicit FlightController(FlightController&& rhs)        = delete;
    FlightController& operator=(const FlightController& rhs) = delete;
    FlightController& operator=(FlightController&& rhs)      = delete;

    bool Initialize();

    // input sources (called by higher-level orchestrator or callbacks)
    void OnSensorUpdate(const SensorData& s);
    void OnBatteryUpdate(const BatteryData& b);
    void OnRemoteUpdate(const RemoteData& r);

    // output sink (set by orchestrator)
    void SetMotorOutputCallback(MotorCallback cb);

    // request current motor outputs
    [[nodiscard]] MotorData GetMotorData() const;

    void StepFC();

  private:
    Config m_cfg;

    // internal state
    SensorData  m_last_sensor;
    BatteryData m_last_battery;
    RemoteData  m_last_remote;
    MotorData   m_last_motor;

    MotorCallback m_motor_cb;

    static constexpr auto REFRESH_RATE_HZ = 200;
};