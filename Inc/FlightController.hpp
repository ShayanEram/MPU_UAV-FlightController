#pragma once
/**
 * @file FlightController.hpp
 * @brief Header file for the FlightController class.
 * The brain of the aircraft, responsible for stabilization, navigation, and autonomous flight.
 */

#include "Apex.hpp"
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

    // request current motor outputs
    [[nodiscard]] MotorData GetMotorData() const;

    void StepFC();

  private:
    Config m_cfg;

    QueuingPort m_sensor_rx_port;
    QueuingPort m_battery_rx_port;
    QueuingPort m_remote_rx_port;
    QueuingPort m_motor_tx_port;

    // internal state
    SensorData  m_last_sensor;
    BatteryData m_last_battery;
    RemoteData  m_last_remote;
    MotorData   m_last_motor;

    // PID Integral Error Accumulators (Anti-windup)
    float m_integral_roll{0.0F};
    float m_integral_pitch{0.0F};
    float m_integral_yaw{0.0F};

    // Timing tracking for dt calculation
    std::chrono::steady_clock::time_point m_last_loop_time;

    static constexpr auto REFRESH_RATE_HZ = 200;
};