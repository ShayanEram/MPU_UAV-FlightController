#pragma once

#include <cstdint>
#include <functional>

// Health flags
struct HealthFlags {
    bool m_imu_healthy{false};
    bool m_gps_fix{false};
    bool m_baro_healthy{false};
    bool m_airspeed_healthy{false};
    bool m_lidar_healthy{false};
    bool m_battery_healthy{false};
    bool m_rc_connected{false};
};

// Primary sensor bundle
struct SensorData {
    // IMU
    float m_accel_x{0.0F}, m_accel_y{0.0F}, m_accel_z{0.0F};
    float m_gyro_x{0.0F}, m_gyro_y{0.0F}, m_gyro_z{0.0F};

    // Attitude / heading (fused)
    float m_roll{0.0F}, m_pitch{0.0F}, m_yaw{0.0F};
    float m_heading{0.0F}; // from compass

    // GPS
    double m_latitude{0.0};
    double m_longitude{0.0};
    float  m_altitude_gps{0.0F};
    float  m_ground_speed{0.0F};
    int    m_gps_satellites{0};

    // Barometer
    float m_altitude_baro{0.0F};
    float m_pressure{0.0F};
    float m_temperature{0.0F};

    // Airspeed
    float m_airspeed{0.0F};

    // Rangefinder / LiDAR
    float m_range{0.0F};

    // Timestamp (ms since epoch or monotonic)
    uint64_t m_timestamp_ms{0};

    // Health
    HealthFlags m_health;
};

// Battery telemetry
struct BatteryData {
    float    m_voltage{0.0F}; // V
    float    m_current{0.0F}; // A
    float    m_remaining_pct{100.0F};
    bool     m_is_low{false};
    bool     m_is_critical{false};
    uint64_t m_timestamp_ms{0};
};

// Remote control channels (normalized -1..1 or 0..1 depending on mapping)
struct RemoteData {
    float    m_throttle{0.0F};
    float    m_roll{0.0F};
    float    m_pitch{0.0F};
    float    m_yaw{0.0F};
    bool     m_mode_switch{false}; // e.g., manual/auto
    bool     m_kill_switch{false};
    bool     m_is_connected{false};
    uint64_t m_timestamp_ms{0};
};

// Motor / actuator outputs
struct MotorData {
    // For fixed wing: motor throttle 0..1, servos -1..1 or 0..1 depending on mapping
    float    m_motor_throttle{0.0F};
    float    m_servo_aileron{0.0F};
    float    m_servo_elevator{0.0F};
    float    m_servo_rudder{0.0F};
    uint64_t m_timestamp_ms{0};
};

// Telemetry packet combining key data
struct TelemetryPacket {
    SensorData  m_sensor;
    BatteryData m_battery;
    RemoteData  m_remote;
    MotorData   m_motor;
    uint64_t    m_timestamp_ms{0};
};

// Callback types
using SensorCallback    = std::function<void(const SensorData&)>;
using BatteryCallback   = std::function<void(const BatteryData&)>;
using RemoteCallback    = std::function<void(const RemoteData&)>;
using MotorCallback     = std::function<void(const MotorData&)>;
using TelemetryCallback = std::function<void(const TelemetryPacket&)>;