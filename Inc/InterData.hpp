#pragma once

#include <cstdint>
#include <functional>

// Health flags
struct HealthFlags {
    bool imu_healthy{false};
    bool gps_fix{false};
    bool baro_healthy{false};
    bool airspeed_healthy{false};
    bool lidar_healthy{false};
    bool battery_healthy{false};
    bool rc_connected{false};
};

// Primary sensor bundle
struct SensorData {
    // IMU
    float accel_x{0.0F}, accel_y{0.0F}, accel_z{0.0F};
    float gyro_x{0.0F}, gyro_y{0.0F}, gyro_z{0.0F};

    // Attitude / heading (fused)
    float roll{0.0F}, pitch{0.0F}, yaw{0.0F};
    float heading{0.0F}; // from compass

    // GPS
    double latitude{0.0};
    double longitude{0.0};
    float  altitude_gps{0.0F};
    float  ground_speed{0.0F};
    int    gps_satellites{0};

    // Barometer
    float altitude_baro{0.0F};
    float pressure{0.0F};
    float temperature{0.0F};

    // Airspeed
    float airspeed{0.0F};

    // Rangefinder / LiDAR
    float range{0.0F};

    // Timestamp (ms since epoch or monotonic)
    uint64_t timestamp_ms{0};

    // Health
    HealthFlags health;
};

// Battery telemetry
struct BatteryData {
    float    voltage{0.0F}; // V
    float    current{0.0F}; // A
    float    remaining_pct{100.0F};
    bool     is_low{false};
    bool     is_critical{false};
    uint64_t timestamp_ms{0};
};

// Remote control channels (normalized -1..1 or 0..1 depending on mapping)
struct RemoteData {
    float    throttle{0.0F};
    float    roll{0.0F};
    float    pitch{0.0F};
    float    yaw{0.0F};
    bool     mode_switch{false}; // e.g., manual/auto
    bool     kill_switch{false};
    bool     is_connected{false};
    uint64_t timestamp_ms{0};
};

// Motor / actuator outputs
struct MotorData {
    // For fixed wing: motor throttle 0..1, servos -1..1 or 0..1 depending on mapping
    float    motor_throttle{0.0F};
    float    servo_aileron{0.0F};
    float    servo_elevator{0.0F};
    float    servo_rudder{0.0F};
    uint64_t timestamp_ms{0};
};

// Telemetry packet combining key data
struct TelemetryPacket {
    SensorData  sensor;
    BatteryData battery;
    RemoteData  remote;
    MotorData   motor;
    uint64_t    timestamp_ms{0};
};

// Callback types
using SensorCallback    = std::function<void(const SensorData&)>;
using BatteryCallback   = std::function<void(const BatteryData&)>;
using RemoteCallback    = std::function<void(const RemoteData&)>;
using MotorCallback     = std::function<void(const MotorData&)>;
using TelemetryCallback = std::function<void(const TelemetryPacket&)>;