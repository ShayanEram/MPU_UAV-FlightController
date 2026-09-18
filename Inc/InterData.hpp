#pragma once
#include <cstdint>
#include <functional>
#include <array>

namespace uav {

// Health flags
struct HealthFlags {
    bool imuHealthy{false};
    bool gpsFix{false};
    bool baroHealthy{false};
    bool airspeedHealthy{false};
    bool lidarHealthy{false};
    bool batteryHealthy{false};
    bool rcConnected{false};
};

// Primary sensor bundle
struct SensorData {
    // IMU
    float accelX{0.0f}, accelY{0.0f}, accelZ{0.0f};
    float gyroX{0.0f},  gyroY{0.0f},  gyroZ{0.0f};

    // Attitude / heading (fused)
    float roll{0.0f}, pitch{0.0f}, yaw{0.0f};
    float heading{0.0f}; // from compass

    // GPS
    double latitude{0.0};
    double longitude{0.0};
    float altitudeGPS{0.0f};
    float groundSpeed{0.0f};
    int   gpsSatellites{0};

    // Barometer
    float altitudeBaro{0.0f};
    float pressure{0.0f};
    float temperature{0.0f};

    // Airspeed
    float airspeed{0.0f};

    // Rangefinder / LiDAR
    float range{0.0f};

    // Timestamp (ms since epoch or monotonic)
    uint64_t timestampMs{0};

    // Health
    HealthFlags health;
};

// Battery telemetry
struct BatteryData {
    float voltage{0.0f};      // V
    float current{0.0f};      // A
    float remainingPct{100.0f};
    bool  isLow{false};
    bool  isCritical{false};
    uint64_t timestampMs{0};
};

// Remote control channels (normalized -1..1 or 0..1 depending on mapping)
struct RemoteData {
    float throttle{0.0f};
    float roll{0.0f};
    float pitch{0.0f};
    float yaw{0.0f};
    bool  modeSwitch{false};   // e.g., manual/auto
    bool  killSwitch{false};
    bool  isConnected{false};
    uint64_t timestampMs{0};
};

// Motor / actuator outputs
struct MotorData {
    // For fixed wing: motor throttle 0..1, servos -1..1 or 0..1 depending on mapping
    float motorThrottle{0.0f};
    float servoAileron{0.0f};
    float servoElevator{0.0f};
    float servoRudder{0.0f};
    uint64_t timestampMs{0};
};

// Telemetry packet combining key data
struct TelemetryPacket {
    SensorData sensor;
    BatteryData battery;
    RemoteData remote;
    MotorData motor;
    uint64_t timestampMs{0};
};

// Callback types
using SensorCallback = std::function<void(const SensorData&)>;
using BatteryCallback = std::function<void(const BatteryData&)>;
using RemoteCallback  = std::function<void(const RemoteData&)>;
using MotorCallback   = std::function<void(const MotorData&)>;
using TelemetryCallback = std::function<void(const TelemetryPacket&)>;

} // namespace uav
