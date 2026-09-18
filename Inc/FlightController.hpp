#pragma once
#include "InterData.hpp"
#include <functional>
#include <atomic>

namespace uav {

class FlightController {
public:
    struct Config {
        // tuning parameters, control loop frequency, failsafe thresholds
        float kpRoll{1.0f}, kiRoll{0.0f}, kdRoll{0.0f};
        float kpPitch{1.0f}, kiPitch{0.0f}, kdPitch{0.0f};
        float kpYaw{1.0f}, kiYaw{0.0f}, kdYaw{0.0f};
        uint32_t loopHz{200};
    };

    explicit FlightController(const Config& cfg);
    ~FlightController();

    // lifecycle
    bool init();
    void start();
    void stop();

    // input sources (called by higher-level orchestrator or callbacks)
    void onSensorUpdate(const SensorData& s);
    void onBatteryUpdate(const BatteryData& b);
    void onRemoteUpdate(const RemoteData& r);

    // output sink (set by orchestrator)
    void setMotorOutputCallback(MotorCallback cb);

    // request current motor outputs
    MotorData getMotorData() const;

private:
    void controlLoopIteration();
    Config _cfg;
    std::atomic<bool> _running{false};

    // internal state
    SensorData _lastSensor;
    BatteryData _lastBattery;
    RemoteData _lastRemote;
    MotorData _lastMotor;

    MotorCallback _motorCb;
};

} // namespace uav
