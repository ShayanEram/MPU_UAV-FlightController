#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include <memory>
#include <atomic>
#include <thread>

namespace uav {

class BatteryManager {
public:
    struct Config {
        std::shared_ptr<hw::AdcReader> adc;
        float lowThresholdPct{20.0f};
        float criticalThresholdPct{10.0f};
        uint32_t loopHz{2};
    };

    explicit BatteryManager(const Config& cfg);
    ~BatteryManager();

    bool init();
    void start();
    void stop();

    void setBatteryCallback(BatteryCallback cb);
    bool readOnce(BatteryData& out);

private:
    void runLoop();
    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;
    BatteryCallback _callback;
};

} // namespace uav
