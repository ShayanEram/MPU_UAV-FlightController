#pragma once
#include "HardwareAbstractions.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <atomic>

extern "C" {
#include "mavlink/v2.0/common/mavlink.h"
}

namespace uav {

class MavlinkReceiver {
public:
    using CommandCallback = std::function<void(const mavlink_command_long_t&)>;
    using ArmCallback = std::function<void(bool arm)>; // true=arm, false=disarm
    using ParamSetCallback = std::function<void(const mavlink_param_set_t&)>;

    struct Config {
        std::shared_ptr<uav::hw::UartPort> uart;
        uint8_t systemId{1};
        uint8_t componentId{1};
    };

    explicit MavlinkReceiver(const Config& cfg);
    ~MavlinkReceiver();

    bool init();
    void start();
    void stop();

    void setCommandCallback(CommandCallback cb);
    void setArmCallback(ArmCallback cb);
    void setParamSetCallback(ParamSetCallback cb);

private:
    void runLoop();
    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;

    CommandCallback _cmdCb;
    ArmCallback _armCb;
    ParamSetCallback _paramCb;
};

} // namespace uav
