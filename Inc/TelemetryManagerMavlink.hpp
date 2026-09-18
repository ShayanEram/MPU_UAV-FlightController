#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <cstdint>

extern "C" {
#include "mavlink/v2.0/common/mavlink.h"
}

namespace uav {

class TelemetryManagerMavlink {
public:
    struct Config {
        std::shared_ptr<uav::hw::UartPort> telemetryUart;
        uint32_t sendHz{5};
        uint8_t systemId{1};
        uint8_t componentId{1};
        uint8_t targetSystem{255};   // 255 = broadcast
        uint8_t targetComponent{190};
    };

    explicit TelemetryManagerMavlink(const Config& cfg);
    ~TelemetryManagerMavlink();

    bool init();
    void start();
    void stop();

    // push telemetry packet to be sent (thread-safe)
    void send(const TelemetryPacket& pkt);

private:
    void runLoop();
    void sendHeartbeat();
    void sendAttitude(const SensorData& s);
    void sendGlobalPosition(const SensorData& s);
    void sendSysStatus(const BatteryData& b);
    void sendBatteryStatus(const BatteryData& b);
    void sendRcChannels(const RemoteData& r);

    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;

    // internal queue
    struct QueueItem { TelemetryPacket pkt; };
    std::mutex _qmut;
    std::deque<QueueItem> _queue;

    uint8_t _seqCounter{0};
};

} // namespace uav
