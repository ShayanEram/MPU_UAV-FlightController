#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include <memory>
#include <atomic>
#include <thread>

namespace uav {

class TelemetryManager {
public:
    struct Config {
        std::shared_ptr<hw::UartPort> telemetryUart; // TELEM1/2
        uint32_t sendHz{5};
    };

    explicit TelemetryManager(const Config& cfg);
    ~TelemetryManager();

    bool init();
    void start();
    void stop();

    // push telemetry packet to be sent (thread-safe)
    void send(const TelemetryPacket& pkt);

    // optional callback when telemetry ack or command received
    void setTelemetryCallback(TelemetryCallback cb);

private:
    void runLoop();
    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;
    TelemetryCallback _callback;

    // internal queue implementation omitted from header
};

} // namespace uav
