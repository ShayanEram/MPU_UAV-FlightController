#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include <memory>
#include <atomic>
#include <thread>

namespace uav {

class RemoteController {
public:
    struct Config {
        std::shared_ptr<hw::RcInput> rcInput; // PPM/SBUS adapter
        uint32_t loopHz{50};
    };

    explicit RemoteController(const Config& cfg);
    ~RemoteController();

    bool init();
    void start();
    void stop();

    void setRemoteCallback(RemoteCallback cb);
    bool readOnce(RemoteData& out);

private:
    void runLoop();
    RemoteData mapChannelsToRemote(const std::vector<float>& channels);

    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;
    RemoteCallback _callback;
};

} // namespace uav
