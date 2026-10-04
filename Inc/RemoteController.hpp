#pragma once
/**
 * @file RemoteController.hpp
 * @brief Header file for the RemoteController class.
 * Remote control via radio signals, Wi-Fi, or satellite links.
 */

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <memory>

#include "Hardware_Interface/HardwareAbstractions.hpp"
#include "InterData.hpp"

class RemoteController {
  public:
    struct Config {
        std::shared_ptr<HW::Rc> m_rc_input; // PPM/SBUS adapter
        uint32_t                m_loop_hz{UPDATE_FREQ};
    };

    enum class ChannelIndex { CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6 };

    explicit RemoteController(Config& cfg);
    ~RemoteController();

    explicit RemoteController(const RemoteController& rhs)   = delete;
    explicit RemoteController(RemoteController&& rhs)        = delete;
    RemoteController& operator=(const RemoteController& rhs) = delete;
    RemoteController& operator=(RemoteController&& rhs)      = delete;

    bool Initialize();

    void SetRemoteCallback(RemoteCallback cb);
    bool ReadOnce(RemoteData& out);

    void StepRC();

  private:
    RemoteData MapChannelsToRemote(const std::vector<float>& channels);

    Config         m_cfg;
    RemoteCallback m_callback;

    static constexpr auto UPDATE_FREQ = 50;
};