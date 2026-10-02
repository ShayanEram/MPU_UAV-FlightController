#include "RemoteController.hpp"

#include <spdlog/spdlog.h>

RemoteController::RemoteController(Config& cfg) : m_cfg(std::move(cfg)) {}

RemoteController::~RemoteController() {}

//------------------------------------------------------------------------------------
bool RemoteController::Initialize() {
    if (!m_cfg.m_rc_input) {
        spdlog::debug("[RemoteController] Warning: no RC adapter injected; using simulated RC");
    }
    return true;
}
//------------------------------------------------------------------------------------
void RemoteController::StepRC() {
    const auto period = std::chrono::milliseconds(1000 / std::max<uint32_t>(1, m_cfg.m_loop_hz));
    RemoteData r;
    if (ReadOnce(r) && m_callback) {
        m_callback(r);
    }
    std::this_thread::sleep_for(period);
}
//------------------------------------------------------------------------------------
void RemoteController::SetRemoteCallback(RemoteCallback cb) {
    m_callback = std::move(cb);
}

bool RemoteController::ReadOnce(RemoteData& out) {
    if (m_cfg.m_rc_input) {
        std::vector<float> channels;
        if (!m_cfg.m_rc_input->ReadChannels(channels)) {
            return false;
        }
        out = MapChannelsToRemote(channels);
    }
    else {
        // simulated neutral RC
        out.m_throttle     = 0.0F;
        out.m_roll         = 0.0F;
        out.m_pitch        = 0.0F;
        out.m_yaw          = 0.0F;
        out.m_mode_switch  = false;
        out.m_kill_switch  = false;
        out.m_is_connected = true;
    }
    out.m_timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return true;
}

RemoteData RemoteController::MapChannelsToRemote(const std::vector<float>& channels) {
    RemoteData r;
    // Expect channels: throttle, aileron, elevator, rudder, switches...
    if (channels.size() >= 4) {
        r.m_throttle     = channels[0];
        r.m_roll         = channels[1];
        r.m_pitch        = channels[2];
        r.m_yaw          = channels[3];
        r.m_is_connected = true;
    }
    else {
        r.m_is_connected = false;
    }
    // map additional channels to switches if present
    if (channels.size() >= 5) {
        r.m_mode_switch = channels[4] > 0.5f;
    }
    if (channels.size() >= 6) {
        r.m_kill_switch = channels[5] > 0.5f;
    }
    return r;
}