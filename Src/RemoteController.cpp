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
        out.throttle     = 0.0F;
        out.roll         = 0.0F;
        out.pitch        = 0.0F;
        out.yaw          = 0.0F;
        out.mode_switch  = false;
        out.kill_switch  = false;
        out.is_connected = true;
    }
    out.timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return true;
}

RemoteData RemoteController::MapChannelsToRemote(const std::vector<float>& channels) {
    RemoteData r;
    // Expect channels: throttle, aileron, elevator, rudder, switches...
    if (channels.size() >= 4) {
        r.throttle     = channels[0];
        r.roll         = channels[1];
        r.pitch        = channels[2];
        r.yaw          = channels[3];
        r.is_connected = true;
    }
    else {
        r.is_connected = false;
    }
    // map additional channels to switches if present
    if (channels.size() >= 5) {
        r.mode_switch = channels[4] > 0.5F;
    }
    if (channels.size() >= 6) {
        r.kill_switch = channels[5] > 0.5F;
    }
    return r;
}