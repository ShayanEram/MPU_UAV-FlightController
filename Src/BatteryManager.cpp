#include "BatteryManager.hpp"

#include <spdlog/spdlog.h>

BatteryManager::BatteryManager(Config& cfg) : m_cfg(std::move(cfg)) {}

BatteryManager::~BatteryManager() {
    spdlog::debug("BatteryManager stopped!");
}
//------------------------------------------------------------------------------------
bool BatteryManager::Initialize() {
    if (!m_cfg.m_adc) {
        spdlog::debug("[BatteryManager] Warning: no ADC adapter injected; using simulated battery");
    }
    return true;
}
//------------------------------------------------------------------------------------
void BatteryManager::StepBM() {
    const auto  period = std::chrono::milliseconds(1000 / std::max<uint32_t>(1, m_cfg.m_loop_hz));
    BatteryData b;
    ReadOnce(b);
    if (m_callback) {
        m_callback(b);
    }
    std::this_thread::sleep_for(period);
}
//------------------------------------------------------------------------------------
void BatteryManager::SetBatteryCallback(BatteryCallback cb) {
    m_callback = std::move(cb);
}
bool BatteryManager::ReadOnce(BatteryData& out) {
    if (m_cfg.m_adc) {
        float v = 0.0F;
        float a = 0.0F;
        if (m_cfg.m_adc->ReadVoltage(v)) {
            out.m_voltage = v;
        }
        if (m_cfg.m_adc->ReadCurrent(a)) {
            out.m_current = a;
        }
    }
    else {
        // simulated
        out.m_voltage = 11.1F;
        out.m_current = 1.2F;
    }
    // simple SOC estimate
    out.m_remaining_pct = std::max(0.0F, std::min(100.0F, (out.m_voltage - 9.0F) / (12.6F - 9.0F) * 100.0F));
    out.m_is_low        = out.m_remaining_pct < m_cfg.m_low_threshold_pct;
    out.m_is_critical   = out.m_remaining_pct < m_cfg.m_critical_threshold_pct;
    out.m_timestamp_ms  = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return true;
}