#pragma once
/**
 * @file BatteryManager.hpp
 * @brief Header file for the BatteryManager class.
 * The ESC manager.
 */
#include <memory>

#include "Hardware_Interface/HardwareAbstractions.hpp"
#include "InterData.hpp"

class BatteryManager {
  public:
    struct Config {
        std::shared_ptr<HW::Adc> m_adc;
        float                    m_low_threshold_pct{LOW_THRESH_PCT};
        float                    m_critical_threshold_pct{CRITIC_THRESH_PCT};
        uint32_t                 m_loop_hz{2};
    };

    explicit BatteryManager(Config& cfg);
    ~BatteryManager();

    explicit BatteryManager(const BatteryManager& rhs)   = delete;
    explicit BatteryManager(BatteryManager&& rhs)        = delete;
    BatteryManager& operator=(const BatteryManager& rhs) = delete;
    BatteryManager& operator=(BatteryManager&& rhs)      = delete;

    bool Initialize();
    void SetBatteryCallback(BatteryCallback cb);
    bool ReadOnce(BatteryData& out);

  private:
    void StepBM();

    Config          m_cfg;
    BatteryCallback m_callback;

    static constexpr auto LOW_THRESH_PCT    = 20.0F;
    static constexpr auto CRITIC_THRESH_PCT = 10.0F;
};