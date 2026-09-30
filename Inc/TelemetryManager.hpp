#pragma once
/**
 * @file TelemetryManager.hpp
 * @brief Header file for the TelemetryManager class.
 * Provides real-time data on altitude, speed, battery status, and environmental conditions.
 */

#include <atomic>
#include <thread>

#include "InterData.hpp"

class TelemetryManager {
  public:
    explicit TelemetryManager();
    ~TelemetryManager();

    explicit TelemetryManager(const TelemetryManager& rhs)   = delete;
    explicit TelemetryManager(TelemetryManager&& rhs)        = delete;
    TelemetryManager& operator=(const TelemetryManager& rhs) = delete;
    TelemetryManager& operator=(TelemetryManager&& rhs)      = delete;

    bool Initialize();

  private:
    void Step();
};