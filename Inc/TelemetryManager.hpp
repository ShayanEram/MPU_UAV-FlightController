/**
 * @file TelemetryManager.hpp
 * @brief Header file for the TelemetryManager class.
 * Provides real-time data on altitude, speed, battery status, and environmental conditions.
 */

#ifndef TELEMETRY_MANAGER_HPP
#define TELEMETRY_MANAGER_HPP

#include <atomic>
#include <iostream>
#include <thread>

#include "InterData.hpp"

class TelemetryManager {
  public:
    explicit TelemetryManager(Observer<SensorData>& telemetryObserver);
    ~TelemetryManager();

    void start();
    void stop();

  protected:
    void updateTelemetryData();
    void sendTelemetryData();

    static constexpr int TELEMETRY_UPDATE_INTERVAL_MS = 1000;

    std::string _telemetryPacket;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    Observer<SensorData>& _telemetryObserver;
};

#endif // TELEMETRY_MANAGER_HPP