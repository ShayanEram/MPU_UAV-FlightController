#ifndef BATTERYMANAGER_HPP
#define BATTERYMANAGER_HPP

#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>

#include "InterData.hpp"

class BatteryManager {
  public:
    explicit BatteryManager(MessageQueue<BatteryData>& batteryQueue);
    ~BatteryManager();
    void start();
    void stop();

  protected:
    struct BatteryStatus {
        bool healthy;
        bool charging;
        bool batteryFull;
        bool batteryLow;
        bool batteryOverheating;
    };

    float         getBatteryVolatage() const;
    float         getBatteryCurrent() const;
    float         getBatteryTemperature() const;
    float         predictBatteryLife(float voltage, float current) const;
    BatteryStatus getBatteryStatus(float voltage, float current, float temperature, float life) const;
    void          emergencyShutdown() const;

    const std::string BATTERY_STATUS_VOLTAGE_PATH     = "/sys/class/power_supply/BAT0/status";
    const std::string BATTERY_STATUS_CURRENT_PATH     = "/sys/class/power_supply/BAT0/current_now";
    const std::string BATTERY_STATUS_TEMPERATURE_PATH = "/sys/class/power_supply/BAT0/temp";
    const std::string BATTERY_STATUS_CAPACITY_PATH    = "/sys/class/power_supply/BAT0/capacity";

    static constexpr float BATTERY_VOLTAGE_THRESHOLD{3.0f};
    static constexpr float BATTERY_CURRENT_THRESHOLD{0.5f};
    static constexpr float BATTERY_TEMPERATURE_THRESHOLD{60.0f};
    static constexpr float BATTERY_MAX_CAPACITY{5000.0f};
    static constexpr int   BATTERY_CHECK_INTERVAL_MS{5000};
    static constexpr int   BATTERY_EMERGENCY_SHUTDOWN_DELAY_MS{1000};

    float         _voltage;
    float         _current;
    float         _temperature;
    float         _remainingLife;
    BatteryStatus _status;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    MessageQueue<BatteryData>& _batteryQueue;
};

#endif // BATTERYMANAGER_HPP