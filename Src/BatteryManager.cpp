#include "BatteryManager.hpp"

BatteryManager::BatteryManager(MessageQueue<BatteryData>& batteryQueue) : _batteryQueue(batteryQueue), running(false) {
    _voltage       = 0.0f;
    _current       = 0.0f;
    _temperature   = 0.0f;
    _remainingLife = 0.0f;
    _status        = {false, false, false, false, false};
}

BatteryManager::~BatteryManager() {
    stop();
}
//------------------------------------------------------------------------------------
void BatteryManager::start() {
    if (!running.load()) {
        running.store(true);
        moduleThread = std::thread(&BatteryManager::runLoop, this);
    }
}

void BatteryManager::stop() {
    if (running.load()) {
        running.store(false);
        if (moduleThread.joinable()) {
            moduleThread.join();
        }
    }
}
//------------------------------------------------------------------------------------
void BatteryManager::runLoop() {
    while (running.load()) {

        _voltage            = getBatteryVolatage();
        _current            = getBatteryCurrent();
        _temperature        = getBatteryTemperature();
        float remainingLife = predictBatteryLife(_voltage, _current);
        _status             = getBatteryStatus(_voltage, _current, _temperature, remainingLife);
        _remainingLife      = remainingLife;

#ifdef DEBUG
        std::cout << "Battery management operations running..." << std::endl;
        std::cout << "Voltage: " << _voltage << " V, "
                  << "Current: " << _current << " A, "
                  << "Temperature: " << _temperature << " C, "
                  << "Remaining Life: " << _remainingLife << " hours" << std::endl;
        std::cout << "Status - Healthy: " << _status.healthy << ", Charging: " << _status.charging
                  << ", Battery Full: " << _status.batteryFull << ", Battery Low: " << _status.batteryLow
                  << ", Overheating: " << _status.batteryOverheating << std::endl;
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(BATTERY_CHECK_INTERVAL_MS));
    }
}
//------------------------------------------------------------------------------------
float BatteryManager::getBatteryVolatage() const {
    std::ifstream voltageFile(BATTERY_STATUS_VOLTAGE_PATH);
    float         voltage{0.0f};
    if (voltageFile.is_open()) {
        voltageFile >> voltage;
        voltage /= 1000000.0; // Convert microvolts to volts
    }
    return voltage;
}

float BatteryManager::getBatteryCurrent() const {
    std::ifstream currentFile(BATTERY_STATUS_CURRENT_PATH);
    float         current{0.0f};
    if (currentFile.is_open()) {
        currentFile >> current;
        current /= 1000000.0; // Convert microamps to amps
    }
    return current;
}

float BatteryManager::getBatteryTemperature() const {
    std::ifstream tempFile(BATTERY_STATUS_TEMPERATURE_PATH);
    float         temperature{0.0f};
    if (tempFile.is_open()) {
        tempFile >> temperature;
        temperature /= 10.0; // Convert to Celsius
    }
    return temperature;
}

float BatteryManager::predictBatteryLife(float voltage, float current) const {
    std::ifstream capacityFile(BATTERY_STATUS_CAPACITY_PATH);
    float         capacity{0.0f};
    if (capacityFile.is_open()) {
        capacityFile >> capacity;
    }
    else {
        capacity = BATTERY_MAX_CAPACITY;
    }
    float remainingLife = (capacity * (voltage / 4.2f)) / current;
    return remainingLife;
}

BatteryManager::BatteryStatus BatteryManager::getBatteryStatus(float voltage, float current, float temperature,
                                                               float remainingLife) const {
    BatteryStatus status;

    status.healthy            = true;
    status.charging           = false;
    status.batteryFull        = false;
    status.batteryLow         = false;
    status.batteryOverheating = false;

    if (voltage < BATTERY_VOLTAGE_THRESHOLD || remainingLife < 0.1f) {
        status.batteryLow = true;
    }

    if (current < BATTERY_CURRENT_THRESHOLD || temperature > BATTERY_TEMPERATURE_THRESHOLD) {
        status.healthy = false;
    }

    if (temperature > BATTERY_TEMPERATURE_THRESHOLD) {
        status.batteryOverheating = true;
    }

    if (remainingLife <= 0) {
        status.batteryFull = true;
    }

    if (remainingLife > _remainingLife) {
        status.charging = true;
    }

    return status;
}

void BatteryManager::emergencyShutdown() const {
    std::cerr << "Emergency shutdown initiated!\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(BATTERY_EMERGENCY_SHUTDOWN_DELAY_MS));
    std::cerr << "System shutting down...\n";
    // system("shutdown -h now"); // Linux command to shut down
}