#pragma once
#include "InterData.hpp"
#include "HardwareAbstractions.hpp"
#include "MadgwickAHRS.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>
#include <chrono>

namespace uav {

class SensorManager {
public:
    struct Config {
        std::shared_ptr<uav::hw::I2CBus> i2c;          // primary I2C bus
        std::shared_ptr<uav::hw::UartPort> gpsUart;   // GPS UART (Ublox)
        std::shared_ptr<uav::hw::UartPort> lidarUart; // LiDAR UART (TFmini)
        uint32_t loopHz{100};                         // sensor read frequency
        float madgwickBeta{0.12f};                    // AHRS tuning
        // Optional addresses (override defaults)
        uint8_t icmAddr{0x68};
        uint8_t ms5611Addr{0x77};
        uint8_t hmcAddr{0x1E};
        uint8_t airspeedAddr{0x28}; // example
    };

    explicit SensorManager(const Config& cfg);
    ~SensorManager();

    bool init();
    void start();
    void stop();

    // Register callback to receive fused SensorData
    void setSensorCallback(SensorCallback cb);

    // Single-shot synchronous read
    bool readOnce(SensorData& out);

private:
    void runLoop();

    // low-level readers
    void readImu(SensorData& s);
    void readBaro(SensorData& s);
    void readCompass(SensorData& s);
    void readGps(SensorData& s);
    void readAirspeed(SensorData& s);
    void readLidar(SensorData& s);

    // helpers
    bool readI2CRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);
    bool writeI2CRegister(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len);

    Config _cfg;
    std::atomic<bool> _running{false};
    std::thread _thread;
    SensorCallback _callback;
    std::mutex _cbMutex;

    // AHRS
    MadgwickAHRS _ahrs;
    std::chrono::steady_clock::time_point _lastAhrsTime;

    // cached magnetometer raw values for AHRS
    float _magX{0.0f}, _magY{0.0f}, _magZ{0.0f};

    // internal buffers and state
    std::vector<uint8_t> _gpsBuf;
    std::mutex _gpsBufMutex;
};

} // namespace uav
