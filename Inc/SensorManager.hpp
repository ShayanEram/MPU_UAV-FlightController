#pragma once
/**
 * @file SensorManager.hpp
 * @brief Header file for SensorManager class.
 * Includes GPS, IMU (gyroscope, accelerometer), barometer, and magnetometer for precise movement tracking.
 *
 */
#include <chrono>
#include <cstdint>
#include <memory>

#include "Hardware_Interface/HardwareAbstractions.hpp"
#include "Hardware_Interface/MadgwickAHRS.hpp"
#include "InterData.hpp"

class SensorManager {
  public:
    struct Config {
        std::shared_ptr<HW::I2C>  m_i2c;                         // primary I2C bus
        std::shared_ptr<HW::Uart> m_gps_uart;                    // GPS UART (Ublox)
        std::shared_ptr<HW::Uart> m_lidar_uart;                  // LiDAR UART (TFmini)
        uint32_t                  m_loop_hz{SENSOR_READ_FREQ};   // sensor read frequency
        float                     m_madgwick_beta{AHRS_TUNINGS}; // AHRS tuning
        // Optional addresses (override defaults)
        uint8_t m_icm_addr{0x68};
        uint8_t m_ms5611_addr{0x77};
        uint8_t m_hmc_addr{0x1E};
        uint8_t airspeed_addr{0x28}; // example
    };
    explicit SensorManager(const Config& cfg);
    ~SensorManager();

    explicit SensorManager(const SensorManager& rhs)   = delete;
    explicit SensorManager(SensorManager&& rhs)        = delete;
    SensorManager& operator=(const SensorManager& rhs) = delete;
    SensorManager& operator=(SensorManager&& rhs)      = delete;

    bool Initialize();

    // Register callback to receive fused SensorData
    void SetSensorCallback(SensorCallback cb);

    // Single-shot synchronous read
    bool ReadOnce(SensorData& out);

  private:
    void StepSm();

    // low-level readers
    void ReadImu(SensorData& s);
    void ReadBaro(SensorData& s);
    void ReadCompass(SensorData& s);
    void ReadGps(SensorData& s);
    void ReadAirspeed(SensorData& s);
    void ReadLidar(SensorData& s);

    // helpers
    bool ReadI2CRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);
    bool WriteI2CRegister(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len);

    Config         m_cfg;
    SensorCallback m_callback;
    std::mutex     m_cb_mutex;

    // AHRS
    HW::IF::MadgwickAHRS                  m_ahrs;
    std::chrono::steady_clock::time_point m_last_ahrs_time;

    // cached magnetometer raw values for AHRS
    float m_mag_x{0.0F}, m_mag_y{0.0F}, m_mag_z{0.0F};

    // internal buffers and state
    std::vector<uint8_t> m_gps_buf;
    std::mutex           m_gps_buf_mutex;

    static constexpr auto SENSOR_READ_FREQ = 100;
    static constexpr auto AHRS_TUNINGS     = 0.12F;
    static constexpr auto BUFFER_SIZE      = 1024;
    static constexpr auto GPS_BAUD_RATE    = 38400;
    static constexpr auto LIDAR_BAUD_RATE  = 115200;
};