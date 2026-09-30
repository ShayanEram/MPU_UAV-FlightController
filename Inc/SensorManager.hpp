/**
 * @file SensorManager.hpp
 * @brief Header file for SensorManager class.
 * Includes GPS, IMU (gyroscope, accelerometer), barometer, and magnetometer for precise movement tracking.
 *
 */

#ifndef SENSORMANAGER_HPP
#define SENSORMANAGER_HPP

#include <atomic>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "InterData.hpp"

class SensorManager {
  public:
    explicit SensorManager(MessageQueue<SensorData>& sensorQueue, Observer<SensorData>& sensorObserver);
    ~SensorManager();
    void start();
    void stop();

  protected:
    struct SensorReadings {
        // IMU (Gyroscope + Accelerometer)
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;

        // GPS
        double latitude, longitude;
        float  speed; // Velocity (m/s)

        // Barometer (Altitude)
        float altitude;    // Measured altitude in meters
        float airPressure; // Atmospheric pressure in Pascals

        // Magnetometer (Compass)
        float heading; // Orientation (degrees)

        // Battery Monitoring
        float batteryVoltage;
        int   batteryPercentage;
    };

    int  readRegister(std::string file, uint8_t reg);
    void readIMUData(std::string file);
    void readGPSData();
    void readBarometerData(std::string file);
    void readMagnetometerData(std::string file);

    static constexpr int  MPU6050_ADDR             = 0x68;
    static constexpr int  ACCEL_XOUT_H             = 0x3B;
    static constexpr int  GYRO_XOUT_H              = 0x43;
    static constexpr auto GPS_DEVICE               = "/dev/ttyAMA0";
    static constexpr int  BMP280_ADDR              = 0x76;
    static constexpr int  PRESSURE_REG             = 0xF7;
    static constexpr int  HMC5883L_ADDR            = 0x1E;
    static constexpr int  MAG_XOUT_H               = 0x03;
    static constexpr auto IMU_DEVICE_FILE          = "/dev/i2c-1"; // IMU connected via I2C
    static constexpr auto BAROMETER_DEVICE_FILE    = "/dev/i2c-1"; // Barometer connected via I2C
    static constexpr auto MAGNETOMETER_DEVICE_FILE = "/dev/i2c-1"; // Magnetometer connected via I2C

    static constexpr int SENSOR_UPDATE_INTERVAL_MS = 100;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    MessageQueue<SensorData>& _sensorQueue;
    Observer<SensorData>&     _sensorObserver;
};

#endif // SENSORMANAGER_HPP