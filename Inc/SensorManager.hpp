#pragma once
/**
 * @file SensorManager.hpp
 * @brief Header file for SensorManager class.
 * Includes GPS, IMU (gyroscope, accelerometer), barometer, and magnetometer for precise movement tracking.
 *
 */
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "InterData.hpp"

class SensorManager {
  public:
    explicit SensorManager();
    ~SensorManager();

    explicit SensorManager(const SensorManager& rhs)   = delete;
    explicit SensorManager(SensorManager&& rhs)        = delete;
    SensorManager& operator=(const SensorManager& rhs) = delete;
    SensorManager& operator=(SensorManager&& rhs)      = delete;

    bool Initialize();

  private:
    void Step();
};