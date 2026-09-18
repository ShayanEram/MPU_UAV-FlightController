#pragma once
#include <cstdint>
#include <vector>

namespace uav::hw {

// Generic I2C bus abstraction
class I2CBus {
public:
    virtual ~I2CBus() = default;
    virtual bool read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) = 0;
};

// Generic UART port abstraction
class UartPort {
public:
    virtual ~UartPort() = default;
    virtual bool open(const char* device, uint32_t baud) = 0;
    virtual void close() = 0;
    virtual ssize_t read(uint8_t* buf, size_t len, uint32_t timeoutMs) = 0;
    virtual ssize_t write(const uint8_t* buf, size_t len) = 0;
};

// PWM output abstraction for servos/ESCs
class PwmOutput {
public:
    virtual ~PwmOutput() = default;
    // channel index 0..N-1, pulseWidth in microseconds (1000..2000 typical)
    virtual bool setPulseWidth(size_t channel, uint16_t pulseWidthUs) = 0;
    virtual bool setDutyCycle(size_t channel, float duty) = 0; // 0..1
};

// ADC reader abstraction for direct voltage/current sensing
class AdcReader {
public:
    virtual ~AdcReader() = default;
    virtual bool readVoltage(float& volts) = 0;
    virtual bool readCurrent(float& amps) = 0;
};

// PPM / SBUS input abstraction for RC receivers
class RcInput {
public:
    virtual ~RcInput() = default;
    // Read raw channel values normalized to -1..1 or 0..1 depending on implementation
    virtual bool readChannels(std::vector<float>& channels) = 0;
};

} // namespace uav::hw
