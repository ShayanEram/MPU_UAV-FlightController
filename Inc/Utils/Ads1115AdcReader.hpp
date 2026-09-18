#pragma once
#include "HardwareAbstractions.hpp"
#include <memory>

namespace uav::hw {

class Ads1115AdcReader : public AdcReader {
public:
    // i2c: injected LinuxI2CBus; addr default 0x48
    Ads1115AdcReader(std::shared_ptr<I2CBus> i2c, uint8_t addr = 0x48);
    ~Ads1115AdcReader() override = default;

    bool readVoltage(float& volts) override;
    bool readCurrent(float& amps) override;

private:
    std::shared_ptr<I2CBus> _i2c;
    uint8_t _addr;
    // conversion helpers
    bool readRawChannel(int channel, int16_t& out);
};
