#include "Ads1115AdcReader.hpp"
#include <iostream>
#include <cmath>

using namespace uav::hw;

// ADS1115 registers and config (single-shot)
static constexpr uint8_t ADS1115_REG_CONV = 0x00;
static constexpr uint8_t ADS1115_REG_CONFIG = 0x01;
static constexpr uint8_t ADS1115_ADDR_DEFAULT = 0x48;

Ads1115AdcReader::Ads1115AdcReader(std::shared_ptr<I2CBus> i2c, uint8_t addr)
    : _i2c(i2c), _addr(addr) {}

bool Ads1115AdcReader::readRawChannel(int channel, int16_t& out) {
    if (!_i2c) return false;
    // build config for single-shot conversion on channel
    uint16_t config = 0x8000; // OS = 1 (start single)
    // MUX
    uint16_t mux = 0x4000; // AIN0 default
    switch (channel) {
        case 0: mux = 0x4000; break;
        case 1: mux = 0x5000; break;
        case 2: mux = 0x6000; break;
        case 3: mux = 0x7000; break;
        default: mux = 0x4000; break;
    }
    config |= mux;
    config |= 0x0200; // PGA = ±4.096V (example)
    config |= 0x0080; // data rate 128 SPS
    config |= 0x0003; // comparator disabled

    uint8_t cfgBuf[3];
    cfgBuf[0] = ADS1115_REG_CONFIG;
    cfgBuf[1] = static_cast<uint8_t>((config >> 8) & 0xFF);
    cfgBuf[2] = static_cast<uint8_t>(config & 0xFF);
    if (!_i2c->write(_addr, cfgBuf[0], cfgBuf + 1, 2)) {
        std::cerr << "[Ads1115] write config failed\n";
        return false;
    }
    // wait for conversion ~8ms for 128SPS
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t convBuf[2];
    if (!_i2c->read(_addr, ADS1115_REG_CONV, convBuf, 2)) {
        std::cerr << "[Ads1115] read conv failed\n";
        return false;
    }
    out = static_cast<int16_t>((convBuf[0] << 8) | convBuf[1]);
    return true;
}

bool Ads1115AdcReader::readVoltage(float& volts) {
    int16_t raw = 0;
    if (!readRawChannel(0, raw)) return false;
    // PGA ±4.096V => LSB = 125uV
    float lsb = 4.096f / 32768.0f;
    volts = raw * lsb;
    return true;
}

bool Ads1115AdcReader::readCurrent(float& amps) {
    // If current sense uses a shunt + amplifier, map channel 1 to current
    int16_t raw = 0;
    if (!readRawChannel(1, raw)) return false;
    float lsb = 4.096f / 32768.0f;
    float voltage = raw * lsb;
    // Example: shunt amplifier gain and shunt resistor mapping
    // User must calibrate: assume 0.1 ohm shunt and amplifier gain 20 => I = V / (R * G)
    float shuntR = 0.1f;
    float gain = 20.0f;
    amps = voltage / (shuntR * gain);
    return true;
}
