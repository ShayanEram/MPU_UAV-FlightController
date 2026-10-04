#include "Ads1115Adc.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

using namespace HW::IF;

// ADS1115 registers and config (single-shot)
static constexpr uint8_t ADS1115_REG_CONV     = 0x00;
static constexpr uint8_t ADS1115_REG_CONFIG   = 0x01;
static constexpr uint8_t ADS1115_ADDR_DEFAULT = 0x48;

Ads1115Adc::Ads1115Adc(std::shared_ptr<I2C> i2c, uint8_t addr) : m_i2c(std::move(i2c)), m_addr(addr) {}

bool Ads1115Adc::ReadRawChannel(int channel, int16_t& out) {
    if (!m_i2c) {
        return false;
    }
    // build config for single-shot conversion on channel
    uint16_t config = 0x8000; // OS = 1 (start single)
    // MUX
    uint16_t mux = 0x4000; // AIN0 default
    switch (channel) {
    case 0:
        mux = 0x4000;
        break;
    case 1:
        mux = 0x5000;
        break;
    case 2:
        mux = 0x6000;
        break;
    case 3:
        mux = 0x7000;
        break;
    default:
        mux = 0x4000;
        break;
    }
    config |= mux;
    config |= 0x0200; // PGA = ±4.096V (example)
    config |= 0x0080; // data rate 128 SPS
    config |= 0x0003; // comparator disabled

    std::array<uint8_t, 3> cfg_buf{};
    cfg_buf.at(0) = ADS1115_REG_CONFIG;
    cfg_buf.at(1) = static_cast<uint8_t>((config >> 8) & 0xFF);
    cfg_buf.at(1) = static_cast<uint8_t>(config & 0xFF);
    if (!m_i2c->Write(m_addr, cfg_buf.at(0), cfg_buf.data() + 1, 2)) {
        spdlog::error("[Ads1115] write config failed");
        return false;
    }
    // wait for conversion ~8ms for 128SPS
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::array<uint8_t, 2> conv_buf;
    if (!m_i2c->Read(m_addr, ADS1115_REG_CONV, conv_buf.data(), 2)) {
        spdlog::error("[Ads1115] read conv failed");
        return false;
    }
    out = static_cast<int16_t>((conv_buf.at(0) << 8) | conv_buf.at(1));
    return true;
}

bool Ads1115Adc::ReadVoltage(float& volts) {
    int16_t raw = 0;
    if (!ReadRawChannel(0, raw)) {
        return false;
    }
    // PGA ±4.096V => LSB = 125uV
    float lsb = 4.096f / 32768.0f;
    volts     = raw * lsb;
    return true;
}

bool Ads1115Adc::ReadCurrent(float& amps) {
    // If current sense uses a shunt + amplifier, map channel 1 to current
    int16_t raw = 0;
    if (!ReadRawChannel(1, raw)) {
        return false;
    }
    float lsb     = 4.096f / 32768.0f;
    float voltage = static_cast<float>(raw) * lsb;
    // Example: shunt amplifier gain and shunt resistor mapping
    // User must calibrate: assume 0.1 ohm shunt and amplifier gain 20 => I = V / (R * G)
    float shunt_r = 0.1f;
    float gain    = 20.0f;
    amps          = voltage / (shunt_r * gain);
    return true;
}