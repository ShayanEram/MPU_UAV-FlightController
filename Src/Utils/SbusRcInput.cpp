#include "SbusRcInput.hpp"
#include <iostream>
#include <cstring>

using namespace uav::hw;

// SBUS frame constants
static constexpr uint8_t SBUS_START = 0x0F;
static constexpr uint8_t SBUS_END = 0x00;
static constexpr size_t SBUS_FRAME_LEN = 25;

SbusRcInput::SbusRcInput(std::shared_ptr<UartPort> uart) : _uart(uart) {}

bool SbusRcInput::parseSbusFrame(const uint8_t* buf, size_t len, std::vector<int>& outRaw) {
    if (len < SBUS_FRAME_LEN) return false;
    if (buf[0] != SBUS_START) return false;
    // decode 16 channels 11-bit
    outRaw.assign(16, 0);
    outRaw[0]  = ((buf[1]    | buf[2]<<8) & 0x07FF);
    outRaw[1]  = ((buf[2]>>3 | buf[3]<<5) & 0x07FF);
    outRaw[2]  = ((buf[3]>>6 | buf[4]<<2 | buf[5]<<10) & 0x07FF);
    outRaw[3]  = ((buf[5]>>1 | buf[6]<<7) & 0x07FF);
    outRaw[4]  = ((buf[6]>>4 | buf[7]<<4) & 0x07FF);
    outRaw[5]  = ((buf[7]>>7 | buf[8]<<1 | buf[9]<<9) & 0x07FF);
    outRaw[6]  = ((buf[9]>>2 | buf[10]<<6) & 0x07FF);
    outRaw[7]  = ((buf[10]>>5 | buf[11]<<3) & 0x07FF);
    outRaw[8]  = ((buf[12]    | buf[13]<<8) & 0x07FF);
    outRaw[9]  = ((buf[13]>>3 | buf[14]<<5) & 0x07FF);
    outRaw[10] = ((buf[14]>>6 | buf[15]<<2 | buf[16]<<10) & 0x07FF);
    outRaw[11] = ((buf[16]>>1 | buf[17]<<7) & 0x07FF);
    outRaw[12] = ((buf[17]>>4 | buf[18]<<4) & 0x07FF);
    outRaw[13] = ((buf[18]>>7 | buf[19]<<1 | buf[20]<<9) & 0x07FF);
    outRaw[14] = ((buf[20]>>2 | buf[21]<<6) & 0x07FF);
    outRaw[15] = ((buf[21]>>5 | buf[22]<<3) & 0x07FF);
    return true;
}

bool SbusRcInput::readChannels(std::vector<float>& channels) {
    if (!_uart) {
        // simulated neutral channels
        channels.assign(8, 0.0f);
        return true;
    }
    uint8_t buf[64];
    ssize_t n = _uart->read(buf, sizeof(buf), 20);
    if (n <= 0) return false;
    // find start byte
    for (ssize_t i = 0; i + SBUS_FRAME_LEN <= n; ++i) {
        if (buf[i] == SBUS_START) {
            std::vector<int> raw;
            if (parseSbusFrame(buf + i, SBUS_FRAME_LEN, raw)) {
                channels.clear();
                for (size_t ch = 0; ch < raw.size(); ++ch) {
                    // SBUS raw 0..2047 -> map to -1..1
                    float v = (static_cast<float>(raw[ch]) - 1024.0f) / 1024.0f;
                    channels.push_back(v);
                }
                return true;
            }
        }
    }
    return false;
}
