#include "SbusRc.hpp"

#include <array>
#include <cstring>

using namespace HW::IF;

// SBUS frame constants
static constexpr uint8_t SBUS_START     = 0x0F;
static constexpr uint8_t SBUS_END       = 0x00;
static constexpr size_t  SBUS_FRAME_LEN = 25;

SbusRc::SbusRc(std::shared_ptr<Uart> uart) : m_uart(std::move(uart)) {}

bool SbusRc::ParseSbusFrame(const uint8_t* buf, size_t len, std::vector<int>& out_raw) {
    if (len < SBUS_FRAME_LEN) {
        return false;
    }
    if (buf[0] != SBUS_START) {
        return false;
    }
    // decode 16 channels 11-bit
    out_raw.assign(16, 0);
    out_raw[0]  = ((buf[1] | buf[2] << 8) & 0x07FF);
    out_raw[1]  = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    out_raw[2]  = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    out_raw[3]  = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    out_raw[4]  = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    out_raw[5]  = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    out_raw[6]  = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    out_raw[7]  = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    out_raw[8]  = ((buf[12] | buf[13] << 8) & 0x07FF);
    out_raw[9]  = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
    out_raw[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
    out_raw[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
    out_raw[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
    out_raw[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
    out_raw[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
    out_raw[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);
    return true;
}

bool SbusRc::ReadChannels(std::vector<float>& channels) {
    if (!m_uart) {
        // simulated neutral channels
        channels.assign(8, 0.0F);
        return true;
    }
    std::array<uint8_t, 64> buf{};
    ssize_t                 n = m_uart->Read(buf.data(), sizeof(buf), 20);
    if (n <= 0) {
        return false;
    }
    // find start byte
    for (ssize_t i = 0; i + SBUS_FRAME_LEN <= n; ++i) {
        if (buf.at(i) == SBUS_START) {
            std::vector<int> raw;
            if (ParseSbusFrame(buf.data() + i, SBUS_FRAME_LEN, raw)) {
                channels.clear();
                for (const auto& raw_val : raw) {
                    // SBUS raw 0..2047 -> map to -1..1
                    float v = (static_cast<float>(raw_val) - 1024.0F) / 1024.0F;
                    channels.push_back(v);
                }
                return true;
            }
        }
    }
    return false;
}
