#include "I2CBus.hpp"

#include <spdlog/spdlog.h>

#include <cstring>

using namespace HW::IF;

I2CBus::I2CBus(std::string dev) : m_dev(std::move(dev)) {}

I2CBus::~I2CBus() {
    CloseBus();
}

bool I2CBus::OpenBus() {
    if (m_fd >= 0) {
        return true;
    }
    m_fd = ::open(m_dev.c_str(), O_RDWR);
    if (m_fd < 0) {
        spdlog::error("[I2CBus] open failed: {}", ::strerror(errno));
        return false;
    }
    return true;
}

void I2CBus::CloseBus() {
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool I2CBus::Read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!OpenBus()) {
        return false;
    }
    if (::ioctl(m_fd, I2C_SLAVE, addr) < 0) {
        spdlog::error("[I2CBus] ioctl set addr failed\n");
        return false;
    }
    // write register
    uint8_t reg_buf = reg;
    if (::write(m_fd, &reg_buf, 1) != 1) {
        spdlog::error("[I2CBus] write reg failed");
        return false;
    }
    ssize_t r = ::read(m_fd, buf, len);
    return r == static_cast<ssize_t>(len);
}

bool I2CBus::Write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    if (!OpenBus()) {
        return false;
    }
    if (::ioctl(m_fd, I2C_SLAVE, addr) < 0) {
        spdlog::error("[I2CBus] ioctl set addr failed");
        return false;
    }

    std::vector<uint8_t> out(len + 1);
    out[0] = reg;
    if (buf != nullptr && len > 0) {
        std::memcpy(out.data() + 1, buf, len);
    }

    ssize_t w = ::write(m_fd, out.data(), out.size());
    return w == static_cast<ssize_t>(out.size());
}
