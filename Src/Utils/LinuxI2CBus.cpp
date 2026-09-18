#include "LinuxI2CBus.hpp"
#include <iostream>
#include <cstring>

using namespace uav::hw;

LinuxI2CBus::LinuxI2CBus(const std::string& dev) : _dev(dev) {}

LinuxI2CBus::~LinuxI2CBus() { closeBus(); }

bool LinuxI2CBus::openBus() {
    if (_fd >= 0) return true;
    _fd = ::open(_dev.c_str(), O_RDWR);
    if (_fd < 0) {
        std::cerr << "[LinuxI2CBus] open failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

void LinuxI2CBus::closeBus() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

bool LinuxI2CBus::read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!openBus()) return false;
    if (ioctl(_fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "[LinuxI2CBus] ioctl set addr failed\n";
        return false;
    }
    // write register
    uint8_t regBuf = reg;
    if (::write(_fd, &regBuf, 1) != 1) {
        std::cerr << "[LinuxI2CBus] write reg failed\n";
        return false;
    }
    ssize_t r = ::read(_fd, buf, len);
    return r == static_cast<ssize_t>(len);
}

bool LinuxI2CBus::write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    if (!openBus()) return false;
    if (ioctl(_fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "[LinuxI2CBus] ioctl set addr failed\n";
        return false;
    }
    std::vector<uint8_t> out;
    out.reserve(len + 1);
    out.push_back(reg);
    out.insert(out.end(), buf, buf + len);
    ssize_t w = ::write(_fd, out.data(), out.size());
    return w == static_cast<ssize_t>(out.size());
}
