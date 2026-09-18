#pragma once
#include "HardwareAbstractions.hpp"
#include <string>
#include <termios.h>

namespace uav::hw {

class LinuxUartPort : public UartPort {
public:
    explicit LinuxUartPort(const std::string& device);
    ~LinuxUartPort() override;

    bool open(const char* device, uint32_t baud) override;
    void close() override;
    ssize_t read(uint8_t* buf, size_t len, uint32_t timeoutMs) override;
    ssize_t write(const uint8_t* buf, size_t len) override;

private:
    std::string _device;
    int _fd{-1};
    termios _orig{};
};

} // namespace uav::hw
