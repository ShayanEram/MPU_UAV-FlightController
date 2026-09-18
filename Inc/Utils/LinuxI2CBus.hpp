#pragma once
#include "HardwareAbstractions.hpp"
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <memory>

namespace uav::hw {

class LinuxI2CBus : public I2CBus {
public:
    explicit LinuxI2CBus(const std::string& dev = "/dev/i2c-1");
    ~LinuxI2CBus() override;

    bool openBus();
    void closeBus();

    bool read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) override;

private:
    std::string _dev;
    int _fd{-1};
};

} // namespace uav::hw
