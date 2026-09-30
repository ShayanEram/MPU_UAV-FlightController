#pragma once
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <string>

#include "HardwareAbstractions.hpp"

namespace HW {
    class I2CBus : public I2C {
      public:
        explicit I2CBus(const std::string& dev = I2C_DEV_PATH);
        ~I2CBus() override;

        explicit I2CBus(const I2CBus& rhs)   = delete;
        explicit I2CBus(I2CBus&& rhs)        = delete;
        I2CBus& operator=(const I2CBus& rhs) = delete;
        I2CBus& operator=(I2CBus&& rhs)      = delete;

        bool OpenBus();
        void CloseBus();

        bool Read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override;
        bool Write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) override;

      private:
        std::string m_dev;
        int         m_fd{-1};

        static constexpr auto I2C_DEV_PATH = "/dev/i2c-1";
    };
} // namespace HW