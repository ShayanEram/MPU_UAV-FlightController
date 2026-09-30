#pragma once
#include <termios.h>

#include <string>

#include "HardwareAbstractions.hpp"

namespace HW::IF {
    class UartPort : public Uart {
      public:
        explicit UartPort(const std::string& device);
        ~UartPort() override;

        explicit UartPort(const UartPort& rhs)   = delete;
        explicit UartPort(UartPort&& rhs)        = delete;
        UartPort& operator=(const UartPort& rhs) = delete;
        UartPort& operator=(UartPort&& rhs)      = delete;

        bool Open(const char* device, uint32_t baud) override;
        void Close() override;

        ssize_t Read(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
        ssize_t Write(const uint8_t* buf, size_t len) override;

      private:
        std::string m_device;
        int         m_fd{-1};
        termios     m_orig{};
    };
} // namespace HW::IF