#pragma once
#include <memory>
#include <vector>

#include "HardwareAbstractions.hpp"

namespace HW {
    class SbusRc : public Rc {
      public:
        explicit SbusRc(std::shared_ptr<Uart> uart);
        ~SbusRc() override = default;

        explicit SbusRc(const SbusRc& rhs)   = delete;
        explicit SbusRc(SbusRc&& rhs)        = delete;
        SbusRc& operator=(const SbusRc& rhs) = delete;
        SbusRc& operator=(SbusRc&& rhs)      = delete;

        bool ReadChannels(std::vector<float>& channels) override;

      private:
        std::shared_ptr<Uart> m_uart;
        bool                  ParseSbusFrame(const uint8_t* buf, size_t len, std::vector<int>& out_raw);
    };
} // namespace HW