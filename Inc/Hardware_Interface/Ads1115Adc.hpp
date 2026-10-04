#pragma once

#include <memory>

#include "HardwareAbstractions.hpp"

namespace HW::IF {
    class Ads1115Adc : public Adc {
      public:
        Ads1115Adc(std::shared_ptr<I2C> i2c, uint8_t addr = I2C_DEFAULT_ADDR);
        ~Ads1115Adc() override = default;

        explicit Ads1115Adc(const Ads1115Adc& rhs)   = delete;
        explicit Ads1115Adc(Ads1115Adc&& rhs)        = delete;
        Ads1115Adc& operator=(const Ads1115Adc& rhs) = delete;
        Ads1115Adc& operator=(Ads1115Adc&& rhs)      = delete;

        bool ReadVoltage(float& volts) override;
        bool ReadCurrent(float& amps) override;

      private:
        std::shared_ptr<I2C> m_i2c;
        uint8_t              m_addr;

        // conversion helpers
        bool ReadRawChannel(int channel, int16_t& out);

        static constexpr auto I2C_DEFAULT_ADDR = 0x48;
    };
} // namespace HW::IF