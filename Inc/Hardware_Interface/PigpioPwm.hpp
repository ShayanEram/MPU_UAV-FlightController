#pragma once

#include <vector>

#include "HardwareAbstractions.hpp"

namespace HW::IF {
    class PigpioPwm : public Pwm {
      public:
        // channels: vector of GPIO pins for channels 0..N-1
        explicit PigpioPwm(const std::vector<std::size_t>& gpio_pins);
        ~PigpioPwm() override;

        explicit PigpioPwm(const PigpioPwm& rhs)   = delete;
        explicit PigpioPwm(PigpioPwm&& rhs)        = delete;
        PigpioPwm& operator=(const PigpioPwm& rhs) = delete;
        PigpioPwm& operator=(PigpioPwm&& rhs)      = delete;

        bool SetPulseWidth(size_t channel, uint16_t pulse_width_us) override;
        bool SetDutyCycle(size_t channel, float duty) override;

      private:
        std::vector<std::size_t> m_pins;
        bool                     m_initialized{false};
        bool                     InitPigpio();
        void                     ShutdownPigpio();
    };
} // namespace HW::IF