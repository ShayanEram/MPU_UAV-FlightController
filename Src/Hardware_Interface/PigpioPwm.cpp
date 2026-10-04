#include "PigpioPwm.hpp"

#include <pigpio.h>
#include <spdlog/spdlog.h>

using namespace HW::IF;

PigpioPwm::PigpioPwm(const std::vector<std::size_t>& gpio_pins) : m_pins(gpio_pins), m_initialized(InitPigpio()) {
    if (!m_initialized) {
        spdlog::error("[PigpioPwm] pigpio init failed; PWM disabled");
    }
}

PigpioPwm::~PigpioPwm() {
    ShutdownPigpio();
}

bool PigpioPwm::InitPigpio() {
    if (gpioInitialise() < 0)
        return false;
    for (auto p : m_pins) {
        gpioSetMode(p, PI_OUTPUT);
        // set servo pulse range default
        gpioServo(p, 1500); // neutral
    }
    return true;
}

void PigpioPwm::ShutdownPigpio() {
    if (m_initialized) {
        for (auto p : m_pins) {
            gpioServo(p, 0);
        }
        gpioTerminate();
        m_initialized = false;
    }
}

bool PigpioPwm::SetPulseWidth(size_t channel, uint16_t pulse_width_us) {
    if (!m_initialized) {
        return false;
    }
    if (channel >= m_pins.size()) {
        return false;
    }
    unsigned pin = m_pins.at(channel);
    // pigpio uses microseconds for servo pulses
    int r = gpioServo(pin, pulse_width_us);
    return r == 0;
}

bool PigpioPwm::SetDutyCycle(size_t channel, float duty) {
    // convert duty 0..1 to pulse 1000..2000
    float d     = std::max(0.0F, std::min(1.0F, duty));
    auto  pulse = static_cast<uint16_t>(1000 + d * 1000);
    return SetPulseWidth(channel, pulse);
}
