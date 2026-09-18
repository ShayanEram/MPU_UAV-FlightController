#include "PigpioPwmOutput.hpp"
#include <pigpio.h>
#include <iostream>

using namespace uav::hw;

PigpioPwmOutput::PigpioPwmOutput(const std::vector<unsigned>& gpioPins) : _pins(gpioPins) {
    _initialized = initPigpio();
    if (!_initialized) {
        std::cerr << "[PigpioPwmOutput] pigpio init failed; PWM disabled\n";
    }
}

PigpioPwmOutput::~PigpioPwmOutput() {
    shutdownPigpio();
}

bool PigpioPwmOutput::initPigpio() {
    if (gpioInitialise() < 0) return false;
    for (auto p : _pins) {
        gpioSetMode(p, PI_OUTPUT);
        // set servo pulse range default
        gpioServo(p, 1500); // neutral
    }
    return true;
}

void PigpioPwmOutput::shutdownPigpio() {
    if (_initialized) {
        for (auto p : _pins) gpioServo(p, 0);
        gpioTerminate();
        _initialized = false;
    }
}

bool PigpioPwmOutput::setPulseWidth(size_t channel, uint16_t pulseWidthUs) {
    if (!_initialized) return false;
    if (channel >= _pins.size()) return false;
    unsigned pin = _pins[channel];
    // pigpio uses microseconds for servo pulses
    int r = gpioServo(pin, pulseWidthUs);
    return r == 0;
}

bool PigpioPwmOutput::setDutyCycle(size_t channel, float duty) {
    // convert duty 0..1 to pulse 1000..2000
    float d = std::max(0.0f, std::min(1.0f, duty));
    uint16_t pulse = static_cast<uint16_t>(1000 + d * 1000);
    return setPulseWidth(channel, pulse);
}
