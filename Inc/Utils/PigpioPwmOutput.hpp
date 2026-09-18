#pragma once
#include "HardwareAbstractions.hpp"
#include <vector>
#include <memory>

namespace uav::hw {

class PigpioPwmOutput : public PwmOutput {
public:
    // channels: vector of GPIO pins for channels 0..N-1
    explicit PigpioPwmOutput(const std::vector<unsigned>& gpioPins);
    ~PigpioPwmOutput() override;

    bool setPulseWidth(size_t channel, uint16_t pulseWidthUs) override;
    bool setDutyCycle(size_t channel, float duty) override;

private:
    std::vector<unsigned> _pins;
    bool _initialized{false};
    bool initPigpio();
    void shutdownPigpio();
};

} // namespace uav::hw
