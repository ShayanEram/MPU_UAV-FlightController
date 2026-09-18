#pragma once
#include "HardwareAbstractions.hpp"
#include <vector>
#include <memory>

namespace uav::hw {

class SbusRcInput : public RcInput {
public:
    explicit SbusRcInput(std::shared_ptr<UartPort> uart);
    ~SbusRcInput() override = default;

    bool readChannels(std::vector<float>& channels) override;

private:
    std::shared_ptr<UartPort> _uart;
    bool parseSbusFrame(const uint8_t* buf, size_t len, std::vector<int>& outRaw);
};
