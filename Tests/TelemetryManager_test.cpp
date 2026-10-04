#include <gtest/gtest.h>

#include "Observer.hpp"
#include "TelemetryManager.hpp"

class TestableTelemetryManager : public TelemetryManager {
  public:
    using TelemetryManager::_telemetryPacket;
    using TelemetryManager::TelemetryManager;
    using TelemetryManager::updateTelemetryData;
};

TEST(TelemetryManagerTest, UpdateTelemetryDataSetsPacket) {
    Observer<SensorData>     dummyObserver;
    TestableTelemetryManager manager(dummyObserver);

    manager.updateTelemetryData();
    EXPECT_FALSE(manager._telemetryPacket.empty());
}