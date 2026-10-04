#include <gtest/gtest.h>

#include "BatteryManager.hpp"
#include "MessageQueue.hpp"

class TestableBatteryManager : public BatteryManager {
  public:
    using BatteryManager::BatteryManager;
    using BatteryManager::getBatteryStatus;
};

TEST(BatteryManagerTest, GetBatteryStatusLogic) {
    MessageQueue<BatteryData> dummyQueue;
    TestableBatteryManager    manager(dummyQueue);

    // Test: healthy battery
    float voltage       = 4.0f;
    float current       = 1.0f;
    float temperature   = 25.0f;
    float remainingLife = 2.0f;

    auto status = manager.getBatteryStatus(voltage, current, temperature, remainingLife);

    EXPECT_TRUE(status.healthy);
    EXPECT_FALSE(status.batteryLow);
    EXPECT_FALSE(status.batteryOverheating);
    EXPECT_FALSE(status.batteryFull);
    EXPECT_FALSE(status.charging);
}