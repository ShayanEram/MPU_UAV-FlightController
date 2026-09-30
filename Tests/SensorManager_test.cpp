#include <gtest/gtest.h>

#include "MessageQueue.hpp"
#include "Observer.hpp"
#include "SensorManager.hpp"

class TestableSensorManager : public SensorManager {
  public:
    using SensorManager::readGPSData;
    using SensorManager::SensorManager;
};

TEST(SensorManagerTest, ReadGPSDataCallable) {
    MessageQueue<SensorData> dummyQueue;
    Observer<SensorData>     dummyObserver;
    TestableSensorManager    manager(dummyQueue, dummyObserver);

    EXPECT_NO_THROW(manager.readGPSData());
}