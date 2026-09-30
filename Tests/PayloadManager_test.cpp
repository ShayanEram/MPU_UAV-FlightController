#include <gtest/gtest.h>

#include "Observer.hpp"
#include "PayloadManager.hpp"

class TestablePayloadManager : public PayloadManager {
  public:
    using PayloadManager::getPayloadStatus;
    using PayloadManager::PayloadManager;
    using PayloadManager::PayloadState;
};

TEST(PayloadManagerTest, GetPayloadStatusReturnsErrorByDefault) {
    Observer<PayloadData>  dummyObserver;
    TestablePayloadManager manager(dummyObserver);

    // By default, readGPIO will return -1 (as implemented in PayloadManager)
    auto status = manager.getPayloadStatus();
    EXPECT_EQ(status, TestablePayloadManager::PayloadState::ERROR);
}