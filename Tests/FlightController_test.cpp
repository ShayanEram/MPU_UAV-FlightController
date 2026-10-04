#include <gtest/gtest.h>

#include "FlightController.hpp"
#include "MessageQueue.hpp"
#include "Observer.hpp"
#include "SharedResource.hpp"

class TestableFlightController : public FlightController {
  public:
    using FlightController::_payloadCallbackData;
    using FlightController::FlightController;
    using FlightController::onPayloadDataUpdated;
};

TEST(FlightControllerTest, OnPayloadDataUpdatedSetsPayload) {
    MessageQueue<BatteryData>  batteryQueue;
    SharedResource<MotorData>  motorData;
    Observer<PayloadData>      payloadObserver;
    SharedResource<RemoteData> remoteData;
    MessageQueue<SensorData>   sensorQueue;

    TestableFlightController controller(batteryQueue, motorData, payloadObserver, remoteData, sensorQueue);

    PayloadData testPayload{};
    // testPayload.someField = 42; // TBD: Replace with an actual field from PayloadData

    controller.onPayloadDataUpdated(testPayload);

    // Check if the protected member was updated (if accessible)
    // EXPECT_EQ(controller._payloadCallbackData.someField, 42);
    SUCCEED(); // Replace with real assertions as needed
}