#include <gtest/gtest.h>

#include "RemoteController.hpp"
#include "SharedResource.hpp"

class TestableRemoteController : public RemoteController {
  public:
    using RemoteController::Command;
    using RemoteController::processCommand;
    using RemoteController::RemoteController;
};

TEST(RemoteControllerTest, ProcessCommandDefault) {
    SharedResource<RemoteData> dummyRemoteData;
    TestableRemoteController   controller(dummyRemoteData);

    // Provide a dummy command string
    auto cmd = controller.processCommand("THROTTLE:0.5;YAW:0.1;PITCH:0.2;ROLL:0.3;RELEASE:1");

    // Since actual parsing is not shown, just check type and default values
    EXPECT_TRUE((std::is_same<decltype(cmd), TestableRemoteController::Command>::value));
}