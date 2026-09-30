#include <gtest/gtest.h>

#include "MotorController.hpp"
#include "SharedResource.hpp"

class TestableMotorController : public MotorController {
  public:
    using MotorController::MotorController;
    using MotorController::MotorID;
    using MotorController::setMotorSpeed;
};

TEST(MotorControllerTest, SetMotorSpeedClampsToMinAndMax) {
    SharedResource<MotorData> dummyMotorData;
    TestableMotorController   controller(dummyMotorData);

    // Test throttle below minimum
    int minSpeed = controller.setMotorSpeed(TestableMotorController::MotorID::MOTOR_1, -1.0f);
    EXPECT_EQ(minSpeed, 0); // MIN_SPEED is 0

    // Test throttle above maximum
    int maxSpeed = controller.setMotorSpeed(TestableMotorController::MotorID::MOTOR_1, 2.0f);
    EXPECT_EQ(maxSpeed, 255); // MAX_SPEED is 255

    // Test throttle in normal range
    int normalSpeed = controller.setMotorSpeed(TestableMotorController::MotorID::MOTOR_1, 0.5f);
    EXPECT_GE(normalSpeed, 0);
    EXPECT_LE(normalSpeed, 255);
}