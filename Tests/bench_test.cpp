#include "LinuxI2CBus.hpp"
#include "LinuxUartPort.hpp"
#include "PigpioPwmOutput.hpp"
#include "Ads1115AdcReader.hpp"
#include "SbusRcInput.hpp"

#include "SensorManager.hpp"
#include "BatteryManager.hpp"
#include "RemoteController.hpp"
#include "MotorController.hpp"
#include "MavlinkReceiver.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace uav;
using namespace uav::hw;
using namespace std::chrono;

int main() {
    // Adapters
    auto i2c = std::make_shared<LinuxI2CBus>("/dev/i2c-1");
    auto telemetryUart = std::make_shared<LinuxUartPort>("/dev/serial0");
    telemetryUart->open("/dev/serial0", 57600);
    std::vector<unsigned> pwmPins = {18, 19, 20, 21};
    auto pwm = std::make_shared<PigpioPwmOutput>(pwmPins);

    // Motor controller
    MotorController::Config mcCfg;
    mcCfg.pwm = pwm;
    mcCfg.motorChannel = 0;
    mcCfg.aileronChannel = 1;
    mcCfg.elevatorChannel = 2;
    mcCfg.rudderChannel = 3;
    mcCfg.motorMinUs = 1000;
    mcCfg.motorMaxUs = 2000;
    mcCfg.servoMinUs = 1000;
    mcCfg.servoMaxUs = 2000;
    mcCfg.throttleRampRate = 0.5f;
    auto motor = std::make_unique<MotorController>(mcCfg);
    motor->init();

    // Mavlink receiver
    MavlinkReceiver::Config mrCfg;
    mrCfg.uart = telemetryUart;
    MavlinkReceiver mavRx(mrCfg);
    mavRx.init();

    // Arm callback: call motor->arm/disarm
    mavRx.setArmCallback([&](bool arm){
        if (arm) motor->arm();
        else motor->disarm();
    });

    mavRx.start();

    std::cout << "Bench test: sweeping servos (props off). Press Enter to start sweep...\n";
    std::string dummy; std::getline(std::cin, dummy);

    // Servo sweep
    for (int i = 0; i <= 100; i += 5) {
        float v = (i / 100.0f) * 2.0f - 1.0f; // -1..1
        MotorData m;
        m.motorThrottle = 0.0f;
        m.servoAileron = v;
        m.servoElevator = v;
        m.servoRudder = v;
        motor->apply(m);
        std::this_thread::sleep_for(milliseconds(100));
    }

    std::cout << "Servo sweep done. Now testing arming sequence.\n";
    std::cout << "Attempting to set throttle to 0.5 while disarmed (should remain 0)\n";
    MotorData test;
    test.motorThrottle = 0.5f;
    test.servoAileron = 0.0f;
    motor->apply(test);
    std::this_thread::sleep_for(seconds(2));

    std::cout << "Arming now (via API). Ensure props are removed.\n";
    motor->arm();
    std::this_thread::sleep_for(seconds(1));

    std::cout << "Ramping throttle to 0.5 over 5 seconds\n";
    for (int i = 0; i <= 50; ++i) {
        MotorData m;
        m.motorThrottle = i / 100.0f;
        m.servoAileron = 0.0f;
        motor->apply(m);
        std::this_thread::sleep_for(milliseconds(100));
    }

    std::cout << "Ramping down and disarming\n";
    for (int i = 50; i >= 0; --i) {
        MotorData m;
        m.motorThrottle = i / 100.0f;
        motor->apply(m);
        std::this_thread::sleep_for(milliseconds(50));
    }
    motor->disarm();

    mavRx.stop();
    std::cout << "Bench test complete.\n";
    return 0;
}
