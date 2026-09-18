#include "SensorManager.hpp"
#include "BatteryManager.hpp"
#include "RemoteController.hpp"
#include "MotorController.hpp"
#include "TelemetryManager.hpp"
#include "FlightController.hpp"
#include "TelemetryManagerMavlink.hpp"

#include "LinuxI2CBus.hpp"
#include "LinuxUartPort.hpp"
#include "PigpioPwmOutput.hpp"
#include "Ads1115AdcReader.hpp"
#include "SbusRcInput.hpp"

#include <memory>
#include <iostream>

int main() {
    using namespace uav;
    using namespace uav::hw;

    // Hardware adapters
    auto i2c = std::make_shared<LinuxI2CBus>("/dev/i2c-1");
    auto gpsUart = std::make_shared<LinuxUartPort>("/dev/serial0");
    gpsUart->open("/dev/serial0", 38400);
    auto lidarUart = std::make_shared<LinuxUartPort>("/dev/ttyS1");
    lidarUart->open("/dev/ttyS1", 115200);

    // SBUS RC on /dev/ttyAMA1 (example)
    auto rcUart = std::make_shared<LinuxUartPort>("/dev/ttyAMA1");
    rcUart->open("/dev/ttyAMA1", 100000); // SBUS 100k, 8E2 requires termios config; LinuxUartPort uses raw mode

    auto sbus = std::make_shared<SbusRcInput>(rcUart);

    // ADC for battery
    auto ads = std::make_shared<Ads1115AdcReader>(i2c, 0x48);

    // PWM outputs: choose GPIO pins for channels 0..3
    std::vector<unsigned> pwmPins = {18, 19, 20, 21}; // example pins; change to your wiring
    auto pwmOut = std::make_shared<PigpioPwmOutput>(pwmPins);

    // Managers
    SensorManager::Config sCfg;
    sCfg.i2c = i2c;
    sCfg.gpsUart = gpsUart;
    sCfg.lidarUart = lidarUart;
    sCfg.loopHz = 50;
    auto sensorMgr = std::make_unique<SensorManager>(sCfg);
    sensorMgr->init();

    BatteryManager::Config bCfg;
    bCfg.adc = ads;
    bCfg.loopHz = 1;
    auto battMgr = std::make_unique<BatteryManager>(bCfg);
    battMgr->init();

    RemoteController::Config rCfg;
    rCfg.rcInput = sbus;
    rCfg.loopHz = 50;
    auto rc = std::make_unique<RemoteController>(rCfg);
    rc->init();

    MotorController::Config mCfg;
    mCfg.pwm = pwmOut;
    mCfg.motorChannel = 0;
    mCfg.aileronChannel = 1;
    mCfg.elevatorChannel = 2;
    mCfg.rudderChannel = 3;
    auto motor = std::make_unique<MotorController>(mCfg);
    motor->init();

    // TelemetryManager::Config tCfg;
    // tCfg.telemetryUart = gpsUart; // reuse or use separate telemetry port
    // tCfg.sendHz = 2;
    // auto telem = std::make_unique<TelemetryManager>(tCfg);
    // telem->init();

    uav::TelemetryManagerMavlink::Config tCfg;
    tCfg.telemetryUart = telemetryUart; // same hw::UartPort
    tCfg.sendHz = 5;
    tCfg.systemId = 1;
    tCfg.componentId = 1;
    auto telem = std::make_unique<uav::TelemetryManagerMavlink>(tCfg);
    telem->init();
    telem->start();

    FlightController::Config fcCfg;
    fcCfg.loopHz = 50;
    auto fc = std::make_unique<FlightController>(fcCfg);
    fc->init();

    // Wire callbacks
    sensorMgr->setSensorCallback([&](const SensorData& s){
        fc->onSensorUpdate(s);
        TelemetryPacket p; p.sensor = s; telem->send(p);
    });
    battMgr->setBatteryCallback([&](const BatteryData& b){
        fc->onBatteryUpdate(b);
        TelemetryPacket p; p.battery = b; telem->send(p);
    });
    rc->setRemoteCallback([&](const RemoteData& r){
        fc->onRemoteUpdate(r);
        TelemetryPacket p; p.remote = r; telem->send(p);
    });

    fc->setMotorOutputCallback([&](const MotorData& m){
        if (!motor->apply(m)) motor->setFailsafe();
        TelemetryPacket p; p.motor = m; telem->send(p);
    });

    // Start
    sensorMgr->start();
    battMgr->start();
    rc->start();
    telem->send(pkt);
    fc->start();

    std::cout << "Raspberry Pi UAV stack running. Press Enter to stop.\n";
    std::string dummy;
    std::getline(std::cin, dummy);

    // Stop
    sensorMgr->stop();
    battMgr->stop();
    rc->stop();
    telem->stop();
    fc->stop();

    std::cout << "Stopped.\n";
    return 0;
}
