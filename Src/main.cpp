#include <spdlog/spdlog.h>

#include <memory>

#include "Ads1115Adc.hpp"
#include "BatteryManager.hpp"
#include "FlightController.hpp"
#include "I2CBus.hpp"
#include "MotorController.hpp"
#include "PigpioPwm.hpp"
#include "RemoteController.hpp"
#include "SbusRc.hpp"
#include "SensorManager.hpp"
#include "TelemetryManager.hpp"
#include "UartPort.hpp"

int main() {
    using namespace HW::IF;

    // Hardware adapters
    auto i2c            = std::make_shared<I2CBus>("/dev/i2c-1");
    auto telemetry_uart = std::make_shared<UartPort>("/dev/serial0");
    telemetry_uart->Open("/dev/serial0", 38400);
    auto lidar_uart = std::make_shared<UartPort>("/dev/ttyS1");
    lidar_uart->Open("/dev/ttyS1", 115200);

    // SBUS RC on /dev/ttyAMA1 (example)
    auto rc_uart = std::make_shared<UartPort>("/dev/ttyAMA1");
    rc_uart->Open("/dev/ttyAMA1", 100000); // SBUS 100k, 8E2 requires termios config; LinuxUartPort uses raw mode

    auto sbus = std::make_shared<SbusRc>(rc_uart);

    // ADC for battery
    auto ads = std::make_shared<Ads1115Adc>(i2c, 0x48);

    // PWM outputs: choose GPIO pins for channels 0..3
    std::vector<std::size_t> pwm_pins = {18, 19, 20, 21}; // example pins; change to your wiring
    auto                     pwm_out  = std::make_shared<PigpioPwm>(pwm_pins);

    // Managers
    SensorManager::Config s_cfg;
    s_cfg.m_i2c        = i2c;
    s_cfg.m_gps_uart   = telemetry_uart;
    s_cfg.m_lidar_uart = lidar_uart;
    s_cfg.m_loop_hz    = 50;
    auto sensor_mgr    = std::make_unique<SensorManager>(s_cfg);
    sensor_mgr->Initialize();

    BatteryManager::Config b_cfg;
    b_cfg.m_adc     = ads;
    b_cfg.m_loop_hz = 1;
    auto batt_mgr   = std::make_unique<BatteryManager>(b_cfg);
    batt_mgr->Initialize();

    RemoteController::Config r_cfg;
    r_cfg.m_rc_input = sbus;
    r_cfg.m_loop_hz  = 50;
    auto rc          = std::make_unique<RemoteController>(r_cfg);
    rc->Initialize();

    MotorController::Config m_cfg;
    m_cfg.m_pwm              = pwm_out;
    m_cfg.m_motor_channel    = 0;
    m_cfg.m_aileron_channel  = 1;
    m_cfg.m_elevator_channel = 2;
    m_cfg.m_rudder_channel   = 3;
    auto motor               = std::make_unique<MotorController>(m_cfg);
    motor->Initialize();

    TelemetryManagerMavlink::Config t_cfg;
    t_cfg.m_telemetry_uart = telemetry_uart; // same hw::UartPort
    t_cfg.m_send_hz        = 5;
    t_cfg.m_system_id      = 1;
    t_cfg.m_component_id   = 1;
    auto telem             = std::make_unique<TelemetryManagerMavlink>(t_cfg);
    telem->Initialize();
    // telem->Start();

    FlightController::Config fc_cfg;
    fc_cfg.m_loop_hz = 50;
    auto fc          = std::make_unique<FlightController>(fc_cfg);
    fc->Initialize();

    // Wire callbacks
    sensor_mgr->SetSensorCallback([&](const SensorData& s) {
        fc->OnSensorUpdate(s);
        TelemetryPacket p;
        p.m_sensor = s;
        telem->Send(p);
    });
    batt_mgr->SetBatteryCallback([&](const BatteryData& b) {
        fc->OnBatteryUpdate(b);
        TelemetryPacket p;
        p.m_battery = b;
        telem->Send(p);
    });
    rc->SetRemoteCallback([&](const RemoteData& r) {
        fc->OnRemoteUpdate(r);
        TelemetryPacket p;
        p.m_remote = r;
        telem->Send(p);
    });

    fc->SetMotorOutputCallback([&](const MotorData& m) {
        if (!motor->StepMC(m) /*Apply(m)*/) {
            motor->SetFailsafe();
        }
        TelemetryPacket p;
        p.m_motor = m;
        telem->Send(p);
    });

    // Start
    // sensorMgr->start();
    // battMgr->start();
    // rc->start();
    // telem->send(pkt);
    // fc->start();

    spdlog::info("Raspberry Pi UAV stack running. Press Enter to stop.");
    // std::string dummy;
    // std::getline(std::cin, dummy);

    // Stop
    // sensorMgr->stop();
    // battMgr->stop();
    // rc->stop();
    // telem->stop();
    // fc->stop();

    spdlog::info("Stopped.");
    return 0;
}
