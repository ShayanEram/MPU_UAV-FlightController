#include "TelemetryManagerMavlink.hpp"
#include <chrono>
#include <iostream>
#include <cstring>

using namespace uav;
using namespace std::chrono;

static uint64_t now_us() {
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

TelemetryManagerMavlink::TelemetryManagerMavlink(const Config& cfg) : _cfg(cfg) {}

TelemetryManagerMavlink::~TelemetryManagerMavlink() { stop(); }

bool TelemetryManagerMavlink::init() {
    if (!_cfg.telemetryUart) {
        std::cerr << "[TelemetryMavlink] telemetry UART not provided\n";
        return false;
    }
    return true;
}

void TelemetryManagerMavlink::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&TelemetryManagerMavlink::runLoop, this);
}

void TelemetryManagerMavlink::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void TelemetryManagerMavlink::send(const TelemetryPacket& pkt) {
    std::lock_guard<std::mutex> lk(_qmut);
    _queue.push_back({pkt});
    // keep queue bounded
    if (_queue.size() > 200) _queue.pop_front();
}

void TelemetryManagerMavlink::runLoop() {
    const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.sendHz));
    auto lastHeartbeat = steady_clock::now();
    while (_running) {
        // periodic heartbeat at 1 Hz
        if (steady_clock::now() - lastHeartbeat >= seconds(1)) {
            sendHeartbeat();
            lastHeartbeat = steady_clock::now();
        }

        // drain one queued telemetry packet per loop iteration
        QueueItem item;
        {
            std::lock_guard<std::mutex> lk(_qmut);
            if (!_queue.empty()) {
                item = _queue.front();
                _queue.pop_front();
            } else {
                // no queued packet: send periodic status (optional)
                // we still sleep and continue
                std::this_thread::sleep_for(period);
                continue;
            }
        }

        // send fused attitude
        sendAttitude(item.pkt.sensor);

        // send global position
        sendGlobalPosition(item.pkt.sensor);

        // send battery and sys status
        sendSysStatus(item.pkt.battery);
        sendBatteryStatus(item.pkt.battery);

        // send RC channels if present
        sendRcChannels(item.pkt.remote);

        std::this_thread::sleep_for(period);
    }
}

/* Helper: pack and write mavlink message */
static bool write_mavlink_msg(uav::hw::UartPort* uart, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t w = uart->write(buf, len);
    return w == static_cast<ssize_t>(len);
}

void TelemetryManagerMavlink::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(_cfg.systemId, _cfg.componentId, &msg,
        MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}

void TelemetryManagerMavlink::sendAttitude(const SensorData& s) {
    // ATTITUDE expects roll, pitch, yaw in radians and their rates
    mavlink_message_t msg;
    float roll = s.roll * static_cast<float>(M_PI/180.0);
    float pitch = s.pitch * static_cast<float>(M_PI/180.0);
    float yaw = s.yaw * static_cast<float>(M_PI/180.0);
    float rollspeed = s.gyroX;
    float pitchspeed = s.gyroY;
    float yawspeed = s.gyroZ;
    uint32_t time_boot_ms = static_cast<uint32_t>(now_us() / 1000ULL);
    mavlink_msg_attitude_pack(_cfg.systemId, _cfg.componentId, &msg,
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}

void TelemetryManagerMavlink::sendGlobalPosition(const SensorData& s) {
    mavlink_message_t msg;
    // GLOBAL_POSITION_INT uses lat/lon in 1e7, alt in mm
    int32_t lat = static_cast<int32_t>(s.latitude * 1e7);
    int32_t lon = static_cast<int32_t>(s.longitude * 1e7);
    int32_t alt = static_cast<int32_t>(s.altitudeGPS * 1000.0f);
    int32_t relative_alt = static_cast<int32_t>(s.altitudeBaro * 1000.0f);
    int16_t vx = static_cast<int16_t>(s.groundSpeed * 100.0f); // cm/s -> scaled
    int16_t vy = 0;
    int16_t vz = 0;
    uint32_t time_boot_ms = static_cast<uint32_t>(now_us() / 1000ULL);
    mavlink_msg_global_position_int_pack(_cfg.systemId, _cfg.componentId, &msg,
        time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}

void TelemetryManagerMavlink::sendSysStatus(const BatteryData& b) {
    mavlink_message_t msg;
    // SYS_STATUS: onboard_control_sensors_present, enabled, health, load, voltage_battery (mV), current_battery (mA), battery_remaining (%)
    uint32_t onboard_control_sensors_present = 0;
    uint32_t onboard_control_sensors_enabled = 0;
    uint32_t onboard_control_sensors_health = 0;
    uint16_t load = 0;
    uint16_t voltage_battery = static_cast<uint16_t>(b.voltage * 1000.0f);
    int16_t current_battery = static_cast<int16_t>(b.current * 1000.0f);
    int8_t battery_remaining = static_cast<int8_t>(std::max(-1.0f, std::min(100.0f, b.remainingPct)));
    mavlink_msg_sys_status_pack(_cfg.systemId, _cfg.componentId, &msg,
        onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health,
        load, voltage_battery, current_battery, battery_remaining, 0, 0, 0);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}

void TelemetryManagerMavlink::sendBatteryStatus(const BatteryData& b) {
    mavlink_message_t msg;
    // BATTERY_STATUS: pack minimal fields
    uint8_t battery_id = 0;
    int32_t current_consumed = 0;
    int32_t energy_consumed = 0;
    int16_t temperature = 0;
    int32_t voltages[10] = {0};
    voltages[0] = static_cast<int32_t>(b.voltage * 1000.0f);
    int16_t current_battery = static_cast<int16_t>(b.current * 1000.0f);
    int8_t battery_remaining = static_cast<int8_t>(std::max(-1.0f, std::min(100.0f, b.remainingPct)));
    mavlink_msg_battery_status_pack(_cfg.systemId, _cfg.componentId, &msg,
        battery_id, current_consumed, energy_consumed, temperature, voltages, current_battery, battery_remaining);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}

void TelemetryManagerMavlink::sendRcChannels(const RemoteData& r) {
    mavlink_message_t msg;
    // RC_CHANNELS_RAW expects PWM values 0..2000; map normalized -1..1 to 1000..2000
    uint16_t chan1 = static_cast<uint16_t>(1000 + (r.throttle * 500.0f + 500.0f));
    uint16_t chan2 = static_cast<uint16_t>(1500 + r.roll * 500.0f);
    uint16_t chan3 = static_cast<uint16_t>(1500 + r.pitch * 500.0f);
    uint16_t chan4 = static_cast<uint16_t>(1500 + r.yaw * 500.0f);
    uint32_t time_boot_ms = static_cast<uint32_t>(now_us() / 1000ULL);
    mavlink_msg_rc_channels_raw_pack(_cfg.systemId, _cfg.componentId, &msg,
        time_boot_ms, 0, chan1, chan2, chan3, chan4, 0, 0, 0, 0);
    write_mavlink_msg(_cfg.telemetryUart.get(), msg);
}
