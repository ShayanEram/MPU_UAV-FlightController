#include "TelemetryManager.hpp"

#include <spdlog/spdlog.h>

#include <UartPort.hpp>

namespace {
    uint64_t NowUs() {
        return duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    /* Helper: pack and write mavlink message */
    bool WriteMavlinkMsg(HW::Uart* uart, const mavlink_message_t& msg) {
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};

        size_t  len = mavlink_msg_to_send_buffer(buf.data(), &msg);
        ssize_t w   = uart->Write(buf.data(), len);
        return (w == static_cast<ssize_t>(len));
    }

    float DegreeToRad(float degree) {
        return (degree * static_cast<float>(M_PI / 180.0));
    }
} // namespace

TelemetryManagerMavlink::TelemetryManagerMavlink(Config& cfg) : m_cfg(std::move(cfg)) {}

TelemetryManagerMavlink::~TelemetryManagerMavlink() {
    spdlog::debug("TelemetryManagerMavlink stopped!");
}

//------------------------------------------------------------------------------------
bool TelemetryManagerMavlink::Initialize() {
    if (!m_cfg.m_telemetry_uart) {
        spdlog::error("[TelemetryMavlink] telemetry UART not provided");
        return false;
    }
    return true;
}
//------------------------------------------------------------------------------------
void TelemetryManagerMavlink::StepTM() {
    const auto period         = std::chrono::milliseconds(1000 / std::max<uint32_t>(1, m_cfg.m_send_hz));
    auto       last_heartbeat = std::chrono::steady_clock::now();
    // periodic heartbeat at 1 Hz
    if (std::chrono::steady_clock::now() - last_heartbeat >= std::chrono::seconds(1)) {
        SendHeartbeat();
        last_heartbeat = std::chrono::steady_clock::now();
    }

    // drain one queued telemetry packet per loop iteration
    QueueItem item;
    {
        std::scoped_lock lk(m_qmut);
        if (!m_queue.empty()) {
            item = m_queue.front();
            m_queue.pop_front();
        }
        else {
            // no queued packet: send periodic status (optional)
            // we still sleep and continue
            std::this_thread::sleep_for(period);
        }
    }

    // send fused attitude
    SendAttitude(item.m_pkt.sensor);

    // send global position
    SendGlobalPosition(item.m_pkt.sensor);

    // send battery and sys status
    SendSysStatus(item.m_pkt.battery);
    SendBatteryStatus(item.m_pkt.battery);

    // send RC channels if present
    SendRcChannels(item.m_pkt.remote);

    std::this_thread::sleep_for(period);
}
//------------------------------------------------------------------------------------
void TelemetryManagerMavlink::Send(const TelemetryPacket& m_pkt) {
    std::scoped_lock lk(m_qmut);
    m_queue.push_back({m_pkt});
    // keep queue bounded
    if (m_queue.size() > 200) {
        m_queue.pop_front();
    }
}

void TelemetryManagerMavlink::SendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, 0, 0,
                               MAV_STATE_ACTIVE);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendAttitude(const SensorData& s) {
    // ATTITUDE expects roll, pitch, yaw in radians and their rates
    mavlink_message_t msg;
    float             roll         = DegreeToRad(s.roll);
    float             pitch        = DegreeToRad(s.pitch);
    float             yaw          = DegreeToRad(s.yaw);
    float             rollspeed    = s.gyro_x;
    float             pitchspeed   = s.gyro_y;
    float             yawspeed     = s.gyro_z;
    auto              time_boot_ms = static_cast<uint32_t>(NowUs() / 1000ULL);
    mavlink_msg_attitude_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, time_boot_ms, roll, pitch, yaw, rollspeed,
                              pitchspeed, yawspeed);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendGlobalPosition(const SensorData& s) {
    mavlink_message_t msg;
    // GLOBAL_POSITION_INT uses lat/lon in 1e7, alt in mm
    auto     lat          = static_cast<int32_t>(s.latitude * 1e7);
    auto     lon          = static_cast<int32_t>(s.longitude * 1e7);
    auto     alt          = static_cast<int32_t>(s.altitude_gps * 1000.0f);
    auto     relative_alt = static_cast<int32_t>(s.altitude_baro * 1000.0f);
    auto     vx           = static_cast<int16_t>(s.ground_speed * 100.0f); // cm/s -> scaled
    int16_t  vy           = 0;
    int16_t  vz           = 0;
    uint16_t hdg          = 0;
    auto     time_boot_ms = static_cast<uint32_t>(NowUs() / 1000ULL);
    mavlink_msg_global_position_int_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, time_boot_ms, lat, lon, alt, relative_alt,
                                         vx, vy, vz, hdg);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendSysStatus(const BatteryData& b) {
    mavlink_message_t msg;
    // SYS_STATUS: onboard_control_sensors_present, enabled, health, load, voltage_battery (mV), current_battery (mA),
    // battery_remaining (%)
    uint32_t onboard_control_sensors_present = 0;
    uint32_t onboard_control_sensors_enabled = 0;
    uint32_t onboard_control_sensors_health  = 0;
    uint16_t load                            = 0;
    auto     voltage_battery                 = static_cast<uint16_t>(b.voltage * 1000.0f);
    auto     current_battery                 = static_cast<int16_t>(b.current * 1000.0f);
    int8_t   battery_remaining               = static_cast<int8_t>(std::max(-1.0f, std::min(100.0f, b.remaining_pct)));
    mavlink_msg_sys_status_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, onboard_control_sensors_present,
                                onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery,
                                current_battery, battery_remaining, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendBatteryStatus(const BatteryData& b) {
    mavlink_message_t msg;
    // BATTERY_STATUS: pack minimal fields
    uint8_t                 battery_id       = 0;
    int32_t                 current_consumed = 0;
    int32_t                 energy_consumed  = 0;
    int16_t                 temperature      = 0;
    std::array<int16_t, 10> voltages         = {0};
    voltages.at(0)                           = static_cast<int32_t>(b.voltage * 1000.0f);
    auto current_battery                     = static_cast<int16_t>(b.current * 1000.0f);
    auto battery_remaining                   = static_cast<int8_t>(std::max(-1.0f, std::min(100.0f, b.remaining_pct)));
    mavlink_msg_battery_status_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, battery_id, 0, energy_consumed, temperature,
                                    reinterpret_cast<const uint16_t*>(voltages.data()), current_battery, current_consumed,
                                    battery_remaining, 0, 0, 0, 0, 0, 0);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendRcChannels(const RemoteData& r) {
    mavlink_message_t msg;
    // RC_CHANNELS_RAW expects PWM values 0..2000; map normalized -1..1 to 1000..2000
    auto chan1        = static_cast<uint16_t>(1000 + ((r.throttle * 500.0f) + 500.0F));
    auto chan2        = static_cast<uint16_t>(1500 + r.roll * 500.0f);
    auto chan3        = static_cast<uint16_t>(1500 + r.pitch * 500.0f);
    auto chan4        = static_cast<uint16_t>(1500 + r.yaw * 500.0f);
    auto time_boot_ms = static_cast<uint32_t>(NowUs() / 1000ULL);
    mavlink_msg_rc_channels_raw_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, time_boot_ms, 0, chan1, chan2, chan3, chan4,
                                     0, 0, 0, 0, 0);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

MavlinkReceiver::MavlinkReceiver(Config& cfg) : m_cfg(std::move(cfg)) {}

MavlinkReceiver::~MavlinkReceiver() {
    spdlog::debug("MavlinkReceiver stopped!");
}
//------------------------------------------------------------------------------------
bool MavlinkReceiver::Initialize() {
    if (!m_cfg.m_uart) {
        spdlog::error("[MavlinkReceiver] No UART provided");
        return false;
    }
    return true;
}
//------------------------------------------------------------------------------------
void MavlinkReceiver::StepMR() {
    std::array<uint8_t, 256> buf{};
    mavlink_message_t        msg;
    mavlink_status_t         status;
    ssize_t                  n = m_cfg.m_uart->Read(buf.data(), sizeof(buf), 100);
    if (n <= 0) {
        spdlog::warn("Not enough data!");
    }
    for (ssize_t i = 0; i < n; ++i) {
        if (static_cast<bool>(mavlink_parse_char(MAVLINK_COMM_0, buf.at(i), &msg, &status))) {
            // handle message
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_COMMAND_LONG: {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                if (m_cmd_cb) {
                    m_cmd_cb(cmd);
                }
                // handle arm/disarm command specially
                if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    bool arm = (cmd.param1 > 0.5f);
                    if (m_arm_cb) {
                        m_arm_cb(arm);
                    }
                }
                break;
            }
            case MAVLINK_MSG_ID_PARAM_SET: {
                mavlink_param_set_t p;
                mavlink_msg_param_set_decode(&msg, &p);
                if (m_param_cb) {
                    m_param_cb(p);
                }
                break;
            }
            case MAVLINK_MSG_ID_HEARTBEAT: {
                // optional: handle GCS heartbeat if needed
                break;
            }
            default:
                // other messages can be handled by registered callbacks via COMMAND_LONG
                break;
            }
        }
    }
}
//------------------------------------------------------------------------------------
void MavlinkReceiver::SetCommandCallback(CommandCallback cb) {
    m_cmd_cb = std::move(cb);
}
void MavlinkReceiver::SetArmCallback(ArmCallback cb) {
    m_arm_cb = std::move(cb);
}
void MavlinkReceiver::SetParamSetCallback(ParamSetCallback cb) {
    m_param_cb = std::move(cb);
}
