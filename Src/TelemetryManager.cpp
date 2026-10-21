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
    RETURN_CODE_TYPE s1 = create_queuing_port(&m_sensor_rx_port, "SensorDataQueue", sizeof(SensorData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s2 = create_queuing_port(&m_batt_rx_port, "BatteryDataQueue", sizeof(BatteryData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s3 = create_queuing_port(&m_remote_rx_port, "RemoteDataQueue", sizeof(RemoteData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s4 = create_queuing_port(&m_motor_rx_port, "MotorDataQueue", sizeof(MotorData), 10, DIR_DESTINATION);
    return (s1 == RC_OK && s2 == RC_OK && s3 == RC_OK && s4 == RC_OK);
}
//------------------------------------------------------------------------------------
void TelemetryManagerMavlink::StepTM() {
    const auto     period     = std::chrono::milliseconds(1000 / std::max<uint32_t>(1, m_cfg.m_send_hz));
    SYSTEM_TIME_NS timeout_ns = 2000000ULL;
    size_t         len        = 0;

    SendHeartbeat();

    SensorData s;
    if (receive_queuing_message(&m_sensor_rx_port, &s, &len, timeout_ns) == RC_OK) {
        SendAttitude(s);
        SendGlobalPosition(s);
    }

    BatteryData b;
    if (receive_queuing_message(&m_batt_rx_port, &b, &len, timeout_ns) == RC_OK) {
        SendSysStatus(b);
        SendBatteryStatus(b);
    }

    RemoteData r;
    if (receive_queuing_message(&m_remote_rx_port, &r, &len, timeout_ns) == RC_OK) {
        SendRcChannels(r);
    }
}
//------------------------------------------------------------------------------------
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
    auto     alt          = static_cast<int32_t>(s.altitude_gps * 1000.0F);
    auto     relative_alt = static_cast<int32_t>(s.altitude_baro * 1000.0F);
    auto     vx           = static_cast<int16_t>(s.ground_speed * 100.0F); // cm/s -> scaled
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
    auto     voltage_battery                 = static_cast<uint16_t>(b.voltage * 1000.0F);
    auto     current_battery                 = static_cast<int16_t>(b.current * 1000.0F);
    int8_t   battery_remaining               = static_cast<int8_t>(std::max(-1.0F, std::min(100.0F, b.remaining_pct)));
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
    voltages.at(0)                           = static_cast<int32_t>(b.voltage * 1000.0F);
    auto current_battery                     = static_cast<int16_t>(b.current * 1000.0F);
    auto battery_remaining                   = static_cast<int8_t>(std::max(-1.0F, std::min(100.0F, b.remaining_pct)));
    mavlink_msg_battery_status_pack(m_cfg.m_system_id, m_cfg.m_component_id, &msg, battery_id, 0, energy_consumed, temperature,
                                    reinterpret_cast<const uint16_t*>(voltages.data()), current_battery, current_consumed,
                                    battery_remaining, 0, 0, 0, 0, 0, 0);
    WriteMavlinkMsg(m_cfg.m_telemetry_uart.get(), msg);
}

void TelemetryManagerMavlink::SendRcChannels(const RemoteData& r) {
    mavlink_message_t msg;
    // RC_CHANNELS_RAW expects PWM values 0..2000; map normalized -1..1 to 1000..2000
    auto chan1        = static_cast<uint16_t>(1000 + ((r.throttle * 500.0F) + 500.0F));
    auto chan2        = static_cast<uint16_t>(1500 + (r.roll * 500.0F));
    auto chan3        = static_cast<uint16_t>(1500 + (r.pitch * 500.0F));
    auto chan4        = static_cast<uint16_t>(1500 + (r.yaw * 500.0F));
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
    RETURN_CODE_TYPE status = create_queuing_port(&m_cmd_tx_port, "CommandDataQueue", sizeof(CommandMessage), 10, DIR_SOURCE);
    return (status == RC_OK);
}
//------------------------------------------------------------------------------------
void MavlinkReceiver::StepMR() {
    std::array<uint8_t, 256> buf{};
    mavlink_message_t        msg;
    mavlink_status_t         status;

    ssize_t n = m_cfg.m_uart->Read(buf.data(), sizeof(buf), 10);
    if (n <= 0) {
        return; // No data received this cycle
    }

    for (ssize_t i = 0; i < n; ++i) {
        if (static_cast<bool>(mavlink_parse_char(MAVLINK_COMM_0, buf.at(i), &msg, &status))) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_COMMAND_LONG: {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);

                // Package into aeroRTOS command structure
                CommandMessage cMsg;
                cMsg.command_id = cmd.command;
                cMsg.param1     = cmd.param1;
                cMsg.param2     = cmd.param2;
                cMsg.is_valid   = true;
                cMsg.timestamp_ms =
                    duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                // Send deterministically via APEX queuing port
                SYSTEM_TIME_NS timeout_ns = 1000000ULL;
                send_queuing_message(&m_cmd_tx_port, &cMsg, sizeof(CommandMessage), timeout_ns);
                break;
            }
            default:
                break;
            }
        }
    }
}