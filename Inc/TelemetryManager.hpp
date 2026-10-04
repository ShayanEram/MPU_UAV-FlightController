#pragma once
/**
 * @file TelemetryManagerMavlink.hpp
 * @brief Header file for the TelemetryManagerMavlink class.
 * Provides real-time data on altitude, speed, battery status, and environmental conditions.
 */

#include <common/mavlink.h>

#include <deque>
#include <memory>

#include "Hardware_Interface/HardwareAbstractions.hpp"
#include "InterData.hpp"

class TelemetryManagerMavlink {
  public:
    struct Config {
        std::shared_ptr<HW::Uart> m_telemetry_uart;
        uint32_t                  m_send_hz{REFRESH_RATE};
        uint8_t                   m_system_id{1};
        uint8_t                   m_component_id{1};
        uint8_t                   m_target_system{TARGET_SYSTEM};
        uint8_t                   m_target_component{TARGET_COMPONENT};
    };
    explicit TelemetryManagerMavlink(Config& cfg);
    ~TelemetryManagerMavlink();

    explicit TelemetryManagerMavlink(const TelemetryManagerMavlink& rhs)   = delete;
    explicit TelemetryManagerMavlink(TelemetryManagerMavlink&& rhs)        = delete;
    TelemetryManagerMavlink& operator=(const TelemetryManagerMavlink& rhs) = delete;
    TelemetryManagerMavlink& operator=(TelemetryManagerMavlink&& rhs)      = delete;

    bool Initialize();

    // push telemetry packet to be sent (thread-safe)
    void Send(const TelemetryPacket& pkt);

  private:
    void StepTM();

    void SendHeartbeat();
    void SendAttitude(const SensorData& s);
    void SendGlobalPosition(const SensorData& s);
    void SendSysStatus(const BatteryData& b);
    void SendBatteryStatus(const BatteryData& b);
    void SendRcChannels(const RemoteData& r);

    Config m_cfg;

    // internal queue
    struct QueueItem {
        TelemetryPacket m_pkt;
    };
    std::mutex            m_qmut;
    std::deque<QueueItem> m_queue;

    uint8_t m_seq_counter{0};

    static constexpr auto REFRESH_RATE     = 5;
    static constexpr auto TARGET_SYSTEM    = 255; // 255 = broadcast
    static constexpr auto TARGET_COMPONENT = 190;
};

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

class MavlinkReceiver {
  public:
    using CommandCallback  = std::function<void(const mavlink_command_long_t&)>;
    using ArmCallback      = std::function<void(bool arm)>; // true=arm, false=disarm
    using ParamSetCallback = std::function<void(const mavlink_param_set_t&)>;

    struct Config {
        std::shared_ptr<HW::Uart> m_uart;
        uint8_t                   m_system_id{1};
        uint8_t                   m_component_id{1};
    };

    explicit MavlinkReceiver(Config& cfg);
    ~MavlinkReceiver();

    explicit MavlinkReceiver(const MavlinkReceiver& rhs)   = delete;
    explicit MavlinkReceiver(MavlinkReceiver&& rhs)        = delete;
    MavlinkReceiver& operator=(const MavlinkReceiver& rhs) = delete;
    MavlinkReceiver& operator=(MavlinkReceiver&& rhs)      = delete;

    bool Initialize();

    void SetCommandCallback(CommandCallback cb);
    void SetArmCallback(ArmCallback cb);
    void SetParamSetCallback(ParamSetCallback cb);

  private:
    void   StepMR();
    Config m_cfg;

    CommandCallback  m_cmd_cb;
    ArmCallback      m_arm_cb;
    ParamSetCallback m_param_cb;
};
