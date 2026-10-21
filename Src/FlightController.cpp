#include "FlightController.hpp"

#include <spdlog/spdlog.h>

FlightController::FlightController(Config& cfg) : m_cfg(std::move(cfg)), m_last_loop_time(std::chrono::steady_clock::now()) {}

FlightController::~FlightController() {
    spdlog::debug("FlightController stopped!");
}
//------------------------------------------------------------------------------------
bool FlightController::Initialize() {
    // initialize internal state
    m_last_motor     = MotorData{};
    m_integral_roll  = 0.0F;
    m_integral_pitch = 0.0F;
    m_integral_yaw   = 0.0F;
    m_last_loop_time = std::chrono::steady_clock::now();

    RETURN_CODE_TYPE s1 = create_queuing_port(&m_sensor_rx_port, "SensorDataQueue", sizeof(SensorData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s2 = create_queuing_port(&m_battery_rx_port, "BatteryDataQueue", sizeof(BatteryData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s3 = create_queuing_port(&m_remote_rx_port, "RemoteDataQueue", sizeof(RemoteData), 10, DIR_DESTINATION);
    RETURN_CODE_TYPE s4 = create_queuing_port(&m_motor_tx_port, "MotorDataQueue", sizeof(MotorData), 10, DIR_SOURCE);
    RETURN_CODE_TYPE s5 = create_queuing_port(&m_cmd_rx_port, "CommandDataQueue", sizeof(CommandMessage), 10, DIR_DESTINATION);

    return (s1 == RC_OK && s2 == RC_OK && s3 == RC_OK && s4 == RC_OK && s5 == RC_OK);
}
//------------------------------------------------------------------------------------
void FlightController::StepFC() {
    SYSTEM_TIME_NS timeout_ns = 5000000ULL; // 5ms timeout
    size_t         len        = 0;

    SensorData incoming_sensor;
    if (receive_queuing_message(&m_sensor_rx_port, &incoming_sensor, &len, timeout_ns) == RC_OK) {
        m_last_sensor = incoming_sensor;
    }

    BatteryData incoming_battery;
    if (receive_queuing_message(&m_battery_rx_port, &incoming_battery, &len, timeout_ns) == RC_OK) {
        m_last_battery = incoming_battery;
    }

    RemoteData incoming_remote;
    if (receive_queuing_message(&m_remote_rx_port, &incoming_remote, &len, timeout_ns) == RC_OK) {
        m_last_remote = incoming_remote;
    }

    CommandMessage gcs_cmd;
    if (receive_queuing_message(&m_cmd_rx_port, &gcs_cmd, &len, timeout_ns) == RC_OK && gcs_cmd.is_valid) {
        if (gcs_cmd.command_id == MAV_CMD_COMPONENT_ARM_DISARM) {
            bool should_arm = (gcs_cmd.param1 > 0.5F);
        }
    }

    // Compute delta time (dt) for PID integration/differentiation
    auto  now        = std::chrono::steady_clock::now();
    float dt         = std::chrono::duration<float>(now - m_last_loop_time).count();
    m_last_loop_time = now;
    if (dt <= 0.0F || dt > 0.1F) {
        dt = 1.0F / static_cast<float>(std::max<uint32_t>(1, m_cfg.m_loop_hz));
    }

    // Setpoints from RC (normalized -1..1 mapped to target angles)
    float target_roll  = m_last_remote.roll * m_cfg.m_max_roll_angle;
    float target_pitch = m_last_remote.pitch * m_cfg.m_max_pitch_angle;

    // --- ROLL PID CONTROLLER ---
    float error_roll = target_roll - m_last_sensor.roll;
    m_integral_roll += error_roll * dt;
    m_integral_roll = std::clamp(m_integral_roll, -1.0F, 1.0F); // Anti-windup clamp
    // Derivative term uses gyro_x directly (negated for damping)
    float output_roll =
        (m_cfg.m_kp_roll * error_roll) + (m_cfg.m_ki_roll * m_integral_roll) - (m_cfg.m_kd_roll * m_last_sensor.gyro_x);

    // --- PITCH PID CONTROLLER ---
    float error_pitch = target_pitch - m_last_sensor.pitch;
    m_integral_pitch += error_pitch * dt;
    m_integral_pitch = std::clamp(m_integral_pitch, -1.0F, 1.0F);
    float output_pitch =
        (m_cfg.m_kp_pitch * error_pitch) + (m_cfg.m_ki_pitch * m_integral_pitch) - (m_cfg.m_kd_pitch * m_last_sensor.gyro_y);

    // --- YAW / RUDDER (Direct pass-through or basic damping) ---
    float output_rudder = m_last_remote.yaw;

    // Assemble Motor & Control Surface Outputs (-1..1 for surfaces, 0..1 for throttle)
    MotorData out;
    out.motor_throttle = m_last_remote.throttle;
    out.servo_aileron  = std::clamp(output_roll, -1.0F, 1.0F);
    out.servo_elevator = std::clamp(output_pitch, -1.0F, 1.0F);
    out.servo_rudder   = std::clamp(output_rudder, -1.0F, 1.0F);

    // Safety Failsafe: Cut throttle if RC is disconnected or battery is critical
    if (!m_last_remote.is_connected || m_last_battery.is_critical) {
        out.motor_throttle = 0.0F;
    }

    out.timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    m_last_motor     = out;

    // Publish actuator command via APEX queuing port
    send_queuing_message(&m_motor_tx_port, &out, sizeof(MotorData), timeout_ns);
}
//------------------------------------------------------------------------------------
MotorData FlightController::GetMotorData() const {
    return m_last_motor;
}