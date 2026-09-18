#include "MavlinkReceiver.hpp"
#include <iostream>
#include <cstring>

using namespace uav;

MavlinkReceiver::MavlinkReceiver(const Config& cfg) : _cfg(cfg) {}

MavlinkReceiver::~MavlinkReceiver() { stop(); }

bool MavlinkReceiver::init() {
    if (!_cfg.uart) {
        std::cerr << "[MavlinkReceiver] No UART provided\n";
        return false;
    }
    return true;
}

void MavlinkReceiver::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&MavlinkReceiver::runLoop, this);
}

void MavlinkReceiver::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void MavlinkReceiver::setCommandCallback(CommandCallback cb) { _cmdCb = std::move(cb); }
void MavlinkReceiver::setArmCallback(ArmCallback cb) { _armCb = std::move(cb); }
void MavlinkReceiver::setParamSetCallback(ParamSetCallback cb) { _paramCb = std::move(cb); }

void MavlinkReceiver::runLoop() {
    uint8_t buf[256];
    mavlink_message_t msg;
    mavlink_status_t status;
    while (_running) {
        ssize_t n = _cfg.uart->read(buf, sizeof(buf), 100);
        if (n <= 0) continue;
        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                // handle message
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);
                        if (_cmdCb) _cmdCb(cmd);
                        // handle arm/disarm command specially
                        if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                            bool arm = (cmd.param1 > 0.5f);
                            if (_armCb) _armCb(arm);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_SET: {
                        mavlink_param_set_t p;
                        mavlink_msg_param_set_decode(&msg, &p);
                        if (_paramCb) _paramCb(p);
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
}
