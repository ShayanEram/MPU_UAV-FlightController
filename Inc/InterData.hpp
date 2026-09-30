#ifndef INTERDATA_HPP
#define INTERDATA_HPP

#include <string>

#include "MessageQueue.hpp"
#include "Observer.hpp"
#include "SharedResource.hpp"

struct BatteryData {
    int  chargeLevel; // Percentage of battery charge
    bool isCharging;  // Charging status
    bool isLow;       // Low battery warning
};

struct MotorData {
    int  speed;    // Speed of the motors
    bool isActive; // Motor activation status
};

struct PayloadData {
    std::string action;    // Action to perform with the payload
    int         payloadId; // Identifier for the payload
};

struct RemoteData {
    std::string command;   // Command from the remote controller
    int         commandId; // Identifier for the command
};

struct SensorData {
    float temperature; // Temperature reading
    float pressure;    // Pressure reading
    float altitude;    // Altitude reading
};

#endif // INTERDATA_HPP