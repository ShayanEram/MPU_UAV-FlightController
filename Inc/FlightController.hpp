/**
 * @file FlightController.hpp
 * @brief Header file for the FlightController class.
 * The brain of the aircraft, responsible for stabilization, navigation, and autonomous flight.
 */

#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include <atomic>
#include <iostream>
#include <thread>

#include "InterData.hpp"

class FlightController {
  public:
    explicit FlightController(MessageQueue<BatteryData>& batteryQueue, SharedResource<MotorData>& motorData,
                              Observer<PayloadData>& payloadObserver, SharedResource<RemoteData>& remoteData,
                              MessageQueue<SensorData>& sensorQueue);
    ~FlightController();

    void start();
    void stop();

  protected:
    void        onPayloadDataUpdated(const PayloadData& payloadData);
    PayloadData _payloadCallbackData = {};

    static constexpr int FLIGHT_CONTROLLER_UPDATE_RATE_MS = 100; // Update rate in milliseconds

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    MessageQueue<BatteryData>&  _batteryQueue;
    SharedResource<MotorData>&  _motorData;
    Observer<PayloadData>&      _payloadObserver;
    SharedResource<RemoteData>& _remoteData;
    MessageQueue<SensorData>&   _sensorQueue;
};

#endif // FLIGHTCONTROLLER_HPP