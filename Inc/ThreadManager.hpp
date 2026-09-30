#ifndef THREAD_MANAGER_HPP
#define THREAD_MANAGER_HPP

#include "BatteryManager.hpp"
#include "FlightController.hpp"
#include "InterData.hpp"
#include "MotorController.hpp"
#include "PayloadManager.hpp"
#include "RemoteController.hpp"
#include "SensorManager.hpp"
#include "TelemetryManager.hpp"

class ThreadManager final {
  public:
    static ThreadManager& getInstance() {
        static ThreadManager instance;
        return instance;
    }
    void startAll();
    void stopAll();

  protected:
    BatteryManager   batteryManager;
    FlightController flightController;
    MotorController  motorController;
    PayloadManager   payloadManager;
    RemoteController remoteController;
    SensorManager    sensorManager;
    TelemetryManager telemetryManager;

    /*Shared communication objects: */
    MessageQueue<BatteryData>  batteryFlightQueue;
    SharedResource<MotorData>  motorFlightData;
    Observer<PayloadData>      payloadFlightObserver;
    SharedResource<RemoteData> remoteFlightData;
    MessageQueue<SensorData>   sensorFlightQueue;
    Observer<SensorData>       telemetrySensorObserver;

  private:
    ThreadManager();
    ~ThreadManager()                               = default;
    ThreadManager(const ThreadManager&)            = delete;
    ThreadManager& operator=(const ThreadManager&) = delete;
};

#endif
