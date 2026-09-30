#include "ThreadManager.hpp"

ThreadManager::ThreadManager() :
    flightController(batteryFlightQueue, motorFlightData, payloadFlightObserver, remoteFlightData, sensorFlightQueue),
    batteryManager(batteryFlightQueue), motorController(motorFlightData), remoteController(remoteFlightData),
    payloadManager(payloadFlightObserver), sensorManager(sensorFlightQueue, telemetrySensorObserver),
    telemetryManager(telemetrySensorObserver) {}

void ThreadManager::startAll() {
    batteryManager.start();
    flightController.start();
    motorController.start();
    payloadManager.start();
    remoteController.start();
    sensorManager.start();
    telemetryManager.start();
}

void ThreadManager::stopAll() {
    batteryManager.stop();
    flightController.stop();
    motorController.stop();
    payloadManager.stop();
    remoteController.stop();
    sensorManager.stop();
    telemetryManager.stop();
}