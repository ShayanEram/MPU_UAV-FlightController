#include "FlightController.hpp"

FlightController::FlightController(MessageQueue<BatteryData>& batteryQueue, SharedResource<MotorData>& motorData,
                                   Observer<PayloadData>& payloadObserver, SharedResource<RemoteData>& remoteData,
                                   MessageQueue<SensorData>& sensorQueue) :
    _batteryQueue(batteryQueue), _motorData(motorData), _payloadObserver(payloadObserver), _remoteData(remoteData),
    _sensorQueue(sensorQueue), running(false) {
    _payloadObserver.addListener([this](const PayloadData& payloadData) { this->onPayloadDataUpdated(payloadData); });
}
FlightController::~FlightController() {
    stop();
}
//------------------------------------------------------------------------------------
void FlightController::start() {
    if (!running.load()) {
        running.store(true);
        moduleThread = std::thread(&FlightController::runLoop, this);
    }
}
void FlightController::stop() {
    if (running.load()) {
        running.store(false);
        if (moduleThread.joinable()) {
            moduleThread.join();
        }
    }
}
//------------------------------------------------------------------------------------
void FlightController::runLoop() {

    std::future<BatteryData> batteryData = {};
    std::future<MotorData>   motorData   = {};
    std::future<RemoteData>  remoteData  = {};
    std::future<SensorData>  sensorData  = {};

    while (running.load()) {
        batteryData = _batteryQueue.popCommandAsync();
        motorData   = _motorData.getDataAsync();
        remoteData  = _remoteData.getDataAsync();
        sensorData  = _sensorQueue.popCommandAsync();

        std::this_thread::sleep_for(std::chrono::milliseconds(FLIGHT_CONTROLLER_UPDATE_RATE_MS));

        std::cout << "FlightController running" << std::endl;
    }
}
//------------------------------------------------------------------------------------
void FlightController::onPayloadDataUpdated(const PayloadData& payloadData) {
    std::cout << "Payload action: " << payloadData.action << std::endl;
    std::cout << "Payload ID: " << payloadData.payloadId << std::endl;

    _payloadCallbackData = payloadData;
}