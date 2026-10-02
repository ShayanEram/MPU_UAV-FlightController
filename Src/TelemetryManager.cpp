#include <spdlog/spdlog.h>
// #include "TelemetryManager.hpp"

// TelemetryManager::TelemetryManager(Observer<SensorData>& telemetryObserver) :
//     _telemetryObserver(telemetryObserver), running(false) {
//     _telemetryPacket = "Telemetry Data: Altitude, Speed, Battery, Environment";
// }
// TelemetryManager::~TelemetryManager() {
//     stop();
// }
// //------------------------------------------------------------------------------------
// void TelemetryManager::start() {
//     if (!running.load()) {
//         running.store(true);
//         moduleThread = std::thread(&TelemetryManager::runLoop, this);
//     }
// }
// void TelemetryManager::stop() {
//     if (running.load()) {
//         running.store(false);
//         if (moduleThread.joinable()) {
//             moduleThread.join();
//         }
//     }
// }
// //------------------------------------------------------------------------------------
// void TelemetryManager::runLoop() {
//     while (running.load()) {

//         updateTelemetryData();
//         sendTelemetryData();

//         std::this_thread::sleep_for(std::chrono::milliseconds(TELEMETRY_UPDATE_INTERVAL_MS));

//         std::cout << "TelemetryManager running... Data sent." << std::endl;
//     }
// }
// //------------------------------------------------------------------------------------
// void TelemetryManager::updateTelemetryData() {
//     _telemetryPacket = "Updated Telemetry Data: Altitude, Speed, Battery, Environment";
// }
// void TelemetryManager::sendTelemetryData() {
//     std::cout << _telemetryPacket << std::endl;
// }