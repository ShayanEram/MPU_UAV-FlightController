#include "SensorManager.hpp"

SensorManager::SensorManager(MessageQueue<SensorData>& sensorQueue, Observer<SensorData>& sensorObserver) :
    _sensorQueue(sensorQueue), _sensorObserver(sensorObserver), running(false) {
    // Constructor implementation
}
SensorManager::~SensorManager() {
    stop();
}
//------------------------------------------------------------------------------------
void SensorManager::start() {
    if (!running.load()) {
        running.store(true);
        moduleThread = std::thread(&SensorManager::runLoop, this);
    }
}
void SensorManager::stop() {
    if (running.load()) {
        running.store(false);
        if (moduleThread.joinable()) {
            moduleThread.join();
        }
    }
}
//------------------------------------------------------------------------------------
void SensorManager::runLoop() {

    while (running.load()) {
        SensorReadings data = {};

        readIMUData(IMU_DEVICE_FILE); // Reads and prints IMU data
        data.accelX = static_cast<float>(readRegister(IMU_DEVICE_FILE, ACCEL_XOUT_H));
        data.gyroX  = static_cast<float>(readRegister(IMU_DEVICE_FILE, GYRO_XOUT_H));

        // Read GPS
        readGPSData(); // Reads and prints GPS data

        // Read Barometer (Altitude)
        readBarometerData(BAROMETER_DEVICE_FILE); // Reads and prints barometer pressure
        data.altitude = static_cast<float>(readRegister(BAROMETER_DEVICE_FILE, PRESSURE_REG));

        // Read Magnetometer (Compass Heading)
        readMagnetometerData(MAGNETOMETER_DEVICE_FILE); // Reads and prints compass heading
        data.heading = static_cast<float>(readRegister(MAGNETOMETER_DEVICE_FILE, MAG_XOUT_H));

        std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_UPDATE_INTERVAL_MS));

        std::cout << "SensorManager running... Sensor data processed." << std::endl;
    }
}
//------------------------------------------------------------------------------------
int SensorManager::readRegister(std::string file, uint8_t reg) {
    std::fstream fs;
    fs.open(file, std::ios::in | std::ios::out);
    if (!fs.is_open()) {
        throw std::runtime_error("Failed to open I2C device");
    }

    fs.write(reinterpret_cast<char*>(&reg), sizeof(reg));
    uint8_t value;
    fs.read(reinterpret_cast<char*>(&value), sizeof(value));
    fs.close();
    return value;
}
void SensorManager::readIMUData(std::string file) {
    int accelX = (readRegister(file, ACCEL_XOUT_H) << 8) | readRegister(file, ACCEL_XOUT_H + 1);
    int gyroX  = (readRegister(file, GYRO_XOUT_H) << 8) | readRegister(file, GYRO_XOUT_H + 1);

    std::cout << "IMU Data - AccelX: " << accelX << " GyroX: " << gyroX << std::endl;
}
void SensorManager::readGPSData() {
    std::ifstream gpsFile(GPS_DEVICE);
    std::string   line;

    while (getline(gpsFile, line)) {
        if (line.find("$GPGGA") != std::string::npos) { // Look for GPGGA sentence
            std::cout << "GPS Data: " << line << std::endl;
            break;
        }
    }
}
void SensorManager::readBarometerData(std::string file) {
    std::fstream barometerFile;
    barometerFile.open(file, std::ios::in | std::ios::out);
    if (!barometerFile.is_open()) {
        throw std::runtime_error("Failed to open Barometer I2C device");
    }

    uint8_t buffer[3] = {PRESSURE_REG};
    barometerFile.write(reinterpret_cast<char*>(buffer), 1);
    barometerFile.read(reinterpret_cast<char*>(buffer), 3);
    barometerFile.close();

    int pressure = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);
    std::cout << "Barometer Pressure: " << pressure << " Pa" << std::endl;
}
void SensorManager::readMagnetometerData(std::string file) {
    int magX = (readRegister(file, MAG_XOUT_H) << 8) | readRegister(file, MAG_XOUT_H + 1);
    std::cout << "Magnetometer Heading: " << magX << std::endl;
}