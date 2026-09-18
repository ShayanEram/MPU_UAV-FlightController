#include "SensorManager.hpp"
#include "LinuxI2CBus.hpp"
#include "LinuxUartPort.hpp"
#include <chrono>
#include <iostream>
#include <cstring>
#include <cmath>

using namespace uav;
using namespace std::chrono;

// UBX helpers
static uint8_t ubx_ck_a(const uint8_t* buf, size_t len) {
    uint8_t a = 0;
    for (size_t i = 0; i < len; ++i) a += buf[i];
    return a;
}
static uint8_t ubx_ck_b(const uint8_t* buf, size_t len) {
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < len; ++i) { a += buf[i]; b += a; }
    return b;
}

SensorManager::SensorManager(const Config& cfg)
    : _cfg(cfg),
      _ahrs(cfg.madgwickBeta),
      _lastAhrsTime(steady_clock::now()) {
    _gpsBuf.reserve(1024);
}

SensorManager::~SensorManager() {
    stop();
}

bool SensorManager::init() {
    // Ensure adapters exist; create defaults if not provided
    if (!_cfg.i2c) {
        _cfg.i2c = std::make_shared<LinuxI2CBus>("/dev/i2c-1");
    }
    if (!_cfg.gpsUart) {
        _cfg.gpsUart = std::make_shared<LinuxUartPort>("/dev/serial0");
        _cfg.gpsUart->open("/dev/serial0", 38400);
    }
    if (!_cfg.lidarUart) {
        _cfg.lidarUart = std::make_shared<LinuxUartPort>("/dev/ttyS1");
        _cfg.lidarUart->open("/dev/ttyS1", 115200);
    }
    _lastAhrsTime = steady_clock::now();
    return true;
}

void SensorManager::start() {
    if (_running.exchange(true)) return;
    _thread = std::thread(&SensorManager::runLoop, this);
}

void SensorManager::stop() {
    if (!_running.exchange(false)) return;
    if (_thread.joinable()) _thread.join();
}

void SensorManager::setSensorCallback(SensorCallback cb) {
    std::lock_guard<std::mutex> lk(_cbMutex);
    _callback = std::move(cb);
}

bool SensorManager::readOnce(SensorData& out) {
    readImu(out);
    readBaro(out);
    readCompass(out);
    readGps(out);
    readAirspeed(out);
    readLidar(out);

    // AHRS update
    auto now = steady_clock::now();
    float dt = std::chrono::duration<float>(now - _lastAhrsTime).count();
    if (dt <= 0.0f) dt = 0.001f;
    _lastAhrsTime = now;

    // gyro expected in rad/s; ensure readImu sets that
    _ahrs.update(out.gyroX, out.gyroY, out.gyroZ,
                 out.accelX, out.accelY, out.accelZ,
                 _magX, _magY, _magZ, dt);

    float roll, pitch, yaw;
    _ahrs.getEuler(roll, pitch, yaw);
    out.roll = roll;
    out.pitch = pitch;
    out.yaw = yaw;

    out.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    return true;
}

void SensorManager::runLoop() {
    const auto period = milliseconds(1000 / std::max<uint32_t>(1, _cfg.loopHz));
    while (_running) {
        SensorData s;
        readImu(s);
        readBaro(s);
        readCompass(s);
        readGps(s);
        readAirspeed(s);
        readLidar(s);

        // AHRS update
        auto now = steady_clock::now();
        float dt = std::chrono::duration<float>(now - _lastAhrsTime).count();
        if (dt <= 0.0f) dt = 0.001f;
        _lastAhrsTime = now;

        _ahrs.update(s.gyroX, s.gyroY, s.gyroZ,
                     s.accelX, s.accelY, s.accelZ,
                     _magX, _magY, _magZ, dt);

        float roll, pitch, yaw;
        _ahrs.getEuler(roll, pitch, yaw);
        s.roll = roll;
        s.pitch = pitch;
        s.yaw = yaw;

        s.timestampMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        // callback
        {
            std::lock_guard<std::mutex> lk(_cbMutex);
            if (_callback) _callback(s);
        }

        std::this_thread::sleep_for(period);
    }
}

/* -------------------------
   Low-level sensor readers
   ------------------------- */

bool SensorManager::readI2CRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!_cfg.i2c) return false;
    return _cfg.i2c->read(addr, reg, buf, len);
}

bool SensorManager::writeI2CRegister(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    if (!_cfg.i2c) return false;
    return _cfg.i2c->write(addr, reg, buf, len);
}

/* IMU: ICM20602 basic read */
void SensorManager::readImu(SensorData& s) {
    // Default simulated values if no I2C
    if (!_cfg.i2c) {
        s.accelX = 0.0f; s.accelY = 0.0f; s.accelZ = -9.80665f;
        s.gyroX = s.gyroY = s.gyroZ = 0.0f;
        s.health.imuHealthy = true;
        return;
    }

    // WHO_AM_I check
    uint8_t who = 0;
    if (!readI2CRegister(_cfg.icmAddr, 0x75, &who, 1)) {
        s.health.imuHealthy = false;
        return;
    }

    // Read accel/gyro registers (ACCEL_XOUT_H = 0x3B, 14 bytes)
    uint8_t buf[14];
    if (!readI2CRegister(_cfg.icmAddr, 0x3B, buf, 14)) {
        s.health.imuHealthy = false;
        return;
    }

    auto toS16 = [](uint8_t hi, uint8_t lo)->int16_t { return static_cast<int16_t>((hi << 8) | lo); };
    int16_t ax = toS16(buf[0], buf[1]);
    int16_t ay = toS16(buf[2], buf[3]);
    int16_t az = toS16(buf[4], buf[5]);
    // temp ignored here
    int16_t gx = toS16(buf[8], buf[9]);
    int16_t gy = toS16(buf[10], buf[11]);
    int16_t gz = toS16(buf[12], buf[13]);

    // Convert: assume accel FS = ±2g, gyro FS = ±250 dps
    const float accelScale = 2.0f / 32768.0f * 9.80665f; // m/s^2
    const float gyroScale = 250.0f / 32768.0f * (M_PI / 180.0f); // rad/s

    s.accelX = ax * accelScale;
    s.accelY = ay * accelScale;
    s.accelZ = az * accelScale;

    s.gyroX = gx * gyroScale;
    s.gyroY = gy * gyroScale;
    s.gyroZ = gz * gyroScale;

    s.health.imuHealthy = true;
}

/* Barometer: MS5611 */
void SensorManager::readBaro(SensorData& s) {
    if (!_cfg.i2c) {
        s.pressure = 1013.25f;
        s.altitudeBaro = 100.0f;
        s.temperature = 20.0f;
        s.health.baroHealthy = true;
        return;
    }

    // Minimal MS5611 sequence (reset + PROM + conversion)
    // Reset
    uint8_t cmd = 0x1E;
    writeI2CRegister(_cfg.ms5611Addr, cmd, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    // Read PROM coefficients
    uint16_t C[7] = {0};
    for (int i = 0; i < 6; ++i) {
        uint8_t buf[2];
        if (!readI2CRegister(_cfg.ms5611Addr, 0xA2 + i*2, buf, 2)) {
            s.health.baroHealthy = false;
            return;
        }
        C[i+1] = (buf[0] << 8) | buf[1];
    }

    // D1 (pressure)
    uint8_t cmdD1 = 0x48; // OSR=4096
    writeI2CRegister(_cfg.ms5611Addr, cmdD1, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t d1buf[3];
    if (!readI2CRegister(_cfg.ms5611Addr, 0x00, d1buf, 3)) {
        s.health.baroHealthy = false;
        return;
    }
    uint32_t D1 = (d1buf[0] << 16) | (d1buf[1] << 8) | d1buf[2];

    // D2 (temp)
    uint8_t cmdD2 = 0x58;
    writeI2CRegister(_cfg.ms5611Addr, cmdD2, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t d2buf[3];
    if (!readI2CRegister(_cfg.ms5611Addr, 0x00, d2buf, 3)) {
        s.health.baroHealthy = false;
        return;
    }
    uint32_t D2 = (d2buf[0] << 16) | (d2buf[1] << 8) | d2buf[2];

    // Datasheet calculations
    int64_t dT = static_cast<int64_t>(D2) - (static_cast<int64_t>(C[5]) << 8);
    int64_t TEMP = 2000 + ((dT * C[6]) >> 23);
    int64_t OFF = (static_cast<int64_t>(C[2]) << 16) + ((static_cast<int64_t>(C[4]) * dT) >> 7);
    int64_t SENS = (static_cast<int64_t>(C[1]) << 15) + ((static_cast<int64_t>(C[3]) * dT) >> 8);
    int64_t P = (((static_cast<int64_t>(D1) * SENS) >> 21) - OFF) >> 15;

    s.temperature = TEMP / 100.0f;
    s.pressure = P / 100.0f; // mbar

    // approximate altitude (ISA)
    const float seaLevelPressure = 1013.25f;
    s.altitudeBaro = 44330.0f * (1.0f - powf(s.pressure / seaLevelPressure, 0.1903f));

    s.health.baroHealthy = true;
}

/* Compass: HMC5883L */
void SensorManager::readCompass(SensorData& s) {
    if (!_cfg.i2c) {
        _magX = _magY = _magZ = 0.0f;
        s.heading = 0.0f;
        return;
    }

    // Configure continuous measurement (CRA/CRB/MODE)
    uint8_t cfg = 0x00; // default
    writeI2CRegister(_cfg.hmcAddr, 0x00, &cfg, 1);

    uint8_t buf[6];
    if (!readI2CRegister(_cfg.hmcAddr, 0x03, buf, 6)) {
        s.health.gpsFix = s.health.gpsFix; // no-op
        return;
    }

    int16_t mx = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t my = static_cast<int16_t>((buf[4] << 8) | buf[5]); // note axis order
    int16_t mz = static_cast<int16_t>((buf[2] << 8) | buf[3]);

    // store raw magnetometer for AHRS
    _magX = static_cast<float>(mx);
    _magY = static_cast<float>(my);
    _magZ = static_cast<float>(mz);

    // compute heading (simple)
    float heading = atan2f(_magY, _magX) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    s.heading = heading;
}

/* GPS: Ublox NAV-PVT parsing (basic) */
void SensorManager::readGps(SensorData& s) {
    if (!_cfg.gpsUart) {
        // simulated
        s.latitude = 45.5;
        s.longitude = -73.6;
        s.altitudeGPS = 100.0f;
        s.groundSpeed = 0.0f;
        s.gpsSatellites = 8;
        s.health.gpsFix = true;
        return;
    }

    // Read available bytes into buffer
    uint8_t buf[512];
    ssize_t n = _cfg.gpsUart->read(buf, sizeof(buf), 20);
    if (n > 0) {
        std::lock_guard<std::mutex> lk(_gpsBufMutex);
        _gpsBuf.insert(_gpsBuf.end(), buf, buf + n);
    }

    // Parse UBX messages from buffer
    std::lock_guard<std::mutex> lk(_gpsBufMutex);
    size_t i = 0;
    while (i + 8 <= _gpsBuf.size()) {
        if (_gpsBuf[i] != 0xB5 || _gpsBuf[i+1] != 0x62) { ++i; continue; }
        if (i + 6 > _gpsBuf.size()) break;
        uint8_t cls = _gpsBuf[i+2];
        uint8_t id = _gpsBuf[i+3];
        uint16_t len = static_cast<uint16_t>(_gpsBuf[i+4]) | (static_cast<uint16_t>(_gpsBuf[i+5]) << 8);
        if (i + 6 + len + 2 > _gpsBuf.size()) break; // wait for full packet
        const uint8_t* payload = _gpsBuf.data() + i + 6;
        uint8_t ck_a = _gpsBuf[i + 6 + len];
        uint8_t ck_b = _gpsBuf[i + 6 + len + 1];

        // compute checksum over class,id,len,payload
        std::vector<uint8_t> ckbuf;
        ckbuf.reserve(4 + len);
        ckbuf.push_back(cls); ckbuf.push_back(id);
        ckbuf.push_back(static_cast<uint8_t>(len & 0xFF));
        ckbuf.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        for (size_t k = 0; k < len; ++k) ckbuf.push_back(payload[k]);
        uint8_t ca = ubx_ck_a(ckbuf.data(), ckbuf.size());
        uint8_t cb = ubx_ck_b(ckbuf.data(), ckbuf.size());
        if (ca == ck_a && cb == ck_b) {
            // NAV-PVT (class 0x01 id 0x07)
            if (cls == 0x01 && id == 0x07 && len >= 92) {
                int32_t lat = static_cast<int32_t>(payload[24] | (payload[25]<<8) | (payload[26]<<16) | (payload[27]<<24));
                int32_t lon = static_cast<int32_t>(payload[28] | (payload[29]<<8) | (payload[30]<<16) | (payload[31]<<24));
                int32_t height = static_cast<int32_t>(payload[32] | (payload[33]<<8) | (payload[34]<<16) | (payload[35]<<24));
                int32_t gSpeed = static_cast<int32_t>(payload[60] | (payload[61]<<8) | (payload[62]<<16) | (payload[63]<<24));
                uint8_t numSV = payload[23];
                s.latitude = lat * 1e-7;
                s.longitude = lon * 1e-7;
                s.altitudeGPS = height / 1000.0f;
                s.groundSpeed = gSpeed / 1000.0f;
                s.gpsSatellites = numSV;
                s.health.gpsFix = true;
            }
        }
        // advance past this packet
        i += 6 + len + 2;
    }
    // erase consumed bytes
    if (i > 0) _gpsBuf.erase(_gpsBuf.begin(), _gpsBuf.begin() + i);
}

/* Airspeed: placeholder for digital pitot (I2C) */
void SensorManager::readAirspeed(SensorData& s) {
    if (!_cfg.i2c) {
        s.airspeed = 0.0f;
        s.health.airspeedHealthy = false;
        return;
    }
    uint8_t addr = _cfg.airspeedAddr;
    uint8_t buf[2];
    if (readI2CRegister(addr, 0x00, buf, 2)) {
        int16_t raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        // sensor-specific mapping required; here assume raw Pa*100
        float diffPa = static_cast<float>(raw) / 100.0f;
        const float rho = 1.225f;
        if (diffPa > 0.1f) s.airspeed = sqrtf(2.0f * diffPa / rho);
        else s.airspeed = 0.0f;
        s.health.airspeedHealthy = true;
    } else {
        s.airspeed = 0.0f;
        s.health.airspeedHealthy = false;
    }
}

/* LiDAR: TFmini UART parsing */
void SensorManager::readLidar(SensorData& s) {
    if (!_cfg.lidarUart) {
        s.range = 999.0f;
        s.health.lidarHealthy = false;
        return;
    }
    uint8_t buf[64];
    ssize_t n = _cfg.lidarUart->read(buf, sizeof(buf), 20);
    if (n <= 0) return;
    for (ssize_t i = 0; i + 8 < n; ++i) {
        if (buf[i] == 0x59 && buf[i+1] == 0x59) {
            uint16_t dist = static_cast<uint16_t>(buf[i+2]) | (static_cast<uint16_t>(buf[i+3]) << 8);
            s.range = static_cast<float>(dist) / 100.0f; // cm -> m
            s.health.lidarHealthy = true;
            return;
        }
    }
    s.health.lidarHealthy = false;
}
