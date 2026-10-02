#include "SensorManager.hpp"

#include <math.h>
#include <spdlog/spdlog.h>

#include <cmath>

#include "I2CBus.hpp"
#include "UartPort.hpp"

namespace {
    // UBX helpers
    uint8_t UbxCkA(const uint8_t* buf, size_t len) {
        uint8_t a = 0;
        for (size_t i = 0; i < len; ++i) a += buf[i];
        return a;
    }
    uint8_t UbxCkB(const uint8_t* buf, size_t len) {
        uint8_t a = 0;
        uint8_t b = 0;
        for (size_t i = 0; i < len; ++i) {
            a += buf[i];
            b += a;
        }
        return b;
    }
} // namespace

//------------------------------------------------------------------------------------
SensorManager::SensorManager(const Config& cfg) :
    m_cfg(cfg), m_ahrs(cfg.m_madgwick_beta), m_last_ahrs_time(std::chrono::steady_clock::now()) {
    m_gps_buf.reserve(BUFFER_SIZE);
}

SensorManager::~SensorManager() {
    spdlog::debug("SensorManager stopped!");
}
//------------------------------------------------------------------------------------
bool SensorManager::Initialize() {
    // Ensure adapters exist; create defaults if not provided
    if (!m_cfg.m_i2c) {
        m_cfg.m_i2c = std::make_shared<HW::IF::I2CBus>("/dev/m_i2c-1");
    }
    if (!m_cfg.m_gps_uart) {
        m_cfg.m_gps_uart = std::make_shared<HW::IF::UartPort>("/dev/serial0");
        m_cfg.m_gps_uart->Open("/dev/serial0", GPS_BAUD_RATE);
    }
    if (!m_cfg.m_lidar_uart) {
        m_cfg.m_lidar_uart = std::make_shared<HW::IF::UartPort>("/dev/ttyS1");
        m_cfg.m_lidar_uart->Open("/dev/ttyS1", LIDAR_BAUD_RATE);
    }
    m_last_ahrs_time = std::chrono::steady_clock::now();
    return true;
}
//------------------------------------------------------------------------------------
void SensorManager::StepSm() {
    const auto period = std::chrono::milliseconds(1000 / std::max<uint32_t>(1, m_cfg.m_loop_hz));
    SensorData s;
    ReadImu(s);
    ReadBaro(s);
    ReadCompass(s);
    ReadGps(s);
    ReadAirspeed(s);
    ReadLidar(s);

    // AHRS update
    auto  now = std::chrono::steady_clock::now();
    float dt  = std::chrono::duration<float>(now - m_last_ahrs_time).count();
    if (dt <= 0.0F) {
        dt = 0.001F;
    }
    m_last_ahrs_time = now;

    m_ahrs.Update(s.m_gyro_x, s.m_gyro_y, s.m_gyro_z, s.m_accel_x, s.m_accel_y, s.m_accel_z, m_mag_z, m_mag_y, m_mag_z, dt);

    float roll  = NAN;
    float pitch = NAN;
    float yaw   = NAN;
    m_ahrs.GetEuler(roll, pitch, yaw);
    s.m_roll  = roll;
    s.m_pitch = pitch;
    s.m_yaw   = yaw;

    s.m_timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // callback
    {
        std::scoped_lock lk(m_cb_mutex);
        if (m_callback) {
            m_callback(s);
        }
    }

    std::this_thread::sleep_for(period);
}
//------------------------------------------------------------------------------------
void SensorManager::SetSensorCallback(SensorCallback cb) {
    std::scoped_lock lk(m_cb_mutex);
    m_callback = std::move(cb);
}

bool SensorManager::ReadOnce(SensorData& out) {
    ReadImu(out);
    ReadBaro(out);
    ReadCompass(out);
    ReadGps(out);
    ReadAirspeed(out);
    ReadLidar(out);

    // AHRS update
    auto  now = std::chrono::steady_clock::now();
    float dt  = std::chrono::duration<float>(now - m_last_ahrs_time).count();
    if (dt <= 0.0F) {
        dt = 0.001F;
    }
    m_last_ahrs_time = now;

    // gyro expected in rad/s; ensure readImu sets that
    m_ahrs.Update(out.m_gyro_x, out.m_gyro_y, out.m_gyro_z, out.m_accel_x, out.m_accel_y, out.m_accel_z, m_mag_x, m_mag_y,
                  m_mag_z, dt);

    float roll  = NAN;
    float pitch = NAN;
    float yaw   = NAN;
    m_ahrs.GetEuler(roll, pitch, yaw);
    out.m_roll  = roll;
    out.m_pitch = pitch;
    out.m_yaw   = yaw;

    out.m_timestamp_ms = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return true;
}

//--Low-level sensor readers-----------------------------------------
bool SensorManager::ReadI2CRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!m_cfg.m_i2c) {
        return false;
    }
    return m_cfg.m_i2c->Read(addr, reg, buf, len);
}

bool SensorManager::WriteI2CRegister(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    if (!m_cfg.m_i2c) {
        return false;
    }
    return m_cfg.m_i2c->Write(addr, reg, buf, len);
}

/* IMU: ICM20602 basic read */
void SensorManager::ReadImu(SensorData& s) {
    // Default simulated values if no I2C
    if (!m_cfg.m_i2c) {
        s.m_accel_x = 0.0f;
        s.m_accel_y = 0.0f;
        s.m_accel_z = -9.80665f;
        s.m_gyro_x = s.m_gyro_y = s.m_gyro_z = 0.0f;
        s.m_health.m_imu_healthy             = true;
        return;
    }

    // WHO_AM_I check
    uint8_t who = 0;
    if (!ReadI2CRegister(m_cfg.m_icm_addr, 0x75, &who, 1)) {
        s.m_health.m_imu_healthy = false;
        return;
    }

    // Read accel/gyro registers (ACCEL_XOUT_H = 0x3B, 14 bytes)
    uint8_t buf[14];
    if (!ReadI2CRegister(m_cfg.m_icm_addr, 0x3B, buf, 14)) {
        s.m_health.m_imu_healthy = false;
        return;
    }

    auto    toS16 = [](uint8_t hi, uint8_t lo) -> int16_t { return static_cast<int16_t>((hi << 8) | lo); };
    int16_t ax    = toS16(buf[0], buf[1]);
    int16_t ay    = toS16(buf[2], buf[3]);
    int16_t az    = toS16(buf[4], buf[5]);
    // temp ignored here
    int16_t gx = toS16(buf[8], buf[9]);
    int16_t gy = toS16(buf[10], buf[11]);
    int16_t gz = toS16(buf[12], buf[13]);

    // Convert: assume accel FS = ±2g, gyro FS = ±250 dps
    const float accelScale = 2.0f / 32768.0f * 9.80665f;          // m/s^2
    const float gyroScale  = 250.0f / 32768.0f * (M_PI / 180.0f); // rad/s

    s.m_accel_x = ax * accelScale;
    s.m_accel_y = ay * accelScale;
    s.m_accel_z = az * accelScale;

    s.m_gyro_x = gx * gyroScale;
    s.m_gyro_y = gy * gyroScale;
    s.m_gyro_z = gz * gyroScale;

    s.m_health.m_imu_healthy = true;
}

/* Barometer: MS5611 */
void SensorManager::ReadBaro(SensorData& s) {
    if (!m_cfg.m_i2c) {
        s.m_pressure              = 1013.25f;
        s.m_altitude_baro         = 100.0f;
        s.m_temperature           = 20.0f;
        s.m_health.m_baro_healthy = true;
        return;
    }

    // Minimal MS5611 sequence (reset + PROM + conversion)
    // Reset
    uint8_t cmd = 0x1E;
    WriteI2CRegister(m_cfg.m_ms5611_addr, cmd, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    // Read PROM coefficients
    uint16_t C[7] = {0};
    for (int i = 0; i < 6; ++i) {
        uint8_t buf[2];
        if (!ReadI2CRegister(m_cfg.m_ms5611_addr, 0xA2 + i * 2, buf, 2)) {
            s.m_health.m_baro_healthy = false;
            return;
        }
        C[i + 1] = (buf[0] << 8) | buf[1];
    }

    // D1 (m_pressure)
    uint8_t cmdD1 = 0x48; // OSR=4096
    WriteI2CRegister(m_cfg.m_ms5611_addr, cmdD1, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t d1buf[3];
    if (!ReadI2CRegister(m_cfg.m_ms5611_addr, 0x00, d1buf, 3)) {
        s.m_health.m_baro_healthy = false;
        return;
    }
    uint32_t D1 = (d1buf[0] << 16) | (d1buf[1] << 8) | d1buf[2];

    // D2 (temp)
    uint8_t cmdD2 = 0x58;
    WriteI2CRegister(m_cfg.m_ms5611_addr, cmdD2, nullptr, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t d2buf[3];
    if (!ReadI2CRegister(m_cfg.m_ms5611_addr, 0x00, d2buf, 3)) {
        s.m_health.m_baro_healthy = false;
        return;
    }
    uint32_t D2 = (d2buf[0] << 16) | (d2buf[1] << 8) | d2buf[2];

    // Datasheet calculations
    int64_t dT   = static_cast<int64_t>(D2) - (static_cast<int64_t>(C[5]) << 8);
    int64_t TEMP = 2000 + ((dT * C[6]) >> 23);
    int64_t OFF  = (static_cast<int64_t>(C[2]) << 16) + ((static_cast<int64_t>(C[4]) * dT) >> 7);
    int64_t SENS = (static_cast<int64_t>(C[1]) << 15) + ((static_cast<int64_t>(C[3]) * dT) >> 8);
    int64_t P    = (((static_cast<int64_t>(D1) * SENS) >> 21) - OFF) >> 15;

    s.m_temperature = TEMP / 100.0f;
    s.m_pressure    = P / 100.0f; // mbar

    // approximate altitude (ISA)
    const float seaLevelPressure = 1013.25f;
    s.m_altitude_baro            = 44330.0f * (1.0f - powf(s.m_pressure / seaLevelPressure, 0.1903f));

    s.m_health.m_baro_healthy = true;
}

/* Compass: HMC5883L */
void SensorManager::ReadCompass(SensorData& s) {
    if (!m_cfg.m_i2c) {
        m_mag_x = m_mag_y = m_mag_z = 0.0f;
        s.m_heading                 = 0.0f;
        return;
    }

    // Configure continuous measurement (CRA/CRB/MODE)
    uint8_t cfg = 0x00; // default
    WriteI2CRegister(m_cfg.m_hmc_addr, 0x00, &cfg, 1);

    uint8_t buf[6];
    if (!ReadI2CRegister(m_cfg.m_hmc_addr, 0x03, buf, 6)) {
        s.m_health.m_gps_fix = s.m_health.m_gps_fix; // no-op
        return;
    }

    int16_t mx = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t my = static_cast<int16_t>((buf[4] << 8) | buf[5]); // note axis order
    int16_t mz = static_cast<int16_t>((buf[2] << 8) | buf[3]);

    // store raw magnetometer for AHRS
    m_mag_x = static_cast<float>(mx);
    m_mag_y = static_cast<float>(my);
    m_mag_z = static_cast<float>(mz);

    // compute m_heading (simple)
    float m_heading = atan2f(m_mag_y, m_mag_x) * 180.0f / M_PI;
    if (m_heading < 0)
        m_heading += 360.0f;
    s.m_heading = m_heading;
}

/* GPS: Ublox NAV-PVT parsing (basic) */
void SensorManager::ReadGps(SensorData& s) {
    if (!m_cfg.m_gps_uart) {
        // simulated
        s.m_latitude         = 45.5;
        s.m_longitude        = -73.6;
        s.m_altitude_gps     = 100.0f;
        s.m_ground_speed     = 0.0f;
        s.m_gps_satellites   = 8;
        s.m_health.m_gps_fix = true;
        return;
    }

    // Read available bytes into buffer
    uint8_t buf[512];
    ssize_t n = m_cfg.m_gps_uart->Read(buf, sizeof(buf), 20);
    if (n > 0) {
        std::scoped_lock lk(m_gps_buf_mutex);
        m_gps_buf.insert(m_gps_buf.end(), buf, buf + n);
    }

    // Parse UBX messages from buffer
    std::scoped_lock lk(m_gps_buf_mutex);
    size_t           i = 0;
    while (i + 8 <= m_gps_buf.size()) {
        if (m_gps_buf[i] != 0xB5 || m_gps_buf[i + 1] != 0x62) {
            ++i;
            continue;
        }
        if (i + 6 > m_gps_buf.size())
            break;
        uint8_t  cls = m_gps_buf[i + 2];
        uint8_t  id  = m_gps_buf[i + 3];
        uint16_t len = static_cast<uint16_t>(m_gps_buf[i + 4]) | (static_cast<uint16_t>(m_gps_buf[i + 5]) << 8);
        if (i + 6 + len + 2 > m_gps_buf.size())
            break; // wait for full packet
        const uint8_t* payload = m_gps_buf.data() + i + 6;
        uint8_t        ck_a    = m_gps_buf[i + 6 + len];
        uint8_t        ck_b    = m_gps_buf[i + 6 + len + 1];

        // compute checksum over class,id,len,payload
        std::vector<uint8_t> ckbuf;
        ckbuf.reserve(4 + len);
        ckbuf.push_back(cls);
        ckbuf.push_back(id);
        ckbuf.push_back(static_cast<uint8_t>(len & 0xFF));
        ckbuf.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        for (size_t k = 0; k < len; ++k) ckbuf.push_back(payload[k]);
        uint8_t ca = UbxCkA(ckbuf.data(), ckbuf.size());
        uint8_t cb = UbxCkB(ckbuf.data(), ckbuf.size());
        if (ca == ck_a && cb == ck_b) {
            // NAV-PVT (class 0x01 id 0x07)
            if (cls == 0x01 && id == 0x07 && len >= 92) {
                auto lat    = static_cast<int32_t>(payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24));
                auto lon    = static_cast<int32_t>(payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24));
                auto height = static_cast<int32_t>(payload[32] | (payload[33] << 8) | (payload[34] << 16) | (payload[35] << 24));
                auto gSpeed = static_cast<int32_t>(payload[60] | (payload[61] << 8) | (payload[62] << 16) | (payload[63] << 24));
                uint8_t numSV        = payload[23];
                s.m_latitude         = lat * 1e-7;
                s.m_longitude        = lon * 1e-7;
                s.m_altitude_gps     = height / 1000.0f;
                s.m_ground_speed     = gSpeed / 1000.0f;
                s.m_gps_satellites   = numSV;
                s.m_health.m_gps_fix = true;
            }
        }
        // advance past this packet
        i += 6 + len + 2;
    }
    // erase consumed bytes
    if (i > 0)
        m_gps_buf.erase(m_gps_buf.begin(), m_gps_buf.begin() + i);
}

/* Airspeed: placeholder for digital pitot (I2C) */
void SensorManager::ReadAirspeed(SensorData& s) {
    if (!m_cfg.m_i2c) {
        s.m_airspeed                  = 0.0f;
        s.m_health.m_airspeed_healthy = false;
        return;
    }
    uint8_t addr = m_cfg.m_airspeed_addr;
    uint8_t buf[2];
    if (ReadI2CRegister(addr, 0x00, buf, 2)) {
        auto raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        // sensor-specific mapping required; here assume raw Pa*100
        float       diffPa = static_cast<float>(raw) / 100.0f;
        const float rho    = 1.225f;
        if (diffPa > 0.1f)
            s.m_airspeed = sqrtf(2.0f * diffPa / rho);
        else
            s.m_airspeed = 0.0f;
        s.m_health.m_airspeed_healthy = true;
    }
    else {
        s.m_airspeed                  = 0.0f;
        s.m_health.m_airspeed_healthy = false;
    }
}

/* LiDAR: TFmini UART parsing */
void SensorManager::ReadLidar(SensorData& s) {
    if (!m_cfg.m_lidar_uart) {
        s.m_range                  = 999.0f;
        s.m_health.m_lidar_healthy = false;
        return;
    }
    uint8_t buf[64];
    ssize_t n = m_cfg.m_lidar_uart->Read(buf, sizeof(buf), 20);
    if (n <= 0)
        return;
    for (ssize_t i = 0; i + 8 < n; ++i) {
        if (buf[i] == 0x59 && buf[i + 1] == 0x59) {
            uint16_t dist              = static_cast<uint16_t>(buf[i + 2]) | (static_cast<uint16_t>(buf[i + 3]) << 8);
            s.m_range                  = static_cast<float>(dist) / 100.0f; // cm -> m
            s.m_health.m_lidar_healthy = true;
            return;
        }
    }
    s.m_health.m_lidar_healthy = false;
}