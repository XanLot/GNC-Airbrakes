#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>

// 3D vector for gyro, accel, magnetometer data
struct Vec3 {
    float x;
    float y;
    float z;
};

// IMU configuration
struct IMUConfig {
    uint8_t  cs_pin;                // SPI chip select pin
    uint8_t  accel_range;           // ICM20948_ACCEL_RANGE_2_G / _4 / _8 / _16
    uint8_t  gyro_range;            // ICM20948_GYRO_RANGE_250_DPS / _500 / _1000 / _2000
    uint8_t  mag_data_rate;         // AK09916_MAG_DATARATE_* enum value
    uint8_t  accel_dlpf_cfg;        // 0-7: DLPF bandwidth. 4 = 23.9 Hz. 0 = 246 Hz (off).
    uint8_t  gyro_dlpf_cfg;         // 0-7: same scale as accel_dlpf_cfg
    uint16_t accel_sample_rate_div; // Accel ODR = 1125 / (1 + div). e.g. div=10 → 102 Hz
    uint8_t  gyro_sample_rate_div;  // Gyro ODR  = 1100 / (1 + div). e.g. div=10 → 100 Hz
};

// Combined IMU data reading
struct IMUData {
    Vec3 gyro;       // rad/s
    Vec3 accel;      // m/s^2
    Vec3 mag;        // microtesla
    float temp;      // degrees C
};

class IMU {
public:
    IMU();

    // Returns a default configuration
    static IMUConfig defaultConfig();

    static IMUConfig flightConfig();    // ±8g, 23.9 Hz LPF, 100 Hz ODR — use for actual flights
    static IMUConfig lowNoiseConfig();  // ±4g, 11.5 Hz LPF, 50 Hz ODR  — use for ground testing

    // Initialize the IMU with given configuration
    bool init(const IMUConfig& config);

    // Change sensor settings at runtime (skips SPI init).
    // Call this to swap presets mid-flight, e.g. lowNoiseConfig() on pad → flightConfig() at boost.
    bool reconfigure(const IMUConfig& config);

    // Poll sensor for new data. Returns true if new data was read.
    bool update();

    // Read individual sensor data (from last update() call)
    Vec3 readGyro() const;
    Vec3 readAccel() const;
    Vec3 readMag() const;

    // Read all sensor data at once
    IMUData readAll() const;

private:
    void applyConfig(const IMUConfig& config);
    bool initialized_;
    IMUData latest_;
};

#endif // IMU_HPP
