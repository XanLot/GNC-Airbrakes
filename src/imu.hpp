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
    uint8_t cs_pin;              // SPI chip select pin
    uint8_t accel_range;         // 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t gyro_range;          // 0=250, 1=500, 2=1000, 3=2000 dps
    uint8_t mag_data_rate;       // AK09916 data rate enum (see Adafruit_ICM20948.h)
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

    // Initialize the IMU with given configuration
    bool init(const IMUConfig& config);

    // Poll sensor for new data. Returns true if new data was read.
    bool update();

    // Read individual sensor data (from last update() call)
    Vec3 readGyro() const;
    Vec3 readAccel() const;
    Vec3 readMag() const;

    // Read all sensor data at once
    IMUData readAll() const;

private:
    bool initialized_;
    IMUData latest_;
};

#endif // IMU_HPP
