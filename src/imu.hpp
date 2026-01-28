#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>

// 3D vector for gyro, accel, magnetometer data
struct Vec3 {
    float x;
    float y;
    float z;
};

// Quaternion for orientation
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

// IMU configuration
struct IMUConfig {
    uint8_t cs_pin;              // SPI chip select pin
    uint32_t spi_speed;          // SPI clock speed in Hz (max 7MHz)
    uint8_t mode;                // 0 = low power, 1 = high performance
    bool enable_gyroscope;
    bool enable_accelerometer;
    bool enable_magnetometer;
    bool enable_quaternion;
    uint8_t gyroscope_frequency;     // 1-225 Hz
    uint8_t accelerometer_frequency; // 1-225 Hz
    uint8_t magnetometer_frequency;  // 1-70 Hz
    uint8_t quaternion_frequency;    // 50-225 Hz
};

// Combined IMU data reading
struct IMUData {
    Vec3 gyro;       // degrees/second
    Vec3 accel;      // g
    Vec3 mag;        // microtesla
    Quaternion quat; // orientation
};

class IMU {
public:
    IMU();

    // Returns a default configuration
    static IMUConfig defaultConfig();

    // Initialize the IMU with given configuration
    void init(const IMUConfig& config);

    // Must be called frequently in main loop to update sensor values
    void update();

    // Check if new data is available
    bool gyroReady() const;
    bool accelReady() const;
    bool magReady() const;
    bool quatReady() const;

    // Read individual sensor data
    Vec3 readGyro();
    Vec3 readAccel();
    Vec3 readMag();
    Quaternion readQuat();

    // Read all sensor data at once
    IMUData readAll();

private:
    bool initialized_;
};

#endif // IMU_HPP
