#include "imu.hpp"
#include "Teensy-ICM-20948.h"

// Internal ICM-20948 instance
static TeensyICM20948 icm20948;

IMU::IMU() : initialized_(false) {}

IMUConfig IMU::defaultConfig() {
    return IMUConfig{
        .cs_pin = 24,
        .spi_speed = 1000000,
        .mode = 1,
        .enable_gyroscope = true,
        .enable_accelerometer = true,
        .enable_magnetometer = true,
        .enable_quaternion = true,
        .gyroscope_frequency = 1,
        .accelerometer_frequency = 1,
        .magnetometer_frequency = 1,
        .quaternion_frequency = 50
    };
}

void IMU::init(const IMUConfig& config) {
    TeensyICM20948Settings settings = {
        .cs_pin = config.cs_pin,
        .spi_speed = config.spi_speed,
        .mode = config.mode,
        .enable_gyroscope = config.enable_gyroscope,
        .enable_accelerometer = config.enable_accelerometer,
        .enable_magnetometer = config.enable_magnetometer,
        .enable_quaternion = config.enable_quaternion,
        .gyroscope_frequency = config.gyroscope_frequency,
        .accelerometer_frequency = config.accelerometer_frequency,
        .magnetometer_frequency = config.magnetometer_frequency,
        .quaternion_frequency = config.quaternion_frequency
    };

    icm20948.init(settings);
    initialized_ = true;
}

void IMU::update() {
    if (initialized_) {
        icm20948.task();
    }
}

bool IMU::gyroReady() const {
    return initialized_ && icm20948.gyroDataIsReady();
}

bool IMU::accelReady() const {
    return initialized_ && icm20948.accelDataIsReady();
}

bool IMU::magReady() const {
    return initialized_ && icm20948.magDataIsReady();
}

bool IMU::quatReady() const {
    return initialized_ && icm20948.quatDataIsReady();
}

Vec3 IMU::readGyro() {
    Vec3 data = {0, 0, 0};
    if (initialized_) {
        icm20948.readGyroData(&data.x, &data.y, &data.z);
    }
    return data;
}

Vec3 IMU::readAccel() {
    Vec3 data = {0, 0, 0};
    if (initialized_) {
        icm20948.readAccelData(&data.x, &data.y, &data.z);
    }
    return data;
}

Vec3 IMU::readMag() {
    Vec3 data = {0, 0, 0};
    if (initialized_) {
        icm20948.readMagData(&data.x, &data.y, &data.z);
    }
    return data;
}

Quaternion IMU::readQuat() {
    Quaternion data = {0, 0, 0, 0};
    if (initialized_) {
        icm20948.readQuatData(&data.w, &data.x, &data.y, &data.z);
    }
    return data;
}

IMUData IMU::readAll() {
    IMUData data = {};
    if (initialized_) {
        icm20948.readGyroData(&data.gyro.x, &data.gyro.y, &data.gyro.z);
        icm20948.readAccelData(&data.accel.x, &data.accel.y, &data.accel.z);
        icm20948.readMagData(&data.mag.x, &data.mag.y, &data.mag.z);
        icm20948.readQuatData(&data.quat.w, &data.quat.x, &data.quat.y, &data.quat.z);
    }
    return data;
}
