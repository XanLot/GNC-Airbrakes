#include "imu.hpp"
#include <Adafruit_ICM20948.h>
#include <SPI.h>

static Adafruit_ICM20948 icm20948;

IMU::IMU() : initialized_(false), latest_{} {}

IMUConfig IMU::defaultConfig() {
    return IMUConfig{
        .cs_pin = 7,
        .accel_range = ICM20948_ACCEL_RANGE_16_G,
        .gyro_range = ICM20948_GYRO_RANGE_2000_DPS,
        .mag_data_rate = AK09916_MAG_DATARATE_100_HZ
    };
}

bool IMU::init(const IMUConfig& config) {
    if (!icm20948.begin_SPI(config.cs_pin)) {
        return false;
    }

    icm20948.setAccelRange(static_cast<icm20948_accel_range_t>(config.accel_range));
    icm20948.setGyroRange(static_cast<icm20948_gyro_range_t>(config.gyro_range));
    icm20948.setMagDataRate(static_cast<ak09916_data_rate_t>(config.mag_data_rate));

    initialized_ = true;
    return true;
}

bool IMU::update() {
    if (!initialized_) {
        return false;
    }

    sensors_event_t accel, gyro, temp, mag;
    if (!icm20948.getEvent(&accel, &gyro, &temp, &mag)) {
        return false;
    }

    latest_.accel = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
    latest_.gyro = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
    latest_.mag = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
    latest_.temp = temp.temperature;

    return true;
}

Vec3 IMU::readGyro() const {
    return latest_.gyro;
}

Vec3 IMU::readAccel() const {
    return latest_.accel;
}

Vec3 IMU::readMag() const {
    return latest_.mag;
}

IMUData IMU::readAll() const {
    return latest_;
}
