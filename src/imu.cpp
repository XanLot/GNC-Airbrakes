#include "imu.hpp"
#include <Adafruit_ICM20948.h>
#include <SPI.h>

static Adafruit_ICM20948 icm20948;

IMU::IMU() : initialized_(false), latest_{} {}

IMUConfig IMU::defaultConfig() {
    return IMUConfig{
        .cs_pin               = 7,
        .accel_range          = ICM20948_ACCEL_RANGE_16_G,
        .gyro_range           = ICM20948_GYRO_RANGE_2000_DPS,
        .mag_data_rate        = AK09916_MAG_DATARATE_100_HZ,
        .accel_dlpf_cfg       = 0,
        .gyro_dlpf_cfg        = 0,
        .accel_sample_rate_div = 0,
        .gyro_sample_rate_div  = 0
    };
}

IMUConfig IMU::flightConfig() {
    // Tuned for state estimation on a ~2.2 kg rocket (H/I class motor).
    // 225 Hz ODR gives the Kalman filter ~4-5 IMU updates per barometer reading,
    // which tightens velocity integration during the 5-10s coast phase.
    // ±8g fits 6.3g peak with margin. DLPF at 23.9 Hz kills motor vibration
    // while keeping group delay at ~21ms — fast enough for airbrake control.
    return IMUConfig{
        .cs_pin               = 7,
        .accel_range          = ICM20948_ACCEL_RANGE_8_G,   // 6.3g peak fits, 2x less noise than ±16g
        .gyro_range           = ICM20948_GYRO_RANGE_500_DPS, // small rocket won't spin fast
        .mag_data_rate        = AK09916_MAG_DATARATE_100_HZ,
        .accel_dlpf_cfg       = 4,    // 23.9 Hz — filters motor vibration, ~21ms group delay
        .gyro_dlpf_cfg        = 4,    // 23.9 Hz — matches accel filter
        .accel_sample_rate_div = 4,    // 1125 / 5 = 225 Hz — fast updates for state estimator
        .gyro_sample_rate_div  = 4     // 1100 / 5 = 220 Hz
    };
}

IMUConfig IMU::lowNoiseConfig() {
    return IMUConfig{
        .cs_pin               = 7,
        .accel_range          = ICM20948_ACCEL_RANGE_4_G,
        .gyro_range           = ICM20948_GYRO_RANGE_250_DPS,
        .mag_data_rate        = AK09916_MAG_DATARATE_100_HZ,
        .accel_dlpf_cfg       = 5,    // 11.5 Hz — aggressive filtering
        .gyro_dlpf_cfg        = 5,    // 11.5 Hz
        .accel_sample_rate_div = 21,   // 1125 / 22 ≈ 51 Hz
        .gyro_sample_rate_div  = 21    // 1100 / 22 = 50 Hz
    };
}

bool IMU::init(const IMUConfig& config) {
    if (!icm20948.begin_SPI(config.cs_pin)) {
        return false;
    }
    applyConfig(config);
    initialized_ = true;
    return true;
}

bool IMU::reconfigure(const IMUConfig& config) {
    if (!initialized_) {
        return false;
    }
    applyConfig(config);
    return true;
}

void IMU::applyConfig(const IMUConfig& config) {
    icm20948.setAccelRange(static_cast<icm20948_accel_range_t>(config.accel_range));
    icm20948.setGyroRange(static_cast<icm20948_gyro_range_t>(config.gyro_range));
    icm20948.setMagDataRate(static_cast<ak09916_data_rate_t>(config.mag_data_rate));

    // Set sample rate divisors (controls ODR)
    icm20948.setAccelRateDivisor(config.accel_sample_rate_div);
    icm20948.setGyroRateDivisor(config.gyro_sample_rate_div);

    // Set DLPF (digital low-pass filter) configuration
    if (config.accel_dlpf_cfg > 0) {
        icm20948.enableAccelDLPF(true, static_cast<icm20x_accel_cutoff_t>(config.accel_dlpf_cfg));
    } else {
        icm20948.enableAccelDLPF(false, ICM20X_ACCEL_FREQ_246_0_HZ);
    }
    if (config.gyro_dlpf_cfg > 0) {
        icm20948.enableGyrolDLPF(true, static_cast<icm20x_gyro_cutoff_t>(config.gyro_dlpf_cfg));
    } else {
        icm20948.enableGyrolDLPF(false, ICM20X_GYRO_FREQ_196_6_HZ);
    }
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
