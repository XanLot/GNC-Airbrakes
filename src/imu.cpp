#include "imu.hpp"
#include <SparkFun_LSM6DSV16X.h>
#include <SPI.h>

// accel: mg -> m/s^2
static constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;
// gyro: mdps -> rad/s
static constexpr float MDPS_TO_RADS = (float)M_PI / 180.0f / 1000.0f;

IMU::IMU() : sensor_(nullptr), initialized_(false), latest_{} {}

IMU::~IMU() {
    delete sensor_;
}

const IMUConfig IMU::flightConfig = {
    .spi_speed        = 8000000,
    .accel_range      = LSM6DSV16X_8g,
    .gyro_range       = LSM6DSV16X_500dps,
    .accel_odr        = LSM6DSV16X_ODR_AT_240Hz,
    .gyro_odr         = LSM6DSV16X_ODR_AT_240Hz,
    .accel_lp2_enable = true,
    .gyro_lp1_enable  = true
};

const IMUConfig IMU::debugConfig = {
    .spi_speed        = 3000000,
    .accel_range      = LSM6DSV16X_4g,
    .gyro_range       = LSM6DSV16X_250dps,
    .accel_odr        = LSM6DSV16X_ODR_AT_60Hz,
    .gyro_odr         = LSM6DSV16X_ODR_AT_60Hz,
    .accel_lp2_enable = true,
    .gyro_lp1_enable  = true
};

bool IMU::init(const IMUConfig& config, uint8_t cs_pin, SPIClass* bus) {
    sensor_ = new SparkFun_LSM6DSV16X_SPI();

    // library never sets CS pin mode — must do it before begin()
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    SPISettings settings(config.spi_speed, MSBFIRST, SPI_MODE3);
    bool ok = bus
        ? sensor_->begin(*bus, settings, cs_pin)
        : sensor_->begin(cs_pin);

    if (!ok) {
        delete sensor_;
        sensor_ = nullptr;
        return false;
    }

    sensor_->setAccelFullScale(static_cast<lsm6dsv16x_xl_full_scale_t>(config.accel_range));
    sensor_->setGyroFullScale(static_cast<lsm6dsv16x_gy_full_scale_t>(config.gyro_range));
    sensor_->setAccelDataRate(static_cast<lsm6dsv16x_data_rate_t>(config.accel_odr));
    sensor_->setGyroDataRate(static_cast<lsm6dsv16x_data_rate_t>(config.gyro_odr));

    if (config.accel_lp2_enable) {
        sensor_->enableAccelLP2Filter();
    }
    if (config.gyro_lp1_enable) {
        sensor_->enableGyroLP1Filter();
    }

    initialized_ = true;
    return true;
}

bool IMU::update() {
    if (!initialized_) {
        return false;
    }

    sfe_lsm_data_t accel, gyro;
    int16_t rawTemp;

    if (!sensor_->getAccel(&accel)) {
        return false;
    }
    if (!sensor_->getGyro(&gyro)) {
        return false;
    }
    sensor_->getRawTemp(&rawTemp);

    latest_.accel = {accel.xData * MG_TO_MS2,
                     accel.yData * MG_TO_MS2,
                     accel.zData * MG_TO_MS2};
    latest_.gyro  = {gyro.xData * MDPS_TO_RADS,
                     gyro.yData * MDPS_TO_RADS,
                     gyro.zData * MDPS_TO_RADS};
    latest_.temp  = sensor_->convertToCelsius(rawTemp);
    return true;
}

IMUData IMU::readAll() const {
    return latest_;
}
