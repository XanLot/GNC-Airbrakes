#include "magnetometer.hpp"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>

Magnetometer::Magnetometer()
    : sensor_(new SFE_MMC5983MA), initialized_(false), latest_{} {}

Magnetometer::~Magnetometer() {
    delete sensor_;
}

bool Magnetometer::init() {
    Wire.begin();
    Wire.setClock(400000);

    if (!sensor_->begin()) {
        return false;
    }

    // degauss and reset before config
    sensor_->softReset();

    // auto SET/RESET: critical after motor burn degrades the coil
    sensor_->enableAutomaticSetReset();

    // continuous mode at 50 Hz, 200 Hz BW filter
    sensor_->setContinuousModeFrequency(50);
    sensor_->enableContinuousMode();
    sensor_->setFilterBandwidth(200);

    initialized_ = true;
    return true;
}

bool Magnetometer::update() {
    if (!initialized_) {
        return false;
    }

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    sensor_->getMeasurementXYZ(&rawX, &rawY, &rawZ);

    // 18-bit unsigned: midpoint 131072, sensitivity 16384 LSB/Gauss
    latest_.field.x = (static_cast<float>(rawX) - 131072.0f) / 16384.0f;
    latest_.field.y = (static_cast<float>(rawY) - 131072.0f) / 16384.0f;
    latest_.field.z = (static_cast<float>(rawZ) - 131072.0f) / 16384.0f;
    return true;
}

MagData Magnetometer::readAll() const {
    return latest_;
}
