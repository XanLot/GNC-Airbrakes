#include "magnetometer.hpp"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>

struct Magnetometer::Impl {
    SFE_MMC5983MA mag;
};

Magnetometer::Magnetometer()
    : initialized_(false), latest_{}, pimpl_(new Impl) {}

Magnetometer::~Magnetometer() {
    delete pimpl_;
}

bool Magnetometer::init() {
    Wire.begin();
    Wire.setClock(400000);

    if (!pimpl_->mag.begin()) return false;

    // degauss and reset before config
    pimpl_->mag.softReset();

    // auto SET/RESET: critical after motor burn degrades the coil
    pimpl_->mag.enableAutomaticSetReset();

    // continuous mode at 50 Hz, 200 Hz BW filter
    pimpl_->mag.setContinuousModeFrequency(50);
    pimpl_->mag.enableContinuousMode();
    pimpl_->mag.setFilterBandwidth(200);

    initialized_ = true;
    return true;
}

bool Magnetometer::update() {
    if (!initialized_) return false;

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    pimpl_->mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

    // 18-bit unsigned: midpoint 131072, sensitivity 16384 LSB/Gauss
    latest_.field.x = (static_cast<float>(rawX) - 131072.0f) / 16384.0f;
    latest_.field.y = (static_cast<float>(rawY) - 131072.0f) / 16384.0f;
    latest_.field.z = (static_cast<float>(rawZ) - 131072.0f) / 16384.0f;
    return true;
}

MagData Magnetometer::readAll() const {
    return latest_;
}
