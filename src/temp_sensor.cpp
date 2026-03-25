#include "temp_sensor.hpp"
#include <SparkFun_TMP117.h>
#include <Wire.h>

struct TempSensor::Impl {
    TMP117 tmp;
};

TempSensor::TempSensor()
    : initialized_(false), latest_{}, pimpl_(new Impl) {}

TempSensor::~TempSensor() {
    delete pimpl_;
}

bool TempSensor::init() {
    // Wire.begin() / setClock() called by magnetometer init — share the bus
    if (!pimpl_->tmp.begin()) return false;

    // continuous mode, 8x averaging
    pimpl_->tmp.setConversionAverageMode(1);  // AVG=01 -> 8 conversions
    pimpl_->tmp.setContinuousConversionMode();

    initialized_ = true;
    return true;
}

bool TempSensor::update() {
    if (!initialized_) return false;

    if (pimpl_->tmp.dataReady()) {
        latest_.temperature = static_cast<float>(pimpl_->tmp.readTempC());
    }
    return true;
}

TempData TempSensor::readAll() const {
    return latest_;
}
