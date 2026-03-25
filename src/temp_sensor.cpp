#include "temp_sensor.hpp"
#include <SparkFun_TMP117.h>
#include <Wire.h>

TempSensor::TempSensor()
    : sensor_(new TMP117), initialized_(false), latest_{} {}

TempSensor::~TempSensor() {
    delete sensor_;
}

bool TempSensor::init(uint8_t i2c_addr) {
    // Wire.begin() / setClock() called by magnetometer init — share the bus
    if (!sensor_->begin(i2c_addr, Wire)) {
        return false;
    }

    // continuous mode, 8x averaging
    sensor_->setConversionAverageMode(1);  // AVG=01 -> 8 conversions
    sensor_->setContinuousConversionMode();

    initialized_ = true;
    return true;
}

bool TempSensor::update() {
    if (!initialized_) {
        return false;
    }

    if (sensor_->dataReady()) {
        latest_.temperature = static_cast<float>(sensor_->readTempC());
    }
    return true;
}

TempData TempSensor::readAll() const {
    return latest_;
}
