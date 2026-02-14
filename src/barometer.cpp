#include "barometer.hpp"
#include "Adafruit_BMP3XX.h"
#include <cmath>

// Internal BMP3XX instance
static Adafruit_BMP3XX bmp;

Barometer::Barometer()
    : initialized_(false),
      sea_level_pressure_hpa_(1013.25f),
      temperature_(0.0f),
      pressure_(0.0f) {}

BarometerConfig Barometer::defaultConfig() {
    return BarometerConfig{
        .cs_pin = 33,
        .spi_speed = 1000000,
        .temperature_oversampling = 3,  // BMP3_OVERSAMPLING_8X
        .pressure_oversampling = 2,     // BMP3_OVERSAMPLING_4X
        .iir_filter_coeff = 2,          // BMP3_IIR_FILTER_COEFF_3
        .output_data_rate = 2,          // BMP3_ODR_50_HZ
        .sea_level_pressure_hpa = 1013.25f
    };
}

bool Barometer::init(const BarometerConfig& config) {
    sea_level_pressure_hpa_ = config.sea_level_pressure_hpa;

    if (!bmp.begin_SPI(config.cs_pin, &SPI, config.spi_speed)) {
        return false;
    }

    bmp.setTemperatureOversampling(config.temperature_oversampling);
    bmp.setPressureOversampling(config.pressure_oversampling);
    bmp.setIIRFilterCoeff(config.iir_filter_coeff);
    bmp.setOutputDataRate(config.output_data_rate);

    // Discard the first reading as it may be inaccurate
    bmp.performReading();

    initialized_ = true;
    return true;
}

bool Barometer::update() {
    if (!initialized_) {
        return false;
    }

    if (!bmp.performReading()) {
        return false;
    }

    temperature_ = bmp.temperature;
    pressure_ = bmp.pressure;
    return true;
}

float Barometer::temperature() const {
    return temperature_;
}

float Barometer::pressure() const {
    return pressure_;
}

float Barometer::altitude() const {
    float atmospheric = pressure_ / 100.0f;
    return 44330.0f * (1.0f - powf(atmospheric / sea_level_pressure_hpa_, 0.1903f));
}

BarometerData Barometer::readAll() {
    return BarometerData{
        .temperature = temperature_,
        .pressure = pressure_,
        .altitude = altitude()
    };
}

void Barometer::setSeaLevelPressure(float hpa) {
    sea_level_pressure_hpa_ = hpa;
}
