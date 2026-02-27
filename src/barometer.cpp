#include "barometer.hpp"
#include "Adafruit_BMP3XX.h"
#include <cmath>

// Internal BMP3XX instance
static Adafruit_BMP3XX bmp;

Barometer::Barometer()
    : initialized_(false),
      sea_level_pressure_hpa_(1013.25f),
      temperature_(0.0f),
      pressure_(0.0f),
      power_mode_(0) {}

BarometerConfig Barometer::defaultConfig() {
    return BarometerConfig{
        .cs_pin = 33,
        .spi_speed = 1000000,
        .temperature_oversampling = 3,  // BMP3_OVERSAMPLING_8X
        .pressure_oversampling = 2,     // BMP3_OVERSAMPLING_4X
        .iir_filter_coeff = 2,          // BMP3_IIR_FILTER_COEFF_3
        .output_data_rate = 2,          // BMP3_ODR_50_HZ
        .sea_level_pressure_hpa = 1013.25f,
        .power_mode = 0
    };
}

BarometerConfig Barometer::flightConfig() {
    return BarometerConfig{
        .cs_pin                    = 33,
        .spi_speed                 = 1000000,
        .temperature_oversampling  = 1,      // BMP3_OVERSAMPLING_2X
        .pressure_oversampling     = 3,      // BMP3_OVERSAMPLING_8X
        .iir_filter_coeff          = 4,      // BMP3_IIR_FILTER_COEFF_15
        .output_data_rate          = 2,      // BMP3_ODR_50_HZ
        .sea_level_pressure_hpa    = 1013.25f,
        .power_mode                = 1       // normal mode
    };
}

BarometerConfig Barometer::highRateConfig() {
    return BarometerConfig{
        .cs_pin                    = 33,
        .spi_speed                 = 1000000,
        .temperature_oversampling  = 0,      // BMP3_OVERSAMPLING_1X
        .pressure_oversampling     = 2,      // BMP3_OVERSAMPLING_4X
        .iir_filter_coeff          = 2,      // BMP3_IIR_FILTER_COEFF_3
        .output_data_rate          = 0,      // BMP3_ODR_200_HZ
        .sea_level_pressure_hpa    = 1013.25f,
        .power_mode                = 1       // normal mode
    };
}

bool Barometer::init(const BarometerConfig& config) {
    sea_level_pressure_hpa_ = config.sea_level_pressure_hpa;
    power_mode_ = config.power_mode;

    if (!bmp.begin_SPI(config.cs_pin, &SPI, config.spi_speed)) {
        return false;
    }

    bmp.setTemperatureOversampling(config.temperature_oversampling);
    bmp.setPressureOversampling(config.pressure_oversampling);
    bmp.setIIRFilterCoeff(config.iir_filter_coeff);
    bmp.setOutputDataRate(config.output_data_rate);

    if (power_mode_ == 1) {
        // TODO: set normal mode when Adafruit_BMP3XX exposes setOperationMode().
        // The underlying Bosch driver supports BMP3_MODE_NORMAL via bmp3_set_op_mode(),
        // but the Adafruit wrapper hardcodes BMP3_MODE_FORCED in performReading()
        // and keeps the_sensor private. For now, fall back to forced mode via performReading().
    }

    // Discard the first reading as it may be inaccurate
    bmp.performReading();

    initialized_ = true;
    return true;
}

bool Barometer::update() {
    if (!initialized_) {
        return false;
    }

    if (power_mode_ == 1) {
        // TODO: use non-blocking data-ready check when Adafruit_BMP3XX supports normal mode.
        // The Bosch driver can read data via bmp3_get_sensor_data() in normal mode,
        // but the Adafruit wrapper always triggers a forced-mode conversion in performReading().
        // Fall back to performReading() for now.
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
