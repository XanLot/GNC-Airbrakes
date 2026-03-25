#include "barometer.hpp"
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SPI.h>
#include <cmath>

Barometer::Barometer()
    : sensor_(new BMP581), initialized_(false), sea_level_hpa_(1013.25f), latest_{} {}

Barometer::~Barometer() {
    delete sensor_;
}

BarometerConfig Barometer::flightConfig(uint8_t cs_pin, SPIClass* bus) {
    return BarometerConfig{
        .cs_pin        = cs_pin,
        .spi_bus       = bus,
        .spi_speed     = 8000000,
        .pressure_osr  = BMP5_OVERSAMPLING_8X,
        .temp_osr      = BMP5_OVERSAMPLING_1X,
        .odr           = BMP5_ODR_50_HZ,
        .iir_pressure  = BMP5_IIR_FILTER_COEFF_7,
        .iir_temp      = BMP5_IIR_FILTER_BYPASS,
        .sea_level_hpa = 1013.25f
    };
}

BarometerConfig Barometer::debugConfig(uint8_t cs_pin, SPIClass* bus) {
    return BarometerConfig{
        .cs_pin        = cs_pin,
        .spi_bus       = bus,
        .spi_speed     = 4000000,
        .pressure_osr  = BMP5_OVERSAMPLING_4X,
        .temp_osr      = BMP5_OVERSAMPLING_1X,
        .odr           = BMP5_ODR_25_HZ,
        .iir_pressure  = BMP5_IIR_FILTER_COEFF_3,
        .iir_temp      = BMP5_IIR_FILTER_BYPASS,
        .sea_level_hpa = 1013.25f
    };
}

bool Barometer::init(const BarometerConfig& config) {
    sea_level_hpa_ = config.sea_level_hpa;

    SPIClass& bus = config.spi_bus ? *config.spi_bus : SPI;  // default to SPI0 if no bus specified

    // library never sets CS pin mode — must do it before begin()
    pinMode(config.cs_pin, OUTPUT);
    digitalWrite(config.cs_pin, HIGH);

    int8_t err = sensor_->beginSPI(config.cs_pin, config.spi_speed, bus);
    if (err != BMP5_OK) {
        return false;
    }

    bmp5_osr_odr_press_config osrCfg{};
    osrCfg.osr_t    = config.temp_osr;
    osrCfg.osr_p    = config.pressure_osr;
    osrCfg.press_en = BMP5_ENABLE;
    osrCfg.odr      = config.odr;
    err = sensor_->setOSRMultipliers(&osrCfg);
    if (err != BMP5_OK) {
        return false;
    }

    bmp5_iir_config iirCfg{};
    iirCfg.set_iir_t      = config.iir_temp;
    iirCfg.set_iir_p      = config.iir_pressure;
    iirCfg.shdw_set_iir_t = config.iir_temp;
    iirCfg.shdw_set_iir_p = config.iir_pressure;
    err = sensor_->setFilterConfig(&iirCfg);
    if (err != BMP5_OK) {
        return false;
    }

    err = sensor_->setMode(BMP5_POWERMODE_NORMAL);
    if (err != BMP5_OK) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool Barometer::update() {
    if (!initialized_) {
        return false;
    }

    bmp5_sensor_data data{};
    int8_t err = sensor_->getSensorData(&data);
    if (err != BMP5_OK) {
        return false;
    }

    latest_.temperature = data.temperature;
    latest_.pressure    = data.pressure;
    float atm = latest_.pressure / 100.0f;  // Pa -> hPa
    latest_.altitude = 44330.0f * (1.0f - powf(atm / sea_level_hpa_, 0.1903f));
    return true;
}

BarometerData Barometer::readAll() const {
    return latest_;
}

void Barometer::setSeaLevelPressure(float hpa) {
    sea_level_hpa_ = hpa;
}
