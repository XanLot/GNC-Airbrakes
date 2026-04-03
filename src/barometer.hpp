#ifndef BAROMETER_HPP
#define BAROMETER_HPP

#include "sensor_data.hpp"
#include <cstdint>
#include <SPI.h>

struct BarometerConfig {
    uint32_t  spi_speed;
    // BMP5_OVERSAMPLING_1X .. BMP5_OVERSAMPLING_128X
    uint8_t   pressure_osr;
    uint8_t   temp_osr;
    // BMP5_ODR_50_HZ = 0x0F, etc.
    uint8_t   odr;
    // BMP5_IIR_FILTER_BYPASS .. BMP5_IIR_FILTER_COEFF_127
    uint8_t   iir_pressure;
    uint8_t   iir_temp;
    float     sea_level_hpa;
};

class BMP581;

class Barometer {
public:
    Barometer();
    ~Barometer();

    bool init(const BarometerConfig& config, uint8_t cs_pin, SPIClass* bus);
    bool update();
    BarometerData readAll() const;

    static const BarometerConfig flightConfig;
    static const BarometerConfig debugConfig;

    void setSeaLevelPressure(float hpa);

private:
    BMP581*       sensor_;
    bool          initialized_;
    float         sea_level_hpa_;
    BarometerData latest_;
};

#endif // BAROMETER_HPP
