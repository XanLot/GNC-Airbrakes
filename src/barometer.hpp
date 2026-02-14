#ifndef BAROMETER_HPP
#define BAROMETER_HPP

#include <cstdint>

// Barometer configuration
struct BarometerConfig {
    uint8_t cs_pin;                    // SPI chip select pin
    uint32_t spi_speed;                // SPI clock speed in Hz
    uint8_t temperature_oversampling;  // 0=none, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x
    uint8_t pressure_oversampling;     // 0=none, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x
    uint8_t iir_filter_coeff;          // 0=off, 1=1, 2=3, 3=7, 4=15, 5=31, 6=63, 7=127
    uint8_t output_data_rate;          // 0=200Hz .. 0x11=0.001Hz
    float sea_level_pressure_hpa;      // sea level pressure in hPa for altitude calc
};

// Combined barometer data reading
struct BarometerData {
    float temperature;  // degrees Celsius
    float pressure;     // Pascals
    float altitude;     // meters (based on sea level pressure)
};

class Barometer {
public:
    Barometer();

    // Returns a default configuration
    static BarometerConfig defaultConfig();

    // Initialize the barometer with given configuration.
    // Returns true on success.
    bool init(const BarometerConfig& config);

    // Perform a sensor reading (blocking forced mode).
    // Returns true on success.
    bool update();

    // Get last read values
    float temperature() const;  // degrees Celsius
    float pressure() const;     // Pascals
    float altitude() const;     // meters

    // Read all sensor data at once
    BarometerData readAll();

    // Update the sea level pressure reference for altitude calculation
    void setSeaLevelPressure(float hpa);

private:
    bool initialized_;
    float sea_level_pressure_hpa_;
    float temperature_;
    float pressure_;
};

#endif // BAROMETER_HPP
