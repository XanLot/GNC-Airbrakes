#ifndef PINS_HPP
#define PINS_HPP

#include <cstdint>

// SPI0 bus sensors (MOSI=11, MISO=12, SCK=13)
constexpr uint8_t IMU1_CS  = 37;
constexpr uint8_t IMU2_CS  = 38;
constexpr uint8_t BARO1_CS = 10;

// SPI1 bus sensors (MOSI=26, MISO=39, SCK=27)
constexpr uint8_t IMU3_CS  = 28;
constexpr uint8_t IMU4_CS  = 33;
constexpr uint8_t BARO2_CS = 34;

// I2C addresses (Wire, SDA=18, SCL=19)
constexpr uint8_t MAG_I2C_ADDR   = 0x30;
constexpr uint8_t TEMP1_I2C_ADDR = 0x48;
constexpr uint8_t TEMP2_I2C_ADDR = 0x49;

#endif // PINS_HPP
