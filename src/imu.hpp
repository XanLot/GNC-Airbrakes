#ifndef IMU_HPP
#define IMU_HPP

#include "sensor_data.hpp"
#include <cstdint>
#include <SPI.h>

struct IMUConfig {
    uint8_t   cs_pin;
    SPIClass* spi_bus;
    uint32_t  spi_speed;
    // lsm6dsv16x_xl_full_scale_t: LSM6DSV16X_2g/4g/8g/16g
    uint8_t   accel_range;
    // lsm6dsv16x_gy_full_scale_t: LSM6DSV16X_125dps/.._250dps/.._500dps/..
    uint8_t   gyro_range;
    // lsm6dsv16x_data_rate_t: LSM6DSV16X_ODR_AT_240Hz etc
    uint8_t   accel_odr;
    uint8_t   gyro_odr;
    bool      accel_lp2_enable;
    bool      gyro_lp1_enable;
};

class SparkFun_LSM6DSV16X_SPI;

class IMU {
public:
    IMU();
    ~IMU();

    bool init(const IMUConfig& config);
    bool update();
    IMUData readAll() const;

    static IMUConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static IMUConfig debugConfig(uint8_t cs_pin, SPIClass* bus);

private:
    SparkFun_LSM6DSV16X_SPI* sensor_;
    bool    initialized_;
    IMUData latest_;
};

#endif // IMU_HPP
