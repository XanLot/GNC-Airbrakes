#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include "sensor_data.hpp"

class TMP117;
class TwoWire;

class TempSensor {
public:
    TempSensor();
    ~TempSensor();

    bool init(uint8_t i2c_addr = 0x48);
    bool update();
    TempData readAll() const;

private:
    TMP117*  sensor_;
    bool     initialized_;
    TempData latest_;
};

#endif // TEMP_SENSOR_HPP
