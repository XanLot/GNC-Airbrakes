#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include "sensor_data.hpp"

class TempSensor {
public:
    TempSensor();
    ~TempSensor();

    bool init();
    bool update();
    TempData readAll() const;

private:
    bool     initialized_;
    TempData latest_;
    struct Impl;
    Impl*    pimpl_;
};

#endif // TEMP_SENSOR_HPP
