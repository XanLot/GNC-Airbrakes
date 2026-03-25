#ifndef MAGNETOMETER_HPP
#define MAGNETOMETER_HPP

#include "sensor_data.hpp"

class Magnetometer {
public:
    Magnetometer();
    ~Magnetometer();

    bool init();
    bool update();
    MagData readAll() const;

private:
    bool    initialized_;
    MagData latest_;
    struct Impl;
    Impl*   pimpl_;
};

#endif // MAGNETOMETER_HPP
