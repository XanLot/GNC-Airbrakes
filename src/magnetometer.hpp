#ifndef MAGNETOMETER_HPP
#define MAGNETOMETER_HPP

#include "sensor_data.hpp"

class SFE_MMC5983MA;

class Magnetometer {
public:
    Magnetometer();
    ~Magnetometer();

    bool init();
    bool update();
    MagData readAll() const;

private:
    SFE_MMC5983MA* sensor_;
    bool           initialized_;
    MagData        latest_;
};

#endif // MAGNETOMETER_HPP
