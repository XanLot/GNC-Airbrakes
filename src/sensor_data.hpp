#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <cstdint>
#include <cmath>

struct Vec3 {
    float x, y, z;
};

struct IMUData {
    Vec3 accel;    // m/s^2
    Vec3 gyro;     // rad/s
    float temp;    // degrees C (on-die)
};

struct BarometerData {
    float temperature;  // degrees C
    float pressure;     // Pa
    float altitude;     // m
};

struct MagData {
    Vec3 field;  // Gauss
};

struct TempData {
    float temperature;  // degrees C
};

struct SensorData {
    IMUData        imu[4];
    BarometerData  baro[2];
    MagData        mag;
    TempData       tmp;
    unsigned long  timestamp_us;
};

// fill with NAN for failed sensors
inline IMUData nanIMU() {
    return IMUData{
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN
    };
}

inline BarometerData nanBaro() {
    return BarometerData{NAN, NAN, NAN};
}

inline MagData nanMag() {
    return MagData{{NAN, NAN, NAN}};
}

inline TempData nanTemp() {
    return TempData{NAN};
}

#endif // SENSOR_DATA_HPP
