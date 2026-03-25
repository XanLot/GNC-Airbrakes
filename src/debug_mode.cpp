#ifdef DEBUG_MODE

#include "debug_mode.hpp"
#include <Arduino.h>
#include <cmath>

// newlib-nano printf doesn't handle NaN — print explicitly
static void pf(float v, int decimals) {
    if (isnan(v)) {
        Serial.print("nan");
    } else {
        Serial.print(v, decimals);
    }
}

static void printIMU(const char* tag, const IMUData& imu) {
    Serial.print(tag); Serial.print(',');
    pf(imu.accel.x, 4); Serial.print(',');
    pf(imu.accel.y, 4); Serial.print(',');
    pf(imu.accel.z, 4); Serial.print(',');
    pf(imu.gyro.x,  4); Serial.print(',');
    pf(imu.gyro.y,  4); Serial.print(',');
    pf(imu.gyro.z,  4); Serial.print(',');
    pf(imu.temp,    2); Serial.print('\n');
}

static void printBaro(const char* tag, const BarometerData& baro) {
    Serial.print(tag); Serial.print(',');
    pf(baro.temperature, 2); Serial.print(',');
    pf(baro.pressure,    1); Serial.print(',');
    pf(baro.altitude,    2); Serial.print('\n');
}

static void printMag(const char* tag, const MagData& m) {
    Serial.print(tag); Serial.print(',');
    pf(m.field.x, 4); Serial.print(',');
    pf(m.field.y, 4); Serial.print(',');
    pf(m.field.z, 4); Serial.print('\n');
}

static void printTemp(const char* tag, const TempData& t) {
    Serial.print(tag); Serial.print(',');
    pf(t.temperature, 4); Serial.print('\n');
}

void debugPrint(const SensorData& data) {
    printIMU("$IMU1", data.imu[0]);
    printIMU("$IMU2", data.imu[1]);
    printIMU("$IMU3", data.imu[2]);
    printIMU("$IMU4", data.imu[3]);
    printBaro("$BARO1", data.baro[0]);
    printBaro("$BARO2", data.baro[1]);
    printMag("$MAG",   data.mag);
    printTemp("$TMP1", data.tmp[0]);
    printTemp("$TMP2", data.tmp[1]);
    Serial.printf("$TICK,%lu\n", data.timestamp_us);
}

#endif // DEBUG_MODE
