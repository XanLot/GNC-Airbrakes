#ifdef DEBUG_MODE

#include "debug_mode.hpp"
#include <Arduino.h>

static void printIMU(const char* tag, const IMUData& imu) {
    Serial.printf("%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
        tag,
        imu.accel.x, imu.accel.y, imu.accel.z,
        imu.gyro.x,  imu.gyro.y,  imu.gyro.z,
        imu.temp);
}

static void printBaro(const char* tag, const BarometerData& baro) {
    Serial.printf("%s,%.2f,%.1f,%.2f\n",
        tag,
        baro.temperature, baro.pressure, baro.altitude);
}

static void printMag(const char* tag, const MagData& m) {
    Serial.printf("%s,%.4f,%.4f,%.4f\n",
        tag,
        m.field.x, m.field.y, m.field.z);
}

static void printTemp(const char* tag, const TempData& t) {
    Serial.printf("%s,%.4f\n", tag, t.temperature);
}

void debugPrint(const SensorData& data) {
    printIMU("$IMU1", data.imu[0]);
    printIMU("$IMU2", data.imu[1]);
    printIMU("$IMU3", data.imu[2]);
    printIMU("$IMU4", data.imu[3]);
    printBaro("$BARO1", data.baro[0]);
    printBaro("$BARO2", data.baro[1]);
    printMag("$MAG",  data.mag);
    printTemp("$TMP", data.tmp);
    Serial.printf("$TICK,%lu\n", data.timestamp_us);
}

#endif // DEBUG_MODE
