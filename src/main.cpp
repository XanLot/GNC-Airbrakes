/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>
#include "imu.hpp"
#include "barometer.hpp"
#include "sd_log_file.hpp"

IMU imu;
Barometer barometer;
sd_log sdLog;

void setup() {
    Serial.begin(115200);
    delay(500);

    if (!imu.init(IMU::defaultConfig())) {
        Serial.println("ICM-20948 init failed!");
    }

    if (!barometer.init(Barometer::defaultConfig())) {
        Serial.println("BMP388 init failed!");
    }

    if (!sdLog.init()) {
        Serial.println("SD card init failed!");
    }

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
    if (imu.update()) {
        Vec3 gyro = imu.readGyro();
        Serial.print("Gyro (rad/s): [");
        Serial.print(gyro.x); Serial.print(",");
        Serial.print(gyro.y); Serial.print(",");
        Serial.print(gyro.z); Serial.println("]");

        Vec3 accel = imu.readAccel();
        Serial.print("Accel (m/s^2): [");
        Serial.print(accel.x); Serial.print(",");
        Serial.print(accel.y); Serial.print(",");
        Serial.print(accel.z); Serial.println("]");

        Vec3 mag = imu.readMag();
        Serial.print("Mag (uT): [");
        Serial.print(mag.x); Serial.print(",");
        Serial.print(mag.y); Serial.print(",");
        Serial.print(mag.z); Serial.println("]");
    }

    if (barometer.update()) {
        BarometerData data = barometer.readAll();
        Serial.print("Temp (C): "); Serial.println(data.temperature);
        Serial.print("Pressure (hPa): "); Serial.println(data.pressure / 100.0);
        Serial.print("Altitude (m): "); Serial.println(data.altitude);
    }

    sdLog.log(imu.readAll(), barometer.readAll());
}
