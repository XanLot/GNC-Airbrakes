/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>
#include "imu.hpp"
#include "sd_log_file.hpp"

IMU imu;
sd_log sdLog;

void setup() {
    Serial.begin(115200);
    delay(500);

    imu.init(IMU::defaultConfig());

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
    imu.update();

    // Temporary storage for readings
    Vec3 gyro;
    Vec3 accel;
    Vec3 mag;

    // If there is new data from the gyro
    if (imu.gyroReady()) {
        gyro = imu.readGyro();
        sdLog.logGyroData(gyro);
    }

    // If there is new data from the accelerometer
    if (imu.accelReady()) {
        accel = imu.readAccel();
        sdLog.logAccelData(accel);
    }

    // If there is new data from the magnetometer
    if (imu.magReady()) {
        mag = imu.readMag();
        sdLog.logMagData(mag);
    }

    // Write one combined row using the latest available values.
    // If a sensor didn't update this loop, its last known value is reused.
    sdLog.writeCombinedRow();
}

