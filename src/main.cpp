/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>
#include "imu.hpp"

IMU imu;

void setup() {
    Serial.begin(115200);
    delay(500);

    imu.init(IMU::defaultConfig());

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
    imu.update();

    if (imu.gyroReady()) {
        Vec3 gyro = imu.readGyro();
        Serial.print("Gyro (deg/s): [");
        Serial.print(gyro.x); Serial.print(",");
        Serial.print(gyro.y); Serial.print(",");
        Serial.print(gyro.z); Serial.println("]");
    }

    if (imu.accelReady()) {
        Vec3 accel = imu.readAccel();
        Serial.print("Accel (g): [");
        Serial.print(accel.x); Serial.print(",");
        Serial.print(accel.y); Serial.print(",");
        Serial.print(accel.z); Serial.println("]");
    }

    if (imu.magReady()) {
        Vec3 mag = imu.readMag();
        Serial.print("Mag (uT): [");
        Serial.print(mag.x); Serial.print(",");
        Serial.print(mag.y); Serial.print(",");
        Serial.print(mag.z); Serial.println("]");
    }

    if (imu.quatReady()) {
        Quaternion quat = imu.readQuat();
        Serial.print("Quat: [");
        Serial.print(quat.w); Serial.print(",");
        Serial.print(quat.x); Serial.print(",");
        Serial.print(quat.y); Serial.print(",");
        Serial.print(quat.z); Serial.println("]");
    }
}
