#ifndef DIAGNOSTIC_MODE

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "sensor_data.hpp"
#include "pins.hpp"
#include "imu.hpp"
#include "barometer.hpp"
#include "magnetometer.hpp"
#include "temp_sensor.hpp"
#include "sd_log_file.hpp"
#include "state_machine.hpp"

#ifdef DEBUG_MODE
#include "debug_mode.hpp"
#endif

IMU          imu1, imu2, imu3, imu4;
Barometer    baro1, baro2;
Magnetometer mag;
TempSensor   tmp1, tmp2;
sd_log       sdLog;
StateMachine stateMachine(sdLog);

uint16_t sensorStatus = 0;
constexpr uint16_t SENSOR_IMU1  = (1 << 0);
constexpr uint16_t SENSOR_IMU2  = (1 << 1);
constexpr uint16_t SENSOR_IMU3  = (1 << 2);
constexpr uint16_t SENSOR_IMU4  = (1 << 3);
constexpr uint16_t SENSOR_BARO1 = (1 << 4);
constexpr uint16_t SENSOR_BARO2 = (1 << 5);
constexpr uint16_t SENSOR_MAG   = (1 << 6);
constexpr uint16_t SENSOR_TMP1  = (1 << 7);
constexpr uint16_t SENSOR_TMP2  = (1 << 8);

void setup() {
    Serial.begin(115200);
    delay(500);

    SPI.begin();
    SPI1.setMISO(39);  // PCB routes SPI1 MISO to pin 39, not the default pin 1
    SPI1.begin();
    Wire.begin();
    Wire.setClock(400000);

//chose the right config functions based on debug vs flight mode
#ifdef DEBUG_MODE
    using ImuCfgFn  = IMUConfig(*)(uint8_t, SPIClass*);
    using BaroCfgFn = BarometerConfig(*)(uint8_t, SPIClass*);
    ImuCfgFn  imuCfg  = IMU::debugConfig;
    BaroCfgFn baroCfg = Barometer::debugConfig;
#else
    using ImuCfgFn  = IMUConfig(*)(uint8_t, SPIClass*);
    using BaroCfgFn = BarometerConfig(*)(uint8_t, SPIClass*);
    ImuCfgFn  imuCfg  = IMU::flightConfig;
    BaroCfgFn baroCfg = Barometer::flightConfig;
#endif

    if (imu1.init(imuCfg(IMU1_CS, &SPI))) {
        sensorStatus |= SENSOR_IMU1;
    } else {
        Serial.println("IMU1 init failed");
    }

    if (imu2.init(imuCfg(IMU2_CS, &SPI))) {
        sensorStatus |= SENSOR_IMU2;
    } else {
        Serial.println("IMU2 init failed");
    }

    if (imu3.init(imuCfg(IMU3_CS, &SPI1))) {
        sensorStatus |= SENSOR_IMU3;
    } else {
        Serial.println("IMU3 init failed");
    }

    if (imu4.init(imuCfg(IMU4_CS, &SPI1))) {
        sensorStatus |= SENSOR_IMU4;
    } else {
        Serial.println("IMU4 init failed");
    }

    if (baro1.init(baroCfg(BARO1_CS, &SPI))) {
        sensorStatus |= SENSOR_BARO1;
    } else {
        Serial.println("Baro1 init failed");
    }

    if (baro2.init(baroCfg(BARO2_CS, &SPI1))) {
        sensorStatus |= SENSOR_BARO2;
    } else {
        Serial.println("Baro2 init failed");
    }

    if (mag.init()) {
        sensorStatus |= SENSOR_MAG;
    } else {
        Serial.println("Magnetometer init failed");
    }

    if (tmp1.init(TEMP1_I2C_ADDR)) {
        sensorStatus |= SENSOR_TMP1;
    } else {
        Serial.println("TMP117 (0x48) init failed");
    }

    if (tmp2.init(TEMP2_I2C_ADDR)) {
        sensorStatus |= SENSOR_TMP2;
    } else {
        Serial.println("TMP117 (0x49) init failed");
    }

    if (!sdLog.init()) Serial.println("SD card init failed");

    Serial.print("Sensor status: 0x");
    Serial.println(sensorStatus, HEX);
    Serial.println("GNC-Airbrakes firmware initialized");
}

static SensorData readAllSensors() {
    SensorData data{};
    data.timestamp_us = micros();

    if (sensorStatus & SENSOR_IMU1) {
        imu1.update();
        data.imu[0] = imu1.readAll();
    } else {
        data.imu[0] = nanIMU();
    }

    if (sensorStatus & SENSOR_IMU2) {
        imu2.update();
        data.imu[1] = imu2.readAll();
    } else {
        data.imu[1] = nanIMU();
    }

    if (sensorStatus & SENSOR_IMU3) {
        imu3.update();
        data.imu[2] = imu3.readAll();
    } else {
        data.imu[2] = nanIMU();
    }

    if (sensorStatus & SENSOR_IMU4) {
        imu4.update();
        data.imu[3] = imu4.readAll();
    } else {
        data.imu[3] = nanIMU();
    }

    if (sensorStatus & SENSOR_BARO1) {
        baro1.update();
        data.baro[0] = baro1.readAll();
    } else {
        data.baro[0] = nanBaro();
    }

    if (sensorStatus & SENSOR_BARO2) {
        baro2.update();
        data.baro[1] = baro2.readAll();
    } else {
        data.baro[1] = nanBaro();
    }

    if (sensorStatus & SENSOR_MAG) {
        mag.update();
        data.mag = mag.readAll();
    } else {
        data.mag = nanMag();
    }

    if (sensorStatus & SENSOR_TMP1) {
        tmp1.update();
        data.tmp[0] = tmp1.readAll();
    } else {
        data.tmp[0] = nanTemp();
    }

    if (sensorStatus & SENSOR_TMP2) {
        tmp2.update();
        data.tmp[1] = tmp2.readAll();
    } else {
        data.tmp[1] = nanTemp();
    }

    return data;
}

void loop() {
    SensorData data = readAllSensors();

#ifdef DEBUG_MODE
    debugPrint(data);
    delay(4);
#else
    stateMachine.update(data);
    if (stateMachine.isLogging()) {
        sdLog.log(data);
    }
#endif
}

#endif // DIAGNOSTIC_MODE
