/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 *
 * Preset selection — change these two lines to switch sensor configs:
 *   IMU::flightConfig()     — ±8g, 23.9 Hz LPF, 100 Hz ODR (use for actual flights)
 *   IMU::lowNoiseConfig()   — ±4g, 11.5 Hz LPF, 50 Hz ODR  (use for ground testing)
 *   Barometer::flightConfig()   — 8x OSR, IIR 15, 50 Hz, normal mode
 *   Barometer::highRateConfig() — 4x OSR, IIR 3, 200 Hz, normal mode
 */

#include <Arduino.h>
#include "imu.hpp"
#include "barometer.hpp"
#include "sd_log_file.hpp"
#include "state_machine.hpp"
#include "airbrake_motor.hpp"

IMU          imu;
Barometer    barometer;
sd_log       sdLog;
StateMachine stateMachine(sdLog);
AirbrakeMotor airbrakeMotor(3, 4, 5); //slpPin, stepPin, dirPin

void setup() {
    Serial.begin(115200);
    airbrakeMotor.begin();
    delay(500);

    // ── Sensor init ─────────────────────────────────────────────────────────
    // Change the preset here to switch sensor configurations.
    if (!imu.init(IMU::flightConfig())) {
        Serial.println("ICM-20948 init failed!");
    }

    if (!barometer.init(Barometer::flightConfig())) {
        Serial.println("BMP388 init failed!");
    }

    if (!sdLog.init()) {
        Serial.println("SD card init failed!");
    }

    Serial.println("GNC-Airbrakes firmware initialized");
    Serial.println("[STATE] Starting in ON_PAD — buffering pre-launch samples.");
}

void loop() {
    // ── Read sensors ────────────────────────────────────────────────────────
    bool imuReady  = imu.update();
    bool baroReady = barometer.update();

    // ── Update state machine ────────────────────────────────────────────────
    // Always update with latest data even if sensors had no new reading this tick.
    stateMachine.update(imu.readAll(), barometer.readAll());
    airbrakeMotor.update(stateMachine.getAirbrakeStatus());

    // ── SD logging ──────────────────────────────────────────────────────────
    // Only log during flight states (BOOST through RECOVERY).
    // ON_PAD data is buffered in RAM and flushed to SD when launch is detected.
    if (stateMachine.isLogging() && (imuReady || baroReady)) {
        sdLog.log(imu.readAll(), barometer.readAll());
    }
}

// TEST CODE FOR STEPPER MOTOR CONTROL — IGNORE
// // defines pins numbers
// const int stepPin = 3; 
// const int dirPin = 4; 
 
// void setup() {
//   // Sets the two pins as Outputs
//   pinMode(stepPin,OUTPUT); 
//   pinMode(dirPin,OUTPUT);
// }
// void loop() {
//   digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
//   // Makes 200 pulses for making one full cycle rotation
//   for(int x = 0; x < 200; x++) {
//     digitalWrite(stepPin,HIGH); 
//     delayMicroseconds(500); 
//     digitalWrite(stepPin,LOW); 
//     delayMicroseconds(500); 
//   }
//   delay(1000); // One second delay
  
//   digitalWrite(dirPin,LOW); //Changes the rotations direction
//   // Makes 400 pulses for making two full cycle rotation
//   for(int x = 0; x < 400; x++) {
//     digitalWrite(stepPin,HIGH);
//     delayMicroseconds(500);
//     digitalWrite(stepPin,LOW);
//     delayMicroseconds(500);
//   }
//   delay(1000);
// }