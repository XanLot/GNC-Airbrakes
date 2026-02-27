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

IMU          imu;
Barometer    barometer;
sd_log       sdLog;
StateMachine stateMachine(sdLog);

void setup() {
    Serial.begin(115200);
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

    // ── SD logging ──────────────────────────────────────────────────────────
    // Only log during flight states (BOOST through RECOVERY).
    // ON_PAD data is buffered in RAM and flushed to SD when launch is detected.
    if (stateMachine.isLogging() && (imuReady || baroReady)) {
        sdLog.log(imu.readAll(), barometer.readAll());
    }
}
