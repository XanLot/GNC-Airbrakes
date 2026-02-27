# State Machine & Sensor Config Design
**Date:** 2026-02-26
**Branch:** state_machine
**Authors:** Jack, Xander, Claude

---

## Overview

This document covers two things:
1. A robust flight state machine for the airbrake firmware running on Teensy 4.1
2. Named sensor configuration presets for the ICM-20948 IMU and BMP388 barometer, based on their datasheets

---

## 1. Flight State Machine

### States

| State | Airbrake Status | Transition Condition |
|-------|----------------|---------------------|
| ON_PAD | LOCKED | Boot → automatic |
| BOOST | LOCKED | `‖accel‖ >= 5g` |
| COAST_ONSET | PERMITTED | `‖accel‖ <= 0g` (motor burnout) |
| COAST | ACTIVE_CONT | timer elapsed (default 3s, configurable) |
| RECOVERY | LOCKED | altitude starts decreasing |

**LOCKED** — airbrake actuator held closed, no movement allowed
**PERMITTED** — airbrake may be extended but closed-loop control is not yet running
**ACTIVE_CONT** — closed-loop GNC algorithm is running and actively commanding the airbrake

### Architecture: StateMachine Class

A `StateMachine` class holds the current `FlightState` enum. The public interface is:

```cpp
void update(const IMUData& imu, const BarometerData& baro);
FlightState getState() const;
bool isLogging() const;  // true during BOOST, COAST_ONSET, COAST, RECOVERY
```

Internally, each state has its own `onEnter_*()` and `checkTransition_*()` private methods.
This avoids a single giant switch/case and keeps each state self-contained and readable.

`onEnter_*()` is called exactly once when entering a state — this is where the airbrake
stub is commanded and (for BOOST entry) the pre-launch buffer is flushed to SD.

`checkTransition_*()` is called every loop tick — it checks sensor data and fires a
transition if the condition is met.

### Airbrake Stub

```cpp
enum class AirbrakeStatus { LOCKED, PERMITTED, ACTIVE_CONT };
void setAirbrakeStatus(AirbrakeStatus status);
```

The function body is a stub with a comment marking where servo/actuator code goes.
The state machine calls this in each `onEnter_*()`, so when hardware is added it only
needs to be wired in one place.

### Pre-Launch Buffer

During ON_PAD, the state machine maintains a circular buffer of the last 100 sensor
readings in RAM. No SD writes happen on the pad.

When the ON_PAD → BOOST transition fires, `onEnter_Boost()` flushes the buffer to SD
first, then normal flight logging begins. This gives ~1 second of quiet pad data at the
top of every log file — useful for:
- Ground pressure baseline (used for AGL altitude calculation)
- Gyro bias reference
- Confirming sensors are healthy before liftoff

### SD Logging

Logging is active during: BOOST, COAST_ONSET, COAST, RECOVERY
Logging is inactive during: ON_PAD (buffer only, no SD writes)

State transitions are also logged to Serial for debugging.

### Configurable Constants

```cpp
constexpr float BOOST_ACCEL_THRESHOLD_G   = 5.0f;   // g
constexpr float BURNOUT_ACCEL_THRESHOLD_G = 0.0f;   // g
constexpr float COAST_TIMER_SECONDS       = 3.0f;   // seconds
constexpr int   PRE_LAUNCH_BUFFER_SIZE    = 100;    // samples (~1 second at 100 Hz)
```

---

## 2. Sensor Configuration Presets

### Design

Presets are additional static methods on the existing `IMUConfig` and `BarometerConfig`
structs — consistent with the existing `defaultConfig()` pattern. Selected at compile time
in `main.cpp`:

```cpp
imu.init(IMU::flightConfig());
barometer.init(Barometer::flightConfig());
```

### New IMUConfig Fields

```cpp
uint8_t accel_dlpf_cfg;        // 0-7: see table below
uint8_t gyro_dlpf_cfg;         // 0-7: see table below
uint16_t accel_sample_rate_div; // ODR = 1125 / (1 + div)
uint16_t gyro_sample_rate_div;  // ODR = 1100 / (1 + div)
```

DLPF bandwidth options (ICM-20948):
| cfg value | Bandwidth |
|-----------|-----------|
| 0 | 246 Hz |
| 2 | 111 Hz |
| 3 | 50 Hz |
| 4 | 23.9 Hz ← flight recommendation |
| 5 | 11.5 Hz |
| 6 | 5.7 Hz |

### IMU Presets

| Preset | Accel Range | DLPF BW | ODR | Notes |
|--------|-------------|---------|-----|-------|
| `defaultConfig()` | ±16g | off | ~100 Hz | existing, unchanged |
| `flightConfig()` | **±8g** | **23.9 Hz** | **100 Hz** | 6.3g peak fits with margin; LPF cuts vibration aliasing |
| `lowNoiseConfig()` | ±4g | 11.5 Hz | 50 Hz | ground testing at low-g, max resolution |

Why ±8g not ±16g for flight: 6.3g peak fits in ±8g with margin, and halves
quantization noise compared to ±16g. ±16g would only be needed if the motor burns
harder than expected.

Why 23.9 Hz LPF: rocket motor vibrations are high-frequency. 23.9 Hz filters those
out without lagging real motion events — airbrakes move slowly enough that 23.9 Hz
bandwidth is more than sufficient for GNC decisions.

### New BarometerConfig Fields

```cpp
uint8_t power_mode;  // 0=forced (blocking), 1=normal (continuous)
```

### Barometer Presets

| Preset | Pressure OSR | Temp OSR | IIR Coeff | ODR | Mode |
|--------|-------------|----------|-----------|-----|------|
| `defaultConfig()` | 4x | 8x | 3 | 50 Hz | forced |
| `flightConfig()` | **8x** | **2x** | **31** | **50 Hz** | **normal** |
| `highRateConfig()` | 4x | 1x | 7 | 200 Hz | normal |

Why IIR coeff 31 for flight: turbulent airflow during ascent creates real pressure
fluctuations at the sensor, not just electrical noise. Coeff 31 smooths over ~32
samples, which at 50 Hz = 640 ms. This is appropriate since altitude changes on a
rocket are gradual compared to pressure turbulence spikes.

Why normal mode: replaces the current blocking `performReading()` call. In normal mode
the BMP388 samples continuously and the Teensy reads whenever new data is ready — the
main loop never stalls waiting for a pressure reading.

---

## 3. Calibration (Future Work)

Calibration is not implemented yet. When ready, the approach is:

**Accelerometer:** Hold rocket perfectly vertical. Collect N samples. The mean should
read (0, 0, 9.81 m/s²). Subtract the difference from expected to get per-axis offsets.

**Gyroscope:** Leave rocket completely still. Average N samples. Any non-zero reading
is bias — subtract it from all future readings.

**Barometer:** Average pressure readings at a known altitude to get a ground reference.
This reference is then used for AGL (above ground level) altitude calculation.

The implementation hook point will be a `Calibration` struct passed into `init()`,
applied automatically inside `readAccel()`, `readGyro()`, and `readAll()`.

---

## 4. Files Modified / Created

```
src/state_machine.hpp       ← new
src/state_machine.cpp       ← new
src/imu.hpp                 ← add new config fields + presets
src/imu.cpp                 ← apply new config in init()
src/barometer.hpp           ← add power_mode + presets
src/barometer.cpp           ← normal mode support
src/main.cpp                ← integrate state machine + conditional logging
docs/gnc-sensor-primer.md   ← new educational doc
docs/plans/2026-02-26-state-machine-design.md  ← this file
```

---

## 5. Branch Management

- Work on: `origin/state_machine` (Zander's branch, checked out locally)
- Delete: local `state` branch (created by mistake)
