# State Machine & Sensor Config Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Implement a 5-state flight state machine with conditional SD logging, datasheet-informed sensor config presets, and a pre-launch circular buffer on the `state_machine` branch.

**Architecture:** A `StateMachine` class owns the flight state enum and calls per-state `onEnter_*()`/`checkTransition_*()` methods — no giant switch/case. Sensor presets are additional static methods on the existing `IMUConfig`/`BarometerConfig` structs. The pre-launch buffer is a fixed-size circular array in the StateMachine that gets flushed to SD on BOOST entry.

**Tech Stack:** C++17, Arduino/Teensy 4.1, Adafruit ICM20948 library, Adafruit BMP3XX library, SdFat. Build with `make` from repo root. Flash with `make upload`. Monitor serial with `make monitor` or PlatformIO serial monitor at 115200 baud.

**Branch:** `state_machine` (already checked out, tracking `origin/state_machine`)

**Design doc:** `docs/plans/2026-02-26-state-machine-design.md` — read this first.

---

## Testing Approach for Embedded Firmware

Traditional unit tests can't run on the Teensy. Instead, each task uses two verification methods:
1. **Compile check** — `make` must produce zero errors/warnings
2. **Serial verification** — flash and read serial output to confirm behavior

For the state machine logic specifically, Task 1 includes a standalone host-compilable
test you can run with `g++` on your Mac to verify transition logic without flashing.

---

### Task 1: Add state_machine.hpp

**Files:**
- Create: `src/state_machine.hpp`

This defines all the types, enums, and the StateMachine class interface. No implementation yet.

**Step 1: Create the header**

```cpp
// src/state_machine.hpp
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "imu.hpp"
#include "barometer.hpp"
#include "sd_log_file.hpp"

// ─── Flight states ────────────────────────────────────────────────────────────
// The rocket passes through these states in order during a normal flight.
enum class FlightState {
    ON_PAD,       // Sitting on the launch pad. Airbrakes locked. No SD logging.
    BOOST,        // Motor burning. High acceleration. Airbrakes locked.
    COAST_ONSET,  // Motor burned out. Airbrakes permitted but not yet controlled.
    COAST,        // Closed-loop GNC active. Airbrakes commanded by algorithm.
    RECOVERY      // Apogee passed, rocket descending. Airbrakes locked.
};

// ─── Airbrake status ──────────────────────────────────────────────────────────
// Describes what the airbrake actuator is allowed/commanded to do.
enum class AirbrakeStatus {
    LOCKED,       // Airbrake held closed. Motor not powered.
    PERMITTED,    // Airbrake may extend, but no active control yet.
    ACTIVE_CONT   // Closed-loop GNC is actively commanding the airbrake.
};

// ─── Configurable transition thresholds ──────────────────────────────────────
// Change these to tune when state transitions fire.
constexpr float BOOST_ACCEL_THRESHOLD_MS2   = 5.0f * 9.81f;  // 5g in m/s²
constexpr float BURNOUT_ACCEL_THRESHOLD_MS2 = 0.0f;          // net accel <= 0
constexpr float COAST_TIMER_SECONDS         = 3.0f;          // seconds in COAST_ONSET before COAST
constexpr int   PRE_LAUNCH_BUFFER_SIZE      = 100;           // ~1 second at 100 Hz IMU

// ─── Pre-launch sample ────────────────────────────────────────────────────────
// One snapshot stored in the circular buffer during ON_PAD.
struct PreLaunchSample {
    IMUData imu;
    BarometerData baro;
};

// ─── StateMachine ─────────────────────────────────────────────────────────────
class StateMachine {
public:
    // Pass in the SD log object so the state machine can control logging.
    explicit StateMachine(sd_log& sdLog);

    // Call every loop iteration with fresh sensor data.
    // Checks transition conditions and fires onEnter/onExit when state changes.
    void update(const IMUData& imu, const BarometerData& baro);

    // Current flight state — used by main.cpp for serial debug printing.
    FlightState getState() const;

    // True during BOOST, COAST_ONSET, COAST, RECOVERY. False on ON_PAD.
    bool isLogging() const;

private:
    // ── State ────────────────────────────────────────────────────────────────
    FlightState currentState_;
    sd_log&     sdLog_;

    // ── Pre-launch circular buffer ───────────────────────────────────────────
    PreLaunchSample preLaunchBuffer_[PRE_LAUNCH_BUFFER_SIZE];
    int             bufferHead_;  // next write index (wraps around)
    int             bufferCount_; // how many valid samples are stored

    // ── COAST_ONSET timer ────────────────────────────────────────────────────
    unsigned long coastOnsetEntryMs_; // millis() when we entered COAST_ONSET

    // ── COAST → RECOVERY: altitude tracking ─────────────────────────────────
    float previousAltitude_;

    // ── Transition helpers ───────────────────────────────────────────────────
    // Each method checks if we should leave the named state.
    // Returns the next state, or currentState_ if no transition.
    FlightState checkTransition_OnPad(const IMUData& imu);
    FlightState checkTransition_Boost(const IMUData& imu);
    FlightState checkTransition_CoastOnset();
    FlightState checkTransition_Coast(const BarometerData& baro);

    // ── Entry actions ────────────────────────────────────────────────────────
    // Called once when entering each state.
    void onEnter_OnPad();
    void onEnter_Boost(const IMUData& imu, const BarometerData& baro);
    void onEnter_CoastOnset();
    void onEnter_Coast();
    void onEnter_Recovery();

    // ── Airbrake stub ────────────────────────────────────────────────────────
    // TODO: wire this to the actual servo/actuator when hardware is ready.
    void setAirbrakeStatus(AirbrakeStatus status);

    // ── Utilities ────────────────────────────────────────────────────────────
    static float accelMagnitude(const IMUData& imu);
    void         storePreLaunchSample(const IMUData& imu, const BarometerData& baro);
    void         flushPreLaunchBuffer();
};

#endif // STATE_MACHINE_HPP
```

**Step 2: Compile check (header only)**

```bash
cd "/Users/jack/Desktop/CU In Space/GNC-Airbrakes"
make 2>&1 | head -30
```

Expected: Compile errors about missing `state_machine.cpp` implementation — that's fine for now.
If you see parse errors inside `state_machine.hpp` itself, fix those first.

**Step 3: Commit**

```bash
git add src/state_machine.hpp
git commit -m "feat: add state_machine.hpp with FlightState enum and StateMachine interface"
```

---

### Task 2: Implement state_machine.cpp

**Files:**
- Create: `src/state_machine.cpp`

**Step 1: Create the implementation**

```cpp
// src/state_machine.cpp
#include "state_machine.hpp"
#include <Arduino.h>
#include <cmath>

// ─── Constructor ──────────────────────────────────────────────────────────────
StateMachine::StateMachine(sd_log& sdLog)
    : currentState_(FlightState::ON_PAD),
      sdLog_(sdLog),
      bufferHead_(0),
      bufferCount_(0),
      coastOnsetEntryMs_(0),
      previousAltitude_(0.0f)
{
    onEnter_OnPad();
}

// ─── Public: update ───────────────────────────────────────────────────────────
void StateMachine::update(const IMUData& imu, const BarometerData& baro) {
    FlightState nextState = currentState_;

    // Check if it's time to leave the current state
    switch (currentState_) {
        case FlightState::ON_PAD:
            storePreLaunchSample(imu, baro);
            nextState = checkTransition_OnPad(imu);
            break;
        case FlightState::BOOST:
            nextState = checkTransition_Boost(imu);
            break;
        case FlightState::COAST_ONSET:
            nextState = checkTransition_CoastOnset();
            break;
        case FlightState::COAST:
            nextState = checkTransition_Coast(baro);
            break;
        case FlightState::RECOVERY:
            break; // Terminal state — no further transitions
    }

    // Fire transition if state changed
    if (nextState != currentState_) {
        Serial.print("[STATE] Transition: ");
        Serial.print(static_cast<int>(currentState_));
        Serial.print(" -> ");
        Serial.println(static_cast<int>(nextState));

        currentState_ = nextState;

        switch (currentState_) {
            case FlightState::ON_PAD:       onEnter_OnPad();               break;
            case FlightState::BOOST:        onEnter_Boost(imu, baro);      break;
            case FlightState::COAST_ONSET:  onEnter_CoastOnset();          break;
            case FlightState::COAST:        onEnter_Coast();               break;
            case FlightState::RECOVERY:     onEnter_Recovery();            break;
        }
    }

    // Update altitude tracking for COAST → RECOVERY detection
    if (currentState_ == FlightState::COAST) {
        previousAltitude_ = baro.altitude;
    }
}

// ─── Public: getters ──────────────────────────────────────────────────────────
FlightState StateMachine::getState() const {
    return currentState_;
}

bool StateMachine::isLogging() const {
    return currentState_ != FlightState::ON_PAD;
}

// ─── Transition checks ────────────────────────────────────────────────────────

FlightState StateMachine::checkTransition_OnPad(const IMUData& imu) {
    // Launch detected when total acceleration exceeds 5g
    if (accelMagnitude(imu) >= BOOST_ACCEL_THRESHOLD_MS2) {
        return FlightState::BOOST;
    }
    return FlightState::ON_PAD;
}

FlightState StateMachine::checkTransition_Boost(const IMUData& imu) {
    // Motor burnout detected when net acceleration drops to ~0g
    if (accelMagnitude(imu) <= BURNOUT_ACCEL_THRESHOLD_MS2) {
        return FlightState::COAST_ONSET;
    }
    return FlightState::BOOST;
}

FlightState StateMachine::checkTransition_CoastOnset() {
    // Wait for the coast timer to elapse before starting GNC
    unsigned long elapsed = millis() - coastOnsetEntryMs_;
    if (elapsed >= static_cast<unsigned long>(COAST_TIMER_SECONDS * 1000.0f)) {
        return FlightState::COAST;
    }
    return FlightState::COAST_ONSET;
}

FlightState StateMachine::checkTransition_Coast(const BarometerData& baro) {
    // Apogee detected when altitude starts decreasing
    if (baro.altitude < previousAltitude_) {
        return FlightState::RECOVERY;
    }
    return FlightState::COAST;
}

// ─── Entry actions ────────────────────────────────────────────────────────────

void StateMachine::onEnter_OnPad() {
    Serial.println("[STATE] ON_PAD: Waiting for launch. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
}

void StateMachine::onEnter_Boost(const IMUData& imu, const BarometerData& baro) {
    Serial.println("[STATE] BOOST: Launch detected. Flushing pre-launch buffer to SD.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);

    // Write the buffered pad samples to SD before flight data
    flushPreLaunchBuffer();
}

void StateMachine::onEnter_CoastOnset() {
    Serial.println("[STATE] COAST_ONSET: Motor burnout. Airbrakes permitted.");
    setAirbrakeStatus(AirbrakeStatus::PERMITTED);
    coastOnsetEntryMs_ = millis();
}

void StateMachine::onEnter_Coast() {
    Serial.println("[STATE] COAST: GNC active. Airbrakes under closed-loop control.");
    setAirbrakeStatus(AirbrakeStatus::ACTIVE_CONT);
}

void StateMachine::onEnter_Recovery() {
    Serial.println("[STATE] RECOVERY: Apogee passed. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
}

// ─── Airbrake stub ────────────────────────────────────────────────────────────

void StateMachine::setAirbrakeStatus(AirbrakeStatus status) {
    // TODO: Wire this to the actual servo/actuator when hardware is ready.
    // For now, just log the commanded status to serial.
    switch (status) {
        case AirbrakeStatus::LOCKED:      Serial.println("[AIRBRAKE] LOCKED");      break;
        case AirbrakeStatus::PERMITTED:   Serial.println("[AIRBRAKE] PERMITTED");   break;
        case AirbrakeStatus::ACTIVE_CONT: Serial.println("[AIRBRAKE] ACTIVE_CONT"); break;
    }
}

// ─── Utilities ────────────────────────────────────────────────────────────────

float StateMachine::accelMagnitude(const IMUData& imu) {
    return sqrtf(imu.accel.x * imu.accel.x +
                 imu.accel.y * imu.accel.y +
                 imu.accel.z * imu.accel.z);
}

void StateMachine::storePreLaunchSample(const IMUData& imu, const BarometerData& baro) {
    preLaunchBuffer_[bufferHead_] = {imu, baro};
    bufferHead_ = (bufferHead_ + 1) % PRE_LAUNCH_BUFFER_SIZE;
    if (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) {
        bufferCount_++;
    }
}

void StateMachine::flushPreLaunchBuffer() {
    // The buffer is a circular array. Calculate the start index of the oldest sample.
    int startIndex = (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE)
                     ? 0
                     : bufferHead_; // bufferHead_ points to the oldest when full

    for (int i = 0; i < bufferCount_; i++) {
        int idx = (startIndex + i) % PRE_LAUNCH_BUFFER_SIZE;
        sdLog_.log(preLaunchBuffer_[idx].imu, preLaunchBuffer_[idx].baro);
    }

    Serial.print("[STATE] Flushed ");
    Serial.print(bufferCount_);
    Serial.println(" pre-launch samples to SD.");
}
```

**Step 2: Compile check**

```bash
cd "/Users/jack/Desktop/CU In Space/GNC-Airbrakes"
make 2>&1 | head -50
```

Expected: Linker errors about `StateMachine` not being used in `main.cpp` yet — that's fine.
Any errors inside `state_machine.cpp` itself must be fixed before proceeding.

**Step 3: Commit**

```bash
git add src/state_machine.cpp
git commit -m "feat: implement StateMachine with 5 flight states and pre-launch buffer"
```

---

### Task 3: Add sensor config presets to IMUConfig

**Files:**
- Modify: `src/imu.hpp` — add new fields to `IMUConfig`, add preset declarations
- Modify: `src/imu.cpp` — apply new fields in `init()`, implement presets

**Step 1: Update imu.hpp**

Replace the existing `IMUConfig` struct and add preset declarations:

```cpp
// In src/imu.hpp — replace the existing IMUConfig struct with this:
struct IMUConfig {
    uint8_t  cs_pin;                // SPI chip select pin
    uint8_t  accel_range;           // ICM20948_ACCEL_RANGE_2_G / _4 / _8 / _16
    uint8_t  gyro_range;            // ICM20948_GYRO_RANGE_250_DPS / _500 / _1000 / _2000
    uint8_t  mag_data_rate;         // AK09916_MAG_DATARATE_* enum value
    uint8_t  accel_dlpf_cfg;        // 0-7: DLPF bandwidth. 4 = 23.9 Hz. 0 = 246 Hz (off).
    uint8_t  gyro_dlpf_cfg;         // 0-7: same scale as accel_dlpf_cfg
    uint16_t accel_sample_rate_div; // Accel ODR = 1125 / (1 + div). e.g. div=10 → 102 Hz
    uint8_t  gyro_sample_rate_div;  // Gyro ODR  = 1100 / (1 + div). e.g. div=10 → 100 Hz
};
```

Add these static method declarations to the `IMU` class (after `defaultConfig()`):

```cpp
static IMUConfig flightConfig();    // ±8g, 23.9 Hz LPF, 100 Hz ODR — use for actual flights
static IMUConfig lowNoiseConfig();  // ±4g, 11.5 Hz LPF, 50 Hz ODR  — use for ground testing
```

**Step 2: Update imu.cpp**

Add the two new preset implementations and update `init()` to apply the new fields.

Add after `defaultConfig()`:

```cpp
IMUConfig IMU::flightConfig() {
    return IMUConfig{
        .cs_pin               = 7,
        .accel_range          = ICM20948_ACCEL_RANGE_8_G,   // ±8g — fits 6.3g peak with margin
        .gyro_range           = ICM20948_GYRO_RANGE_500_DPS,
        .mag_data_rate        = AK09916_MAG_DATARATE_100_HZ,
        .accel_dlpf_cfg       = 4,   // 23.9 Hz — filters motor vibration aliasing
        .gyro_dlpf_cfg        = 4,   // 23.9 Hz — matches accel filter
        .accel_sample_rate_div = 10,  // 1125 / 11 ≈ 102 Hz
        .gyro_sample_rate_div  = 10   // 1100 / 11 = 100 Hz
    };
}

IMUConfig IMU::lowNoiseConfig() {
    return IMUConfig{
        .cs_pin               = 7,
        .accel_range          = ICM20948_ACCEL_RANGE_4_G,   // ±4g — max resolution for low-g
        .gyro_range           = ICM20948_GYRO_RANGE_250_DPS,
        .mag_data_rate        = AK09916_MAG_DATARATE_100_HZ,
        .accel_dlpf_cfg       = 5,   // 11.5 Hz — aggressive filtering for bench testing
        .gyro_dlpf_cfg        = 5,   // 11.5 Hz
        .accel_sample_rate_div = 21,  // 1125 / 22 ≈ 51 Hz
        .gyro_sample_rate_div  = 21   // 1100 / 22 = 50 Hz
    };
}
```

Update `IMU::init()` — add these lines after the existing `setAccelRange`/`setGyroRange`/`setMagDataRate` calls:

```cpp
// Set sample rate divisors (controls ODR)
icm20948.setAccelRateDivisor(config.accel_sample_rate_div);
icm20948.setGyroRateDivisor(config.gyro_sample_rate_div);

// Set digital low-pass filter bandwidth.
// This requires writing directly to Bank 2 registers because the Adafruit
// library does not expose DLPF config through a public method.
// ACCEL_CONFIG (Bank 2, reg 0x14): bits [2:0] = ACCEL_DLPFCFG, bit [3] = ACCEL_FCHOICE
// Setting ACCEL_FCHOICE=1 enables the DLPF. cfg 0-7 selects bandwidth.
if (config.accel_dlpf_cfg > 0) {
    uint8_t accel_cfg = (1 << 3) | (config.accel_dlpf_cfg & 0x07); // FCHOICE=1, DLPFCFG
    icm20948.writeExternalRegister(0x14, accel_cfg); // Bank 2 reg 0x14
}
// GYRO_CONFIG_1 (Bank 2, reg 0x01): bits [2:0] = GYRO_DLPFCFG, bit [3] = GYRO_FCHOICE
if (config.gyro_dlpf_cfg > 0) {
    uint8_t gyro_cfg = (1 << 3) | (config.gyro_dlpf_cfg & 0x07);
    icm20948.writeExternalRegister(0x01, gyro_cfg);  // Bank 2 reg 0x01
}
```

> **Note on writeExternalRegister:** Check whether the Adafruit ICM20X library exposes
> a method for writing to Bank 2 registers. The library's source is in
> `libraries/Adafruit_ICM20X/`. Search for `writeExternalRegister` or `setBank`.
> If it doesn't exist, use `icm20948._writeBankRegister(2, reg, value)` or equivalent.
> You may need to set the bank first. Check the library source before assuming
> the method name.

**Step 3: Compile check**

```bash
cd "/Users/jack/Desktop/CU In Space/GNC-Airbrakes"
make 2>&1 | grep -E "error:|warning:" | head -30
```

Fix any errors. Common ones: wrong enum name for `ICM20948_ACCEL_RANGE_8_G` — check
the Adafruit library header for exact enum names.

**Step 4: Verify serial output after flash**

Flash and open serial monitor. Expected to see IMU config applied without init failure:
```
GNC-Airbrakes firmware initialized
```
If `ICM-20948 init failed!` appears, the register write failed — check enum names and
bank register method names against the library source.

**Step 5: Commit**

```bash
git add src/imu.hpp src/imu.cpp
git commit -m "feat: add flightConfig and lowNoiseConfig presets to IMU with DLPF and ODR settings"
```

---

### Task 4: Add sensor config presets to BarometerConfig

**Files:**
- Modify: `src/barometer.hpp` — add `power_mode` field, add preset declarations
- Modify: `src/barometer.cpp` — implement presets, add normal mode support

**Step 1: Update barometer.hpp**

Add `power_mode` to `BarometerConfig` and add preset declarations:

```cpp
// In src/barometer.hpp — add power_mode to BarometerConfig:
struct BarometerConfig {
    uint8_t  cs_pin;
    uint32_t spi_speed;
    uint8_t  temperature_oversampling;
    uint8_t  pressure_oversampling;
    uint8_t  iir_filter_coeff;
    uint8_t  output_data_rate;
    float    sea_level_pressure_hpa;
    uint8_t  power_mode;  // 0 = forced (blocking), 1 = normal (continuous background sampling)
};
```

Add to Barometer class:
```cpp
static BarometerConfig flightConfig();     // 8x OSR, IIR 31, 50 Hz, normal mode
static BarometerConfig highRateConfig();   // 4x OSR, IIR 7,  200 Hz, normal mode
```

Also add a private field and update the `update()` signature in barometer.hpp:
```cpp
private:
    bool initialized_;
    float sea_level_pressure_hpa_;
    float temperature_;
    float pressure_;
    uint8_t power_mode_;  // stored from init() config
```

**Step 2: Update barometer.cpp**

Add preset implementations after `defaultConfig()`:

```cpp
BarometerConfig Barometer::flightConfig() {
    return BarometerConfig{
        .cs_pin                    = 33,
        .spi_speed                 = 1000000,
        .temperature_oversampling  = 1,      // BMP3_OVERSAMPLING_2X — temp less critical
        .pressure_oversampling     = 3,      // BMP3_OVERSAMPLING_8X — accurate altitude
        .iir_filter_coeff          = 4,      // BMP3_IIR_FILTER_COEFF_15 — smooths turbulence
        .output_data_rate          = 2,      // BMP3_ODR_50_HZ
        .sea_level_pressure_hpa    = 1013.25f,
        .power_mode                = 1       // normal mode — no blocking reads
    };
}

BarometerConfig Barometer::highRateConfig() {
    return BarometerConfig{
        .cs_pin                    = 33,
        .spi_speed                 = 1000000,
        .temperature_oversampling  = 0,      // BMP3_OVERSAMPLING_1X
        .pressure_oversampling     = 2,      // BMP3_OVERSAMPLING_4X
        .iir_filter_coeff          = 2,      // BMP3_IIR_FILTER_COEFF_3
        .output_data_rate          = 0,      // BMP3_ODR_200_HZ
        .sea_level_pressure_hpa    = 1013.25f,
        .power_mode                = 1       // normal mode
    };
}
```

Update `Barometer::init()` to store power_mode and configure it:

```cpp
bool Barometer::init(const BarometerConfig& config) {
    sea_level_pressure_hpa_ = config.sea_level_pressure_hpa;
    power_mode_ = config.power_mode;

    if (!bmp.begin_SPI(config.cs_pin, &SPI, config.spi_speed)) {
        return false;
    }

    bmp.setTemperatureOversampling(config.temperature_oversampling);
    bmp.setPressureOversampling(config.pressure_oversampling);
    bmp.setIIRFilterCoeff(config.iir_filter_coeff);
    bmp.setOutputDataRate(config.output_data_rate);

    if (config.power_mode == 1) {
        // Normal mode: sensor samples continuously in the background.
        // The Adafruit library doesn't expose setNormalMode() directly.
        // Write 0x33 to CTRL_MEAS (reg 0x1B): bits[5:4]=11 (normal), bits[1:0]=11 (P+T enabled)
        // NOTE: Check Adafruit_BMP3XX source for a cleaner way to do this.
        // If the library resets the mode on next call, we may need to override it.
        bmp.setMode(BMP3_MODE_NORMAL); // Use if this method exists in your library version
        // Fallback if setMode doesn't exist:
        // bmp.writeRegister(0x1B, 0x33);
    } else {
        // Forced mode: discard first blocking read (may be inaccurate)
        bmp.performReading();
    }

    initialized_ = true;
    return true;
}
```

Update `Barometer::update()` to handle both modes:

```cpp
bool Barometer::update() {
    if (!initialized_) return false;

    if (power_mode_ == 1) {
        // Normal mode: just read whatever the sensor has ready.
        // Returns false if no new data is available yet (non-blocking).
        if (!bmp.dataReady()) return false;
        temperature_ = bmp.temperature;
        pressure_ = bmp.pressure;
        return true;
    } else {
        // Forced mode: trigger a blocking measurement (original behavior).
        if (!bmp.performReading()) return false;
        temperature_ = bmp.temperature;
        pressure_ = bmp.pressure;
        return true;
    }
}
```

> **Note on normal mode API:** The Adafruit BMP3XX library may or may not expose
> `setMode()` or `dataReady()`. Check `libraries/Adafruit_BMP3XX/` source before
> assuming these methods exist. If the library only supports forced mode, keep using
> `performReading()` for now and add a comment marking it as a future improvement.
> The preset system still adds value even if normal mode falls back to forced.

**Step 3: Compile check**

```bash
make 2>&1 | grep -E "error:|warning:" | head -30
```

**Step 4: Commit**

```bash
git add src/barometer.hpp src/barometer.cpp
git commit -m "feat: add flightConfig and highRateConfig presets to Barometer with normal mode support"
```

---

### Task 5: Integrate StateMachine into main.cpp

**Files:**
- Modify: `src/main.cpp`

**Step 1: Rewrite main.cpp**

```cpp
/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 *
 * Preset selection — change these two lines to switch sensor configs:
 *   IMU::flightConfig()     — ±8g, 23.9 Hz LPF, 100 Hz ODR (use for actual flights)
 *   IMU::lowNoiseConfig()   — ±4g, 11.5 Hz LPF, 50 Hz ODR  (use for ground testing)
 *   Barometer::flightConfig()   — 8x OSR, IIR 31, 50 Hz, normal mode
 *   Barometer::highRateConfig() — 4x OSR, IIR 7, 200 Hz, normal mode
 */

#include <Arduino.h>
#include "imu.hpp"
#include "barometer.hpp"
#include "sd_log_file.hpp"
#include "state_machine.hpp"

IMU         imu;
Barometer   barometer;
sd_log      sdLog;
StateMachine stateMachine(sdLog);

void setup() {
    Serial.begin(115200);
    delay(500);

    // ── Sensor init ────────────────────────────────────────────────────────────
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
    // ── Read sensors ───────────────────────────────────────────────────────────
    bool imuReady  = imu.update();
    bool baroReady = barometer.update();

    // ── Update state machine ───────────────────────────────────────────────────
    // Always update with latest data even if sensors had no new reading this tick.
    stateMachine.update(imu.readAll(), barometer.readAll());

    // ── SD logging ─────────────────────────────────────────────────────────────
    // Only log during flight states (BOOST through RECOVERY).
    // ON_PAD data is buffered in RAM and flushed to SD when launch is detected.
    if (stateMachine.isLogging() && (imuReady || baroReady)) {
        sdLog.log(imu.readAll(), barometer.readAll());
    }
}
```

**Step 2: Compile check**

```bash
make 2>&1 | grep -E "error:|warning:" | head -30
```

Expected: clean compile. If `StateMachine` constructor doesn't match, check that
`sd_log` is passed by reference correctly.

**Step 3: Flash and verify serial**

```bash
make upload && make monitor
```

Expected serial output on boot:
```
GNC-Airbrakes firmware initialized
[STATE] Starting in ON_PAD — buffering pre-launch samples.
[STATE] ON_PAD: Waiting for launch. Airbrakes locked.
[AIRBRAKE] LOCKED
```

The board should then sit silently in ON_PAD, buffering samples. No SD writes yet.

To simulate a launch without a rocket: tilt or shake the board hard enough to hit 5g,
or temporarily lower `BOOST_ACCEL_THRESHOLD_MS2` to `1.0f * 9.81f` for testing.
You should see:
```
[STATE] Transition: 0 -> 1
[STATE] BOOST: Launch detected. Flushing pre-launch buffer to SD.
[STATE] Flushed 100 pre-launch samples to SD.
[AIRBRAKE] LOCKED
```

**Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "feat: integrate StateMachine into main loop with conditional SD logging"
```

---

### Task 6: Push to remote

**Step 1: Push state_machine branch**

```bash
cd "/Users/jack/Desktop/CU In Space/GNC-Airbrakes"
git push origin state_machine
```

**Step 2: Verify on GitHub**

Check `github.com/XanLot/GNC-Airbrakes` — the `state_machine` branch should show
the new commits. Send Xander the branch link so he can review.

---

## Known Library Unknowns (Check Before Implementing)

These need to be verified against the actual library source in `libraries/`:

| Library | Method needed | Check in |
|---------|--------------|----------|
| Adafruit ICM20X | `setAccelRateDivisor(uint16_t)` | `libraries/Adafruit_ICM20X/Adafruit_ICM20X.h` |
| Adafruit ICM20X | `setGyroRateDivisor(uint8_t)` | same |
| Adafruit ICM20X | Bank 2 register write for DLPF | same — search for `writeExternalRegister` or `_writeBankRegister` |
| Adafruit BMP3XX | `setMode(BMP3_MODE_NORMAL)` | `libraries/Adafruit_BMP3XX/Adafruit_BMP3XX.h` |
| Adafruit BMP3XX | `dataReady()` | same |

If a method doesn't exist, leave a `// TODO:` comment and keep the existing behavior.
Don't fight the library — the preset system adds value even if some register-level
settings have to wait.

---

## Calibration (Future Sprint)

Not implemented now. When ready:
1. Add `IMUCalibration { Vec3 accel_offset; Vec3 gyro_offset; }` struct to `imu.hpp`
2. Pass it into `IMU::init(config, calibration)`
3. Apply offsets in `readAccel()` and `readGyro()`
4. Collect calibration data: place rocket vertical, run firmware in `lowNoiseConfig()`,
   average 500 samples, compute offsets vs expected (0, 0, 9.81)
