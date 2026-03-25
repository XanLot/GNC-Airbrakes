# GNC PCB Sensor Integration Design

## Overview

Replace the current single-IMU + single-baro firmware (ICM-20948 + BMP388) with full support for the GNC PCB's sensor suite: 4x LSM6DSVTR IMUs, 2x BMP581 barometers, 1x MMC5983MA magnetometer, and 1x TMP117 temperature sensor. Add a `DEBUG_MODE` build that streams all sensor data over serial in a parseable format, paired with a Python live visualizer.

## Hardware Topology

### SPI0 (default `SPI` bus)
| Sensor | Type | CS Pin | SPI Mode |
|--------|------|--------|----------|
| IMU1 (LSM6DSVTR) | 6-axis accel+gyro | GPIO 37 | Mode 3 |
| IMU2 (LSM6DSVTR) | 6-axis accel+gyro | GPIO 38 | Mode 3 |
| Baro1 (BMP581) | Pressure/altitude | GPIO 10 | Mode 0 |

### SPI1 (`SPI1` bus)
| Sensor | Type | CS Pin | SPI Mode |
|--------|------|--------|----------|
| IMU3 (LSM6DSVTR) | 6-axis accel+gyro | GPIO 28 | Mode 3 |
| IMU4 (LSM6DSVTR) | 6-axis accel+gyro | GPIO 33 | Mode 3 |
| Baro2 (BMP581) | Pressure/altitude | GPIO 34 | Mode 0 |

### I2C (`Wire` bus, SDA=18, SCL=19)
| Sensor | Type | Address |
|--------|------|---------|
| MMC5983MA | 3-axis magnetometer | 0x30 |
| TMP117 | Precision temperature | 0x48 |

All CS lines have 10k pull-ups to 3V3. All sensors powered from 3.3V.

### SPI Transaction Safety

SPI0 and SPI1 each have mixed SPI modes on the same bus (Mode 3 for IMUs, Mode 0 for baros). Both the Adafruit LSM6DSV and SparkFun BMP581 libraries must use `SPI.beginTransaction(SPISettings(...))` / `SPI.endTransaction()` around every transfer to set the correct mode. This is standard for Adafruit and SparkFun Arduino libs. If the BMP581 library does not accept an `SPIClass*` reference for SPI1, we will need to wrap it or use a different library. Verify during library integration.

### I2C Bus Speed

Use `Wire.setClock(400000)` after `Wire.begin()` to run I2C at 400 kHz (fast mode). Both MMC5983MA and TMP117 support this. At 100 kHz default, two I2C reads add ~1 ms per tick, which eats into the 4.4 ms budget at 225 Hz.

## Libraries

| Sensor | Library | Notes |
|--------|---------|-------|
| LSM6DSVTR | `Adafruit_LSM6DSV` | SPI, pass `&SPI` or `&SPI1` to `begin_SPI()` |
| BMP581 | `SparkFun_BMP581` | SPI, needs `&SPI` or `&SPI1` reference |
| MMC5983MA | `SparkFun_MMC5983MA` | I2C only, address 0x30 |
| TMP117 | `SparkFun_TMP117` | I2C only, address 0x48 |

These get cloned into `libraries/` alongside the existing ones.

## Data Structures

These are the default structs. The old ICM-20948/BMP388 structs are gone.

### Structs (in `src/sensor_data.hpp`)

```cpp
struct Vec3 {
    float x, y, z;
};

// one LSM6DSVTR reading (accel + gyro, no mag since that's a separate chip now)
struct IMUData {
    Vec3 accel;    // m/s^2
    Vec3 gyro;     // rad/s
    float temp;    // degrees C (LSM6DSV on-die temp)
};

// one BMP581 reading
struct BarometerData {
    float temperature;  // degrees C
    float pressure;     // Pa
    float altitude;     // m
};

// one MMC5983MA reading
struct MagData {
    Vec3 field;  // Gauss (+/-8G range, 18-bit resolution)
};

// one TMP117 reading
struct TempData {
    float temperature;  // degrees C (0.0078 C resolution)
};

// everything in one tick
struct SensorData {
    IMUData        imu[4];
    BarometerData  baro[2];
    MagData        mag;
    TempData       tmp;
    unsigned long  timestamp_us;
};
```

Note: `IMUData` no longer has a `mag` field since the LSM6DSVTR has no built-in magnetometer. Magnetometer data comes from the separate MMC5983MA via `MagData`.

## Sensor Drivers

### LSM6DSVTR driver (`src/imu.hpp` / `src/imu.cpp`)

```cpp
struct IMUConfig {
    uint8_t cs_pin;
    SPIClass* spi_bus;         // &SPI or &SPI1
    uint32_t spi_speed;        // Hz, max 10 MHz per datasheet
    uint8_t accel_range;       // LSM6DS_ACCEL_RANGE_*
    uint8_t gyro_range;        // LSM6DS_GYRO_RANGE_*
    uint16_t accel_odr;        // LSM6DS_RATE_*
    uint16_t gyro_odr;         // LSM6DS_RATE_*
    uint8_t accel_lpf2_bw;     // LPF2 bandwidth: 0=ODR/4 .. 7=ODR/800
    bool    accel_lpf2_enable; // enable second-stage accel low-pass
    uint8_t gyro_lpf1_bw;      // gyro LPF1 bandwidth setting
    bool    gyro_lpf1_enable;
};

class IMU {
public:
    bool init(const IMUConfig& config);
    bool update();
    IMUData readAll() const;

    static IMUConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static IMUConfig debugConfig(uint8_t cs_pin, SPIClass* bus);
private:
    void applyConfig(const IMUConfig& config);
    // Adafruit_LSM6DSV instance, config, latest data
};
```

Four instances created in main, one per IMU, each with its own CS pin and SPI bus pointer.

#### flightConfig preset (from LSM6DSV datasheet)
- **Accel range:** +/-8g (6.3g peak fits with margin, 0.244 mg/LSB resolution)
- **Gyro range:** +/-500 dps (small rocket won't spin faster)
- **ODR:** 240 Hz (register 0x07, high-performance mode). Closest Adafruit enum is `LSM6DS_RATE_208_HZ`, may need direct register write for exact 240 Hz.
- **Accel LPF2:** Enabled, LIGHT (ODR/20 = 12 Hz cutoff at 240 Hz). Filters motor vibration while keeping group delay reasonable for airbrake control.
- **Gyro LPF1:** Enabled, setting 101 (53 Hz cutoff at 240 Hz). Rejects vibration, preserves attitude dynamics.
- **SPI speed:** 8 MHz
- **Power mode:** High-performance (lowest noise)

#### debugConfig preset
Same as flight but 60 Hz ODR and more aggressive filtering for cleaner bench readings.

### BMP581 driver (`src/barometer.hpp` / `src/barometer.cpp`)

```cpp
struct BarometerConfig {
    uint8_t cs_pin;
    SPIClass* spi_bus;
    uint32_t spi_speed;        // Hz, max 10 MHz per datasheet
    uint8_t pressure_osr;      // BMP5_OVERSAMPLING_*
    uint8_t temp_osr;          // BMP5_OVERSAMPLING_*
    uint8_t odr;               // BMP5_ODR_*
    uint8_t iir_pressure;      // BMP5_IIR_FILTER_COEFF_*
    uint8_t iir_temp;          // BMP5_IIR_FILTER_COEFF_* (usually bypass)
    uint8_t power_mode;        // BMP5_POWERMODE_*
    float sea_level_hpa;
};

class Barometer {
public:
    bool init(const BarometerConfig& config);
    bool update();
    BarometerData readAll() const;

    static BarometerConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static BarometerConfig debugConfig(uint8_t cs_pin, SPIClass* bus);

    void setSeaLevelPressure(float hpa);
private:
    // SparkFun_BMP581 instance, config, latest data
};
```

#### flightConfig preset (from BMP581 datasheet)
- **Power mode:** Normal (continuous at configured ODR, predictable timing for Kalman filter)
- **ODR:** 50 Hz. Gives 4.8:1 ratio with 240 Hz IMU, good for sensor fusion.
- **Pressure OSR:** 8x (0.30 Pa noise, ~2.5 cm altitude resolution at sea level). Max normal-mode ODR at 8x is 140 Hz, so 50 Hz is well within limits.
- **Temp OSR:** 1x (temperature only needed for pressure compensation)
- **IIR pressure:** Coeff 7 (normalized BW 0.0212, ~1.06 Hz cutoff at 50 Hz). Smooths wind/vibration without being too sluggish for airbrake response.
- **IIR temp:** Bypass (temperature changes slowly)
- **SPI speed:** 8 MHz

### MMC5983MA driver (`src/magnetometer.hpp` / `src/magnetometer.cpp`)

```cpp
class Magnetometer {
public:
    bool init();  // uses Wire, address 0x30
    bool update();
    MagData readAll() const;
private:
    // SparkFun_MMC5983MA instance, latest data
};
```

#### Init config (from MMC5983MA datasheet)
- **Mode:** Continuous at 50 Hz (matches baro rate for unified slow-sensor fusion step)
- **Filter bandwidth:** 200 Hz (4 ms measurement time, low noise while fast enough for 50 Hz continuous)
- **SET/RESET:** Auto SET/RESET enabled. Critical because the rocket motor's magnetic field will saturate the AMR elements during boost. Auto SET/RESET degausses every measurement, ensuring clean data during coast.
- **Output:** 18-bit, +/-8 Gauss range, 16384 LSB/Gauss

### TMP117 driver (`src/temp_sensor.hpp` / `src/temp_sensor.cpp`)

```cpp
class TempSensor {
public:
    bool init();  // uses Wire, address 0x48
    bool update();
    TempData readAll() const;
private:
    // SparkFun_TMP117 instance, latest data
};
```

#### Init config (from TMP117 datasheet)
- **Mode:** Continuous conversion
- **Averaging:** 8x (smooths PCB electrical noise, still updates fast)
- **Resolution:** 0.0078 C (16-bit, way more than needed)
- **Read rate:** Only needs ~1-5 Hz for telemetry. The main loop reads it every tick but the sensor's internal conversion rate limits actual updates.

## Pin Assignments (compile-time constants)

In `src/pins.hpp`:
```cpp
// SPI0 bus sensors
constexpr uint8_t IMU1_CS  = 37;
constexpr uint8_t IMU2_CS  = 38;
constexpr uint8_t BARO1_CS = 10;

// SPI1 bus sensors
constexpr uint8_t IMU3_CS  = 28;
constexpr uint8_t IMU4_CS  = 33;
constexpr uint8_t BARO2_CS = 34;

// I2C addresses
constexpr uint8_t MAG_ADDR  = 0x30;
constexpr uint8_t TEMP_ADDR = 0x48;
```

## Updated main.cpp

```cpp
// sensor instances
IMU          imu1, imu2, imu3, imu4;
Barometer    baro1, baro2;
Magnetometer mag;
TempSensor   tmp117;

void setup() {
    Serial.begin(115200);
    delay(500);

    SPI.begin();
    SPI1.begin();
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast mode

    // init all sensors with flight configs
    imu1.init(IMU::flightConfig(IMU1_CS, &SPI));
    imu2.init(IMU::flightConfig(IMU2_CS, &SPI));
    imu3.init(IMU::flightConfig(IMU3_CS, &SPI1));
    imu4.init(IMU::flightConfig(IMU4_CS, &SPI1));
    baro1.init(Barometer::flightConfig(BARO1_CS, &SPI));
    baro2.init(Barometer::flightConfig(BARO2_CS, &SPI1));
    mag.init();
    tmp117.init();

    sdLog.init();
    // ...
}

void loop() {
    #ifdef DEBUG_MODE
        debugLoop();  // tagged serial dump, no state machine
    #else
        flightLoop(); // normal state machine + SD logging
    #endif
}
```

## SD Logging (wide CSV)

Updated CSV header:
```
timestamp_us,imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_temp,imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_temp,imu3_ax,imu3_ay,imu3_az,imu3_gx,imu3_gy,imu3_gz,imu3_temp,imu4_ax,imu4_ay,imu4_az,imu4_gx,imu4_gy,imu4_gz,imu4_temp,baro1_temp,baro1_pres,baro1_alt,baro2_temp,baro2_pres,baro2_alt,mag_x,mag_y,mag_z,tmp117_temp
```

`sd_log::log()` updated to accept `SensorData` instead of separate args. Same buffered write strategy, same 1s flush interval.

**Buffer sizing:** The wide row is ~450-500 bytes (38 float columns + timestamp + delimiters). The row buffer increases from `char row[256]` to `char row[512]`. The write buffer increases from 4 KB to 16 KB (`BUF_SIZE = 16384`) so it holds ~32 rows before flushing, reducing SD write frequency.

**Sensor failure handling:** If a sensor fails init, its fields are filled with `NAN` in every log row. MATLAB detects these with `ismissing()` or `isnan()`. A `uint8_t sensorStatus` bitmask tracks which sensors initialized successfully.

MATLAB reads it with `readtable('log_0000.csv')` as before, just more columns.

## State Machine Changes

The state machine currently uses accel magnitude for launch/burnout detection and barometer altitude for apogee detection. Updated to accept `SensorData`:

- Launch detection: accel magnitude from `imu[0]` (primary). Could later fuse all 4.
- Apogee detection: altitude from `baro[0]` (primary).
- The `PreLaunchSample` struct holds `SensorData` instead of separate IMU + baro.

No logic changes, just struct swaps.

## DEBUG_MODE Build

### Build: `make debug`

Adds `-DDEBUG_MODE` to DEFINES. Compiles same src/ files. The `#ifdef` in main.cpp routes to `debugLoop()`.

Makefile target:
```makefile
.PHONY: debug
debug: DEFINES += -DDEBUG_MODE
debug: build
```

### Tagged serial protocol

Each sensor prints a tagged line per tick:
```
$IMU1,ax,ay,az,gx,gy,gz,temp
$IMU2,ax,ay,az,gx,gy,gz,temp
$IMU3,ax,ay,az,gx,gy,gz,temp
$IMU4,ax,ay,az,gx,gy,gz,temp
$BARO1,temp,pres,alt
$BARO2,temp,pres,alt
$MAG,x,y,z
$TMP,temp
$TICK,timestamp_us
```

Lines starting with `$` are data. Everything else (boot messages, error prints) is ignored by the parser. `$TICK` marks end of a sample group.

### debugLoop() implementation

Lives in `src/debug_mode.cpp`, guarded by `#ifdef DEBUG_MODE`:
```cpp
void debugLoop() {
    imu1.update(); imu2.update(); imu3.update(); imu4.update();
    baro1.update(); baro2.update();
    mag.update();
    tmp117.update();

    // print tagged lines for each sensor
    printIMU("$IMU1", imu1.readAll());
    printIMU("$IMU2", imu2.readAll());
    printIMU("$IMU3", imu3.readAll());
    printIMU("$IMU4", imu4.readAll());
    printBaro("$BARO1", baro1.readAll());
    printBaro("$BARO2", baro2.readAll());
    printMag("$MAG", mag.readAll());
    printTemp("$TMP", tmp117.readAll());
    Serial.printf("$TICK,%lu\n", micros());

    delay(4); // ~225 Hz to match flight rate
}
```

## Python Live Visualizer (`tools/sensor_monitor.py`)

Dependencies: `pyserial`, `matplotlib`

Usage: `python tools/sensor_monitor.py /dev/cu.usbmodem*`

Layout: matplotlib figure with subplots arranged in a grid:
- Row 1: IMU1-4 accelerometer (4 subplots, each showing X/Y/Z traces)
- Row 2: IMU1-4 gyroscope (4 subplots)
- Row 3: Baro1 alt, Baro2 alt, Mag XYZ, TMP117 temp (4 subplots)

Rolling window of ~5 seconds of data. Updates at ~30 fps using `matplotlib.animation.FuncAnimation`. Filters lines by `$` prefix, splits on `,`, routes to the correct subplot buffer by tag.

## Files to Create/Modify

### New files
- `src/sensor_data.hpp` - shared data structs (Vec3, IMUData, BarometerData, MagData, TempData, SensorData)
- `src/pins.hpp` - pin assignments and I2C addresses
- `src/imu.hpp` / `src/imu.cpp` - LSM6DSVTR driver (replaces old ICM-20948 version)
- `src/barometer.hpp` / `src/barometer.cpp` - BMP581 driver (replaces old BMP388 version)
- `src/magnetometer.hpp` / `src/magnetometer.cpp` - MMC5983MA driver
- `src/temp_sensor.hpp` / `src/temp_sensor.cpp` - TMP117 driver
- `src/debug_mode.hpp` / `src/debug_mode.cpp` - debug serial dump
- `tools/sensor_monitor.py` - Python live visualizer

### Modified files
- `src/main.cpp` - new sensor instances, SPI1/Wire init, DEBUG_MODE routing
- `src/sd_log_file.hpp` / `src/sd_log_file.cpp` - wide CSV with SensorData
- `src/state_machine.hpp` / `src/state_machine.cpp` - use SensorData
- `Makefile` - add `debug` target with `-DDEBUG_MODE`

### Rewritten files (same names, completely new contents)
- `src/imu.hpp` / `src/imu.cpp` - was ICM-20948, now LSM6DSVTR
- `src/barometer.hpp` / `src/barometer.cpp` - was BMP388, now BMP581

### Libraries to add to `libraries/`
- `Adafruit_LSM6DSV`
- `SparkFun_BMP581`
- `SparkFun_MMC5983MA`
- `SparkFun_TMP117`

## What This Does NOT Change

- SIM_MODE: sim mode currently depends on the old `IMUData` and `BarometerData` structs. It lives on the `sim-mode` branch and will be temporarily broken by this change. It needs a separate migration to use `SensorData` and a new `SIM.BIN` format. That is out of scope for this branch.
- Kalman filter / airbrake actuator: future work, not touched here.
- MPC.m: standalone MATLAB script, not affected.
