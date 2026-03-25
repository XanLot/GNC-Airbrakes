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

### New structs (in `src/sensor_data.hpp`)

```cpp
struct Vec3 {
    float x, y, z;
};

// one LSM6DSVTR reading
struct IMU6Data {
    Vec3 accel;    // m/s^2
    Vec3 gyro;     // rad/s
    float temp;    // degrees C (from LSM6DSV on-die temp)
};

// one BMP581 reading
struct BaroData {
    float temperature;  // degrees C
    float pressure;     // Pa
    float altitude;     // m
};

// one MMC5983MA reading
struct MagData {
    Vec3 mag;  // microtesla (or Gauss, depending on lib output)
};

// one TMP117 reading
struct TempData {
    float temperature;  // degrees C
};

// everything in one tick
struct AllSensorData {
    IMU6Data imu[4];
    BaroData baro[2];
    MagData  mag;
    TempData tmp;
    unsigned long timestamp_us;
};
```

The old `IMUData` and `BarometerData` structs are replaced by these. The state machine and SD logger are updated to use `AllSensorData`.

### Why rename IMUData to IMU6Data
The LSM6DSVTR is a 6-axis sensor (accel + gyro). The old `IMUData` included magnetometer data from the ICM-20948's built-in AK09916. With the new PCB, magnetometer data comes from the separate MMC5983MA, so it makes sense to split the structs.

## Sensor Drivers

### LSM6DSVTR driver (`src/lsm6dsv.hpp` / `src/lsm6dsv.cpp`)

```cpp
struct LSM6DSVConfig {
    uint8_t cs_pin;
    SPIClass* spi_bus;       // &SPI or &SPI1
    uint32_t spi_speed;      // Hz, max 10 MHz per datasheet
    uint8_t accel_range;     // LSM6DSV_ACCEL_RANGE_*
    uint8_t gyro_range;      // LSM6DSV_GYRO_RANGE_*
    uint16_t accel_odr;      // LSM6DSV_RATE_*
    uint16_t gyro_odr;
};

class LSM6DSV {
public:
    bool init(const LSM6DSVConfig& config);
    bool update();
    IMU6Data readAll() const;

    static LSM6DSVConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static LSM6DSVConfig debugConfig(uint8_t cs_pin, SPIClass* bus);
private:
    // Adafruit_LSM6DSV instance, config, latest data
};
```

Four instances created in main, one per IMU, each with its own CS pin and SPI bus pointer.

### BMP581 driver (`src/bmp581.hpp` / `src/bmp581.cpp`)

```cpp
struct BMP581Config {
    uint8_t cs_pin;
    SPIClass* spi_bus;
    uint32_t spi_speed;      // Hz, max 10 MHz per datasheet
    uint8_t pressure_osr;
    uint8_t temp_osr;
    uint8_t odr;
    uint8_t iir_coeff;
    float sea_level_hpa;
};

class BMP581Sensor {
public:
    bool init(const BMP581Config& config);
    bool update();
    BaroData readAll() const;

    static BMP581Config flightConfig(uint8_t cs_pin, SPIClass* bus);
private:
    // SparkFun_BMP581 instance, config, latest data
};
```

### MMC5983MA driver (`src/mmc5983.hpp` / `src/mmc5983.cpp`)

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

Sends SET pulse on init per datasheet requirement.

### TMP117 driver (`src/tmp117.hpp` / `src/tmp117.cpp`)

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
LSM6DSV imu1, imu2, imu3, imu4;
BMP581Sensor baro1, baro2;
Magnetometer mag;
TempSensor tmp117;

void setup() {
    Serial.begin(115200);
    delay(500);

    SPI.begin();
    SPI1.begin();
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast mode

    // init all sensors with flight configs
    imu1.init(LSM6DSV::flightConfig(IMU1_CS, &SPI));
    imu2.init(LSM6DSV::flightConfig(IMU2_CS, &SPI));
    imu3.init(LSM6DSV::flightConfig(IMU3_CS, &SPI1));
    imu4.init(LSM6DSV::flightConfig(IMU4_CS, &SPI1));
    baro1.init(BMP581Sensor::flightConfig(BARO1_CS, &SPI));
    baro2.init(BMP581Sensor::flightConfig(BARO2_CS, &SPI1));
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

`sd_log::log()` updated to accept `AllSensorData` instead of separate IMUData + BarometerData. Same buffered write strategy, same 1s flush interval.

**Buffer sizing:** The wide row is ~450-500 bytes (38 float columns + timestamp + delimiters). The row buffer increases from `char row[256]` to `char row[512]`. The write buffer increases from 4 KB to 16 KB (`BUF_SIZE = 16384`) so it holds ~32 rows before flushing, reducing SD write frequency.

**Sensor failure handling:** If a sensor fails init, its fields are filled with `NAN` in every log row. MATLAB detects these with `ismissing()` or `isnan()`. A `uint8_t sensorStatus` bitmask tracks which sensors initialized successfully.

MATLAB reads it with `readtable('log_0000.csv')` as before, just more columns.

## State Machine Changes

The state machine currently uses `IMUData` (accel magnitude) for launch/burnout detection and `BarometerData` for apogee detection. Updated to use `AllSensorData`:

- Launch detection: accel magnitude from `imu[0]` (primary). Could later fuse all 4.
- Apogee detection: altitude from `baro[0]` (primary).
- The `PreLaunchSample` struct holds `AllSensorData` instead of separate IMU + baro.

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
- `src/sensor_data.hpp` - shared data structs (Vec3, IMU6Data, BaroData, MagData, TempData, AllSensorData)
- `src/pins.hpp` - pin assignments and I2C addresses
- `src/lsm6dsv.hpp` / `src/lsm6dsv.cpp` - LSM6DSVTR driver
- `src/bmp581.hpp` / `src/bmp581.cpp` - BMP581 driver
- `src/mmc5983.hpp` / `src/mmc5983.cpp` - MMC5983MA driver
- `src/tmp117.hpp` / `src/tmp117.cpp` - TMP117 driver
- `src/debug_mode.hpp` / `src/debug_mode.cpp` - debug serial dump
- `tools/sensor_monitor.py` - Python live visualizer

### Modified files
- `src/main.cpp` - new sensor instances, SPI1/Wire init, DEBUG_MODE routing
- `src/sd_log_file.hpp` / `src/sd_log_file.cpp` - wide CSV with AllSensorData
- `src/state_machine.hpp` / `src/state_machine.cpp` - use AllSensorData
- `Makefile` - add `debug` target with `-DDEBUG_MODE`

### Removed files
- `src/imu.hpp` / `src/imu.cpp` - replaced by lsm6dsv driver
- `src/barometer.hpp` / `src/barometer.cpp` - replaced by bmp581 driver

### Libraries to add to `libraries/`
- `Adafruit_LSM6DSV`
- `SparkFun_BMP581`
- `SparkFun_MMC5983MA`
- `SparkFun_TMP117`

## What This Does NOT Change

- SIM_MODE: sim mode currently depends on the old `IMUData` and `BarometerData` structs. It lives on the `sim-mode` branch and will be temporarily broken by this change. It needs a separate migration to use `AllSensorData` and a new `SIM.BIN` format. That is out of scope for this branch.
- Kalman filter / airbrake actuator: future work, not touched here.
- MPC.m: standalone MATLAB script, not affected.
