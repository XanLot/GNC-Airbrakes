# PCB Sensor Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ICM-20948 + BMP388 sensor stack with the GNC PCB's full sensor suite (4x LSM6DSVTR, 2x BMP581, MMC5983MA, TMP117), add a debug serial stream build, and a Python live visualizer.

**Architecture:** Sensor drivers wrap Arduino libraries behind the same `init()`/`update()`/`readAll()` pattern used by the old drivers. A shared `SensorData` struct aggregates all 8 sensors per tick. The state machine and SD logger consume `SensorData`. A `DEBUG_MODE` build replaces the flight loop with tagged serial output parsed by a Python matplotlib dashboard.

**Tech Stack:** C++23 (arm-none-eabi-g++), Teensy 4.1, Adafruit_LSM6DSV, SparkFun_BMP581, SparkFun_MMC5983MA, SparkFun_TMP117, Python 3 (pyserial + matplotlib)

**Spec:** `docs/superpowers/specs/2026-03-25-pcb-sensor-integration-design.md`

**Note:** This is embedded firmware. There are no host-side unit tests. "Testing" means `make build` compiles cleanly and `make debug && make upload` works on hardware. Each task ends with a compile check.

---

### Task 1: Add Arduino libraries

Clone the four new sensor libraries into `libraries/`. The Makefile's `find` auto-discovers them.

**Files:**
- Create: `libraries/Adafruit_LSM6DSV/` (git clone)
- Create: `libraries/SparkFun_BMP581/` (git clone)
- Create: `libraries/SparkFun_MMC5983MA/` (git clone)
- Create: `libraries/SparkFun_TMP117/` (git clone)

**Dependencies:** Adafruit_LSM6DSV depends on `Adafruit_BusIO` and `Adafruit_Sensor` (already in `libraries/`). SparkFun libs are self-contained.

- [ ] **Step 1: Clone Adafruit_LSM6DSV**

```bash
cd libraries
git clone https://github.com/adafruit/Adafruit_LSM6DS.git Adafruit_LSM6DSV
```

Note: The repo is named `Adafruit_LSM6DS` (covers LSM6DSV, LSM6DSO, etc.). Clone it as `Adafruit_LSM6DSV` for clarity.

- [ ] **Step 2: Clone SparkFun_BMP581**

```bash
git clone https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library.git SparkFun_BMP581
```

- [ ] **Step 3: Clone SparkFun_MMC5983MA**

```bash
git clone https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library.git SparkFun_MMC5983MA
```

- [ ] **Step 4: Clone SparkFun_TMP117**

```bash
git clone https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library.git SparkFun_TMP117
```

- [ ] **Step 5: Verify library structure**

Check that the Makefile can find headers. The Makefile uses `find libraries -maxdepth 2 -type d` for include paths, so headers must be at depth 1 or 2 within each library folder. SparkFun libs often put source in `src/`, which is at depth 2. Verify:

```bash
# these should all find .h files
ls libraries/Adafruit_LSM6DSV/*.h
ls libraries/SparkFun_BMP581/src/*.h
ls libraries/SparkFun_MMC5983MA/src/*.h
ls libraries/SparkFun_TMP117/src/*.h
```

If SparkFun libs have `src/` subdirectories, the Makefile's `find -maxdepth 2` will find them. If any lib nests headers deeper, symlink or copy headers up.

- [ ] **Step 6: Verify library APIs before writing driver code**

Critical checks that affect Tasks 3-6:

```bash
# 1. Confirm Adafruit_LSM6DSV.h exists
ls libraries/Adafruit_LSM6DSV/Adafruit_LSM6DSV.h

# 2. Check begin_SPI signature accepts SPIClass*
grep "begin_SPI" libraries/Adafruit_LSM6DSV/Adafruit_LSM6DS.h

# 3. Check getEvent takes 3 args (no mag, unlike ICM-20948)
grep "getEvent" libraries/Adafruit_LSM6DSV/Adafruit_LSM6DS.h

# 4. CRITICAL: Check SparkFun BMP581 beginSPI accepts SPIClass* for SPI1
grep -n "beginSPI" libraries/SparkFun_BMP581/src/*.h libraries/SparkFun_BMP581/src/*.cpp
# If only accepts (cs_pin), Baro2 on SPI1 won't work.
# Fallback: patch the library to add SPIClass* parameter.

# 5. Check SparkFun MMC5983MA method names match what Tasks 5 uses
grep -E "setContinuous|setFilter|getMeasurement|enableAutomatic" \
    libraries/SparkFun_MMC5983MA/src/*.h

# 6. Check SparkFun TMP117 method names
grep -E "setConversion|readTemp|dataReady|begin" libraries/SparkFun_TMP117/src/*.h
```

If any API doesn't match the plan, adjust the corresponding task's code before implementing it.

- [ ] **Step 7: Remove old sensor libraries**

```bash
rm -rf libraries/Adafruit_ICM20X
rm -rf libraries/Adafruit_BMP388
```

- [ ] **Step 8: Commit**

```bash
git add libraries/Adafruit_LSM6DSV libraries/SparkFun_BMP581 libraries/SparkFun_MMC5983MA libraries/SparkFun_TMP117
git rm -r libraries/Adafruit_ICM20X libraries/Adafruit_BMP388
git commit -m "swap sensor libraries for GNC PCB"
```

---

### Task 2: Create shared data structs and pin definitions

Foundation types that all other files depend on.

**Files:**
- Create: `src/sensor_data.hpp`
- Create: `src/pins.hpp`

- [ ] **Step 1: Write `src/sensor_data.hpp`**

```cpp
#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <cstdint>
#include <cmath>

struct Vec3 {
    float x, y, z;
};

struct IMUData {
    Vec3 accel;    // m/s^2
    Vec3 gyro;     // rad/s
    float temp;    // degrees C (on-die)
};

struct BarometerData {
    float temperature;  // degrees C
    float pressure;     // Pa
    float altitude;     // m
};

struct MagData {
    Vec3 field;  // Gauss
};

struct TempData {
    float temperature;  // degrees C
};

struct SensorData {
    IMUData        imu[4];
    BarometerData  baro[2];
    MagData        mag;
    TempData       tmp;
    unsigned long  timestamp_us;
};

// helper: fill an IMUData with NAN (for failed sensors)
inline IMUData nanIMU() {
    return IMUData{
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN
    };
}

inline BarometerData nanBaro() {
    return BarometerData{NAN, NAN, NAN};
}

inline MagData nanMag() {
    return MagData{{NAN, NAN, NAN}};
}

inline TempData nanTemp() {
    return TempData{NAN};
}

#endif // SENSOR_DATA_HPP
```

- [ ] **Step 2: Write `src/pins.hpp`**

```cpp
#ifndef PINS_HPP
#define PINS_HPP

#include <cstdint>

// SPI0 bus sensors
constexpr uint8_t IMU1_CS  = 37;
constexpr uint8_t IMU2_CS  = 38;
constexpr uint8_t BARO1_CS = 10;

// SPI1 bus sensors
constexpr uint8_t IMU3_CS  = 28;
constexpr uint8_t IMU4_CS  = 33;
constexpr uint8_t BARO2_CS = 34;

// I2C addresses
constexpr uint8_t MAG_I2C_ADDR  = 0x30;
constexpr uint8_t TEMP_I2C_ADDR = 0x48;

#endif // PINS_HPP
```

- [ ] **Step 3: Compile check**

```bash
make clean_src && make build
```

This will fail because `main.cpp` still includes the old `imu.hpp` which references `Adafruit_ICM20948.h` (now deleted). That's expected. The new headers are correct on their own. Move on to task 3.

- [ ] **Step 4: Commit**

```bash
git add src/sensor_data.hpp src/pins.hpp
git commit -m "add shared sensor data structs and pin definitions"
```

---

### Task 3: Rewrite IMU driver for LSM6DSVTR

Replace the ICM-20948 driver with an LSM6DSVTR driver using Adafruit_LSM6DS.

**Files:**
- Rewrite: `src/imu.hpp`
- Rewrite: `src/imu.cpp`

- [ ] **Step 1: Write `src/imu.hpp`**

```cpp
#ifndef IMU_HPP
#define IMU_HPP

#include "sensor_data.hpp"
#include <cstdint>

class SPIClass;

struct IMUConfig {
    uint8_t cs_pin;
    SPIClass* spi_bus;
    uint32_t spi_speed;
    uint8_t accel_range;
    uint8_t gyro_range;
    uint16_t accel_odr;
    uint16_t gyro_odr;
    uint8_t accel_lpf2_bw;
    bool    accel_lpf2_enable;
    uint8_t gyro_lpf1_bw;
    bool    gyro_lpf1_enable;
};

class IMU {
public:
    IMU();

    bool init(const IMUConfig& config);
    bool update();
    IMUData readAll() const;

    static IMUConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static IMUConfig debugConfig(uint8_t cs_pin, SPIClass* bus);

private:
    void applyConfig(const IMUConfig& config);
    bool initialized_;
    IMUData latest_;
    Adafruit_LSM6DSV* sensor_;
};

#endif // IMU_HPP
```

- [ ] **Step 2: Write `src/imu.cpp`**

Each `IMU` instance owns its own `Adafruit_LSM6DSV*` pointer (stored as a member). No static arrays.

```cpp
#include "imu.hpp"
#include <Adafruit_LSM6DSV.h>
#include <SPI.h>

IMU::IMU() : initialized_(false), latest_{}, sensor_(nullptr) {}

IMUConfig IMU::flightConfig(uint8_t cs_pin, SPIClass* bus) {
    return IMUConfig{
        .cs_pin             = cs_pin,
        .spi_bus            = bus,
        .spi_speed          = 8000000,
        .accel_range        = LSM6DSV_ACCEL_RANGE_8_G,
        .gyro_range         = LSM6DSV_GYRO_RANGE_500_DPS,
        .accel_odr          = LSM6DSV_RATE_240_HZ,
        .gyro_odr           = LSM6DSV_RATE_240_HZ,
        .accel_lpf2_bw      = 2,     // LIGHT: ODR/20 = 12 Hz at 240 Hz
        .accel_lpf2_enable  = true,
        .gyro_lpf1_bw       = 5,     // 53 Hz cutoff at 240 Hz
        .gyro_lpf1_enable   = true
    };
}

IMUConfig IMU::debugConfig(uint8_t cs_pin, SPIClass* bus) {
    return IMUConfig{
        .cs_pin             = cs_pin,
        .spi_bus            = bus,
        .spi_speed          = 8000000,
        .accel_range        = LSM6DSV_ACCEL_RANGE_4_G,
        .gyro_range         = LSM6DSV_GYRO_RANGE_250_DPS,
        .accel_odr          = LSM6DSV_RATE_60_HZ,
        .gyro_odr           = LSM6DSV_RATE_60_HZ,
        .accel_lpf2_bw      = 4,     // STRONG: ODR/100 = 0.6 Hz at 60 Hz
        .accel_lpf2_enable  = true,
        .gyro_lpf1_bw       = 6,     // ~27 Hz cutoff
        .gyro_lpf1_enable   = true
    };
}

bool IMU::init(const IMUConfig& config) {
    sensor_ = new Adafruit_LSM6DSV();
    // begin_SPI(cs, spiClass, sensorId, frequency)
    if (!sensor_->begin_SPI(config.cs_pin, config.spi_bus, 0, config.spi_speed)) {
        delete sensor_;
        sensor_ = nullptr;
        return false;
    }
    applyConfig(config);
    initialized_ = true;
    return true;
}

void IMU::applyConfig(const IMUConfig& config) {
    sensor_->setAccelRange(static_cast<lsm6dsv_accel_range_t>(config.accel_range));
    sensor_->setGyroRange(static_cast<lsm6dsv_gyro_range_t>(config.gyro_range));
    sensor_->setAccelDataRate(static_cast<lsm6dsv_data_rate_t>(config.accel_odr));
    sensor_->setGyroDataRate(static_cast<lsm6dsv_data_rate_t>(config.gyro_odr));

    // TODO: LPF register writes. After cloning the library, check if
    // Adafruit_LSM6DSV exposes enableAccelFilter() or similar. If not,
    // use sensor_->writeRegister(CTRL8_XL, ...) for accel LPF2 and
    // sensor_->writeRegister(CTRL6_G, ...) for gyro LPF1.
}

bool IMU::update() {
    if (!initialized_) return false;

    // LSM6DSV getEvent takes 3 args (no magnetometer, unlike ICM-20948's 4)
    sensors_event_t accel, gyro, temp;
    if (!sensor_->getEvent(&accel, &gyro, &temp)) {
        return false;
    }

    latest_.accel = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
    latest_.gyro  = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
    latest_.temp  = temp.temperature;
    return true;
}

IMUData IMU::readAll() const {
    return latest_;
}
```

- [ ] **Step 3: Compile check**

```bash
make clean_src && make build
```

Will still fail (main.cpp references old types), but `imu.cpp.o` should compile. Check for errors in the IMU translation unit only:

```bash
make build 2>&1 | grep -E "imu\.(cpp|hpp)"
```

- [ ] **Step 4: Commit**

```bash
git add src/imu.hpp src/imu.cpp
git commit -m "rewrite imu driver for LSM6DSVTR"
```

---

### Task 4: Rewrite barometer driver for BMP581

Replace the BMP388 driver with a BMP581 driver using SparkFun_BMP581.

**Files:**
- Rewrite: `src/barometer.hpp`
- Rewrite: `src/barometer.cpp`

- [ ] **Step 1: Write `src/barometer.hpp`**

```cpp
#ifndef BAROMETER_HPP
#define BAROMETER_HPP

#include "sensor_data.hpp"
#include <cstdint>

class SPIClass;

struct BarometerConfig {
    uint8_t cs_pin;
    SPIClass* spi_bus;
    uint32_t spi_speed;
    uint8_t pressure_osr;
    uint8_t temp_osr;
    uint8_t odr;
    uint8_t iir_pressure;
    uint8_t iir_temp;
    uint8_t power_mode;
    float sea_level_hpa;
};

class Barometer {
public:
    Barometer();

    bool init(const BarometerConfig& config);
    bool update();
    BarometerData readAll() const;

    static BarometerConfig flightConfig(uint8_t cs_pin, SPIClass* bus);
    static BarometerConfig debugConfig(uint8_t cs_pin, SPIClass* bus);

    void setSeaLevelPressure(float hpa);

private:
    bool initialized_;
    float sea_level_hpa_;
    BarometerData latest_;
    struct Impl;
    Impl* pimpl_;
};

#endif // BAROMETER_HPP
```

- [ ] **Step 2: Write `src/barometer.cpp`**

Use the SparkFun BMP581 library. Key API:
- `BMP581 bmp;`
- `bmp.beginSPI(cs_pin)` or check if it accepts `SPIClass*`
- `bmp.setODRFrequency(BMP5_ODR_50_HZ)`
- `bmp.setOSRMultipliers(osr_config)`
- `bmp.setFilterConfig(iir_config)`
- `bmp.getSensorData(&data)` returns `bmp5_sensor_data` with `.pressure` and `.temperature`

```cpp
#include "barometer.hpp"
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SPI.h>
#include <cmath>

struct Barometer::Impl {
    BMP581 bmp;
};

Barometer::Barometer()
    : initialized_(false), sea_level_hpa_(1013.25f), latest_{}, pimpl_(new Impl) {}

BarometerConfig Barometer::flightConfig(uint8_t cs_pin, SPIClass* bus) {
    return BarometerConfig{
        .cs_pin        = cs_pin,
        .spi_bus       = bus,
        .spi_speed     = 8000000,
        .pressure_osr  = 3,     // BMP5_OVERSAMPLING_8X
        .temp_osr      = 0,     // BMP5_OVERSAMPLING_1X
        .odr           = 0x0F,  // BMP5_ODR_50_HZ
        .iir_pressure  = 3,     // BMP5_IIR_FILTER_COEFF_7
        .iir_temp      = 0,     // bypass
        .power_mode    = 3,     // BMP5_POWERMODE_NORMAL
        .sea_level_hpa = 1013.25f
    };
}

BarometerConfig Barometer::debugConfig(uint8_t cs_pin, SPIClass* bus) {
    return BarometerConfig{
        .cs_pin        = cs_pin,
        .spi_bus       = bus,
        .spi_speed     = 8000000,
        .pressure_osr  = 2,     // BMP5_OVERSAMPLING_4X
        .temp_osr      = 0,
        .odr           = 0x14,  // BMP5_ODR_25_HZ
        .iir_pressure  = 5,     // BMP5_IIR_FILTER_COEFF_31
        .iir_temp      = 0,
        .power_mode    = 3,     // BMP5_POWERMODE_NORMAL
        .sea_level_hpa = 1013.25f
    };
}

bool Barometer::init(const BarometerConfig& config) {
    sea_level_hpa_ = config.sea_level_hpa;

    // SparkFun BMP581 SPI init
    // Verify the actual beginSPI() signature after cloning (Task 1, Step 6).
    // Expected: beginSPI(cs_pin, clockFreq, spiPort) for SPI1 support.
    // If only beginSPI(cs_pin) exists, patch the library.
    int8_t err = pimpl_->bmp.beginSPI(config.cs_pin, config.spi_speed, *config.spi_bus);
    if (err != BMP5_OK) return false;

    // configure oversampling
    bmp5_osr_odr_press_config osrOdrConfig;
    osrOdrConfig.osr_t = config.temp_osr;
    osrOdrConfig.osr_p = config.pressure_osr;
    osrOdrConfig.press_en = BMP5_ENABLE;
    osrOdrConfig.odr = config.odr;
    err = pimpl_->bmp.setODRFrequency(&osrOdrConfig);
    if (err != BMP5_OK) return false;

    // configure IIR filter
    bmp5_iir_config iirConfig;
    iirConfig.set_iir_t = config.iir_temp;
    iirConfig.set_iir_p = config.iir_pressure;
    iirConfig.shdw_set_iir_t = config.iir_temp;
    iirConfig.shdw_set_iir_p = config.iir_pressure;
    err = pimpl_->bmp.setFilterConfig(&iirConfig);
    if (err != BMP5_OK) return false;

    // set power mode
    err = pimpl_->bmp.setMode(config.power_mode);
    if (err != BMP5_OK) return false;

    initialized_ = true;
    return true;
}

bool Barometer::update() {
    if (!initialized_) return false;

    bmp5_sensor_data data;
    int8_t err = pimpl_->bmp.getSensorData(&data);
    if (err != BMP5_OK) return false;

    latest_.temperature = data.temperature;
    latest_.pressure = data.pressure;
    // altitude from barometric formula
    float atm = latest_.pressure / 100.0f;
    latest_.altitude = 44330.0f * (1.0f - powf(atm / sea_level_hpa_, 0.1903f));
    return true;
}

BarometerData Barometer::readAll() const {
    return latest_;
}

void Barometer::setSeaLevelPressure(float hpa) {
    sea_level_hpa_ = hpa;
}
```

**Important:** The SparkFun BMP581 library's SPI API needs verification. If `beginSPI()` doesn't accept an `SPIClass*` for SPI1, you'll need to either:
1. Check if there's an overload like `beginSPI(cs, clockFreq, spiPort)`
2. Or modify the library to accept it

Check `libraries/SparkFun_BMP581/src/SparkFun_BMP581_Arduino_Library.h` for the exact `beginSPI()` signature after cloning.

- [ ] **Step 3: Compile check**

```bash
make build 2>&1 | grep -E "barometer\.(cpp|hpp)"
```

- [ ] **Step 4: Commit**

```bash
git add src/barometer.hpp src/barometer.cpp
git commit -m "rewrite barometer driver for BMP581"
```

---

### Task 5: Create magnetometer driver

New driver for the MMC5983MA over I2C.

**Files:**
- Create: `src/magnetometer.hpp`
- Create: `src/magnetometer.cpp`

- [ ] **Step 1: Write `src/magnetometer.hpp`**

```cpp
#ifndef MAGNETOMETER_HPP
#define MAGNETOMETER_HPP

#include "sensor_data.hpp"

class Magnetometer {
public:
    Magnetometer();

    bool init();
    bool update();
    MagData readAll() const;

private:
    bool initialized_;
    MagData latest_;
    struct Impl;
    Impl* pimpl_;
};

#endif // MAGNETOMETER_HPP
```

- [ ] **Step 2: Write `src/magnetometer.cpp`**

```cpp
#include "magnetometer.hpp"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>

struct Magnetometer::Impl {
    SFE_MMC5983MA mag;
};

Magnetometer::Magnetometer()
    : initialized_(false), latest_{}, pimpl_(new Impl) {}

bool Magnetometer::init() {
    if (!pimpl_->mag.begin()) {
        return false;
    }

    // auto SET/RESET: degausses every measurement, critical after motor burn exposure
    pimpl_->mag.softReset();
    pimpl_->mag.enableAutomaticSetReset();

    // continuous mode at 50 Hz
    pimpl_->mag.setContinuousModeFrequency(50);
    pimpl_->mag.enableContinuousMode();

    // 200 Hz bandwidth (4 ms measurement, good noise/speed tradeoff)
    pimpl_->mag.setFilterBandwidth(200);

    initialized_ = true;
    return true;
}

bool Magnetometer::update() {
    if (!initialized_) return false;

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    pimpl_->mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

    // convert 18-bit unsigned to signed Gauss
    // midpoint is 131072 (2^17), sensitivity is 16384 LSB/Gauss
    latest_.field.x = (static_cast<float>(rawX) - 131072.0f) / 16384.0f;
    latest_.field.y = (static_cast<float>(rawY) - 131072.0f) / 16384.0f;
    latest_.field.z = (static_cast<float>(rawZ) - 131072.0f) / 16384.0f;

    return true;
}

MagData Magnetometer::readAll() const {
    return latest_;
}
```

- [ ] **Step 3: Commit**

```bash
git add src/magnetometer.hpp src/magnetometer.cpp
git commit -m "add MMC5983MA magnetometer driver"
```

---

### Task 6: Create temperature sensor driver

New driver for TMP117 over I2C.

**Files:**
- Create: `src/temp_sensor.hpp`
- Create: `src/temp_sensor.cpp`

- [ ] **Step 1: Write `src/temp_sensor.hpp`**

```cpp
#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include "sensor_data.hpp"

class TempSensor {
public:
    TempSensor();

    bool init();
    bool update();
    TempData readAll() const;

private:
    bool initialized_;
    TempData latest_;
    struct Impl;
    Impl* pimpl_;
};

#endif // TEMP_SENSOR_HPP
```

- [ ] **Step 2: Write `src/temp_sensor.cpp`**

```cpp
#include "temp_sensor.hpp"
#include <SparkFun_TMP117.h>
#include <Wire.h>

struct TempSensor::Impl {
    TMP117 tmp;
};

TempSensor::TempSensor()
    : initialized_(false), latest_{}, pimpl_(new Impl) {}

bool TempSensor::init() {
    if (!pimpl_->tmp.begin()) {
        return false;
    }

    // continuous mode, 8x averaging
    pimpl_->tmp.setConversionAverageMode(1);  // AVG = 01 -> 8 conversions
    pimpl_->tmp.setContinuousConversionMode();

    initialized_ = true;
    return true;
}

bool TempSensor::update() {
    if (!initialized_) return false;

    if (pimpl_->tmp.dataReady()) {
        latest_.temperature = pimpl_->tmp.readTempC();
    }
    return true;
}

TempData TempSensor::readAll() const {
    return latest_;
}
```

- [ ] **Step 3: Commit**

```bash
git add src/temp_sensor.hpp src/temp_sensor.cpp
git commit -m "add TMP117 temperature sensor driver"
```

---

### Task 7: Update SD logger for wide CSV

Update `sd_log` to accept `SensorData` and write the wide CSV format.

**Files:**
- Modify: `src/sd_log_file.hpp`
- Modify: `src/sd_log_file.cpp`

- [ ] **Step 1: Update `src/sd_log_file.hpp`**

Replace the `#include "imu.hpp"` and `#include "barometer.hpp"` with `#include "sensor_data.hpp"`. Change the `log()` signature:

```cpp
#ifndef SD_LOG_FILE_HPP
#define SD_LOG_FILE_HPP

#include "sensor_data.hpp"

class sd_log {
public:
    sd_log();

    bool init();
    void log(const SensorData& data);
    void flush();

private:
    struct Impl;
    Impl* pimpl;
};

#endif // SD_LOG_FILE_HPP
```

- [ ] **Step 2: Update `src/sd_log_file.cpp`**

Key changes:
- `BUF_SIZE` from 4096 to 16384
- `char row[256]` to `char row[600]`
- New CSV header with 38 columns
- `log()` writes all sensor data per row

```cpp
#include <SD.h>
#include <cstdio>
#include <cstring>
#include "sd_log_file.hpp"

static constexpr size_t BUF_SIZE = 16384;
static constexpr unsigned long FLUSH_INTERVAL_US = 1000000;

struct sd_log::Impl {
    File logFile;
    bool initialized = false;
    char buffer[BUF_SIZE];
    size_t bufPos = 0;
    unsigned long lastFlushUs = 0;
};

sd_log::sd_log() : pimpl(new Impl) {}

bool sd_log::init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return false;
    }

    char filename[32];
    int index = 0;
    do {
        snprintf(filename, sizeof(filename), "log_%04d.csv", index++);
    } while (SD.exists(filename));

    pimpl->logFile = SD.open(filename, FILE_WRITE);
    if (!pimpl->logFile) {
        return false;
    }

    pimpl->initialized = true;
    pimpl->bufPos = 0;

    const char header[] =
        "timestamp_us,"
        "imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_temp,"
        "imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_temp,"
        "imu3_ax,imu3_ay,imu3_az,imu3_gx,imu3_gy,imu3_gz,imu3_temp,"
        "imu4_ax,imu4_ay,imu4_az,imu4_gx,imu4_gy,imu4_gz,imu4_temp,"
        "baro1_temp,baro1_pres,baro1_alt,"
        "baro2_temp,baro2_pres,baro2_alt,"
        "mag_x,mag_y,mag_z,"
        "tmp117_temp\n";

    pimpl->logFile.write(header, strlen(header));
    pimpl->logFile.flush();
    pimpl->lastFlushUs = micros();

    return true;
}

static size_t appendFloat(char* buf, size_t pos, size_t max, float val, uint8_t decimals) {
    char tmp[16];
    dtostrf(val, 0, decimals, tmp);
    size_t tlen = strlen(tmp);
    if (pos + tlen < max) {
        memcpy(buf + pos, tmp, tlen);
        pos += tlen;
    }
    return pos;
}

void sd_log::log(const SensorData& data) {
    if (!pimpl->initialized) return;

    char row[600];
    size_t p = 0;

    // timestamp
    p += snprintf(row + p, sizeof(row) - p, "%lu,", data.timestamp_us);

    // 4 IMUs (7 fields each: ax,ay,az,gx,gy,gz,temp)
    for (int i = 0; i < 4; i++) {
        const auto& imu = data.imu[i];
        p = appendFloat(row, p, sizeof(row), imu.accel.x, 4); row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.accel.y, 4); row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.accel.z, 4); row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.x, 4);  row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.y, 4);  row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.z, 4);  row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.temp, 2);    row[p++] = ',';
    }

    // 2 baros (3 fields each: temp, pressure, altitude)
    for (int i = 0; i < 2; i++) {
        const auto& baro = data.baro[i];
        p = appendFloat(row, p, sizeof(row), baro.temperature, 2); row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), baro.pressure, 1);    row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), baro.altitude, 2);    row[p++] = ',';
    }

    // mag (3 fields)
    p = appendFloat(row, p, sizeof(row), data.mag.field.x, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), data.mag.field.y, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), data.mag.field.z, 4); row[p++] = ',';

    // tmp117 (1 field, last column, no trailing comma)
    p = appendFloat(row, p, sizeof(row), data.tmp.temperature, 4);
    row[p++] = '\n';

    size_t rowLen = p;

    if (pimpl->bufPos + rowLen > BUF_SIZE) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }

    memcpy(pimpl->buffer + pimpl->bufPos, row, rowLen);
    pimpl->bufPos += rowLen;

    unsigned long now = micros();
    if (now - pimpl->lastFlushUs >= FLUSH_INTERVAL_US) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
        pimpl->logFile.flush();
        pimpl->lastFlushUs = now;
    }
}

void sd_log::flush() {
    if (!pimpl->initialized) return;
    if (pimpl->bufPos > 0) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }
    pimpl->logFile.flush();
}
```

- [ ] **Step 3: Commit**

```bash
git add src/sd_log_file.hpp src/sd_log_file.cpp
git commit -m "update sd logger for wide csv with all sensors"
```

---

### Task 8: Update state machine for SensorData

Swap struct types. No logic changes.

**Files:**
- Modify: `src/state_machine.hpp`
- Modify: `src/state_machine.cpp`

- [ ] **Step 1: Update `src/state_machine.hpp`**

Changes:
- Replace `#include "imu.hpp"` and `#include "barometer.hpp"` with `#include "sensor_data.hpp"`
- `PreLaunchSample` holds `SensorData` instead of separate `IMUData` + `BarometerData`
- `update()` takes `const SensorData&`
- `onEnter_Boost()` takes `const SensorData&`
- `checkTransition_OnPad()` takes `const SensorData&`
- `checkTransition_Boost()` takes `const SensorData&`
- `checkTransition_Coast()` takes `const SensorData&`
- `storePreLaunchSample()` takes `const SensorData&`

```cpp
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "sensor_data.hpp"
#include "sd_log_file.hpp"

enum class FlightState {
    ON_PAD,
    BOOST,
    COAST_ONSET,
    COAST,
    RECOVERY
};

enum class AirbrakeStatus {
    LOCKED,
    PERMITTED,
    ACTIVE_CONT
};

constexpr float BOOST_ACCEL_THRESHOLD_MS2   = 5.0f * 9.81f;
constexpr float BURNOUT_ACCEL_THRESHOLD_MS2 = 0.5f * 9.81f;
constexpr float COAST_TIMER_SECONDS         = 3.0f;
constexpr int   PRE_LAUNCH_BUFFER_SIZE      = 100;
constexpr int   BURNOUT_CONFIRM_SAMPLES     = 3;
constexpr int   APOGEE_CONFIRM_SAMPLES      = 5;

struct PreLaunchSample {
    SensorData data;
};

class StateMachine {
public:
    explicit StateMachine(sd_log& sdLog);

    void update(const SensorData& data);
    FlightState getState() const;
    bool isLogging() const;

private:
    FlightState currentState_;
    sd_log&     sdLog_;

    PreLaunchSample preLaunchBuffer_[PRE_LAUNCH_BUFFER_SIZE];
    int             bufferHead_;
    int             bufferCount_;

    unsigned long coastOnsetEntryMs_;
    float previousAltitude_;
    int   altitudeDecreasingCount_;
    int   burnoutConfirmCount_;

    FlightState checkTransition_OnPad(const SensorData& data);
    FlightState checkTransition_Boost(const SensorData& data);
    FlightState checkTransition_CoastOnset();
    FlightState checkTransition_Coast(const SensorData& data);

    void onEnter_OnPad();
    void onEnter_Boost(const SensorData& data);
    void onEnter_CoastOnset();
    void onEnter_Coast();
    void onEnter_Recovery();

    void setAirbrakeStatus(AirbrakeStatus status);

    static float accelMagnitude(const IMUData& imu);
    void         storePreLaunchSample(const SensorData& data);
    void         flushPreLaunchBuffer();
};

#endif // STATE_MACHINE_HPP
```

- [ ] **Step 2: Update `src/state_machine.cpp`**

Key changes (all mechanical struct swaps):
- `update()`: replace `imu`/`baro` params with single `data` param. Use `data.imu[0]` for accel magnitude checks, `data.baro[0]` for altitude checks.
- `checkTransition_OnPad()`: `accelMagnitude(data.imu[0])`
- `checkTransition_Boost()`: `accelMagnitude(data.imu[0])`
- `checkTransition_Coast()`: `data.baro[0].altitude`
- `storePreLaunchSample()`: store entire `SensorData`
- `flushPreLaunchBuffer()`: pass `preLaunchBuffer_[idx].data` to `sdLog_.log()`
- `accelMagnitude()`: unchanged, still takes `const IMUData&`

```cpp
#include "state_machine.hpp"
#include <Arduino.h>
#include <cmath>

StateMachine::StateMachine(sd_log& sdLog)
    : currentState_(FlightState::ON_PAD),
      sdLog_(sdLog),
      bufferHead_(0),
      bufferCount_(0),
      coastOnsetEntryMs_(0),
      previousAltitude_(0.0f),
      altitudeDecreasingCount_(0),
      burnoutConfirmCount_(0)
{
    for (int i = 0; i < PRE_LAUNCH_BUFFER_SIZE; i++) {
        preLaunchBuffer_[i] = PreLaunchSample{};
    }
    onEnter_OnPad();
}

void StateMachine::update(const SensorData& data) {
    FlightState nextState = currentState_;

    switch (currentState_) {
        case FlightState::ON_PAD:
            storePreLaunchSample(data);
            nextState = checkTransition_OnPad(data);
            break;
        case FlightState::BOOST:
            nextState = checkTransition_Boost(data);
            break;
        case FlightState::COAST_ONSET:
            nextState = checkTransition_CoastOnset();
            break;
        case FlightState::COAST:
            nextState = checkTransition_Coast(data);
            previousAltitude_ = data.baro[0].altitude;
            break;
        case FlightState::RECOVERY:
            break;
    }

    if (nextState != currentState_) {
        Serial.print("[STATE] Transition: ");
        Serial.print(static_cast<int>(currentState_));
        Serial.print(" -> ");
        Serial.println(static_cast<int>(nextState));

        currentState_ = nextState;

        switch (currentState_) {
            case FlightState::ON_PAD:      onEnter_OnPad();       break;
            case FlightState::BOOST:       onEnter_Boost(data);   break;
            case FlightState::COAST_ONSET: onEnter_CoastOnset();  break;
            case FlightState::COAST:       onEnter_Coast();       break;
            case FlightState::RECOVERY:    onEnter_Recovery();    break;
        }
    }
}

FlightState StateMachine::getState() const { return currentState_; }
bool StateMachine::isLogging() const { return currentState_ != FlightState::ON_PAD; }

FlightState StateMachine::checkTransition_OnPad(const SensorData& data) {
    if (accelMagnitude(data.imu[0]) >= BOOST_ACCEL_THRESHOLD_MS2) {
        return FlightState::BOOST;
    }
    return FlightState::ON_PAD;
}

FlightState StateMachine::checkTransition_Boost(const SensorData& data) {
    if (accelMagnitude(data.imu[0]) <= BURNOUT_ACCEL_THRESHOLD_MS2) {
        burnoutConfirmCount_++;
    } else {
        burnoutConfirmCount_ = 0;
    }
    if (burnoutConfirmCount_ >= BURNOUT_CONFIRM_SAMPLES) {
        return FlightState::COAST_ONSET;
    }
    return FlightState::BOOST;
}

FlightState StateMachine::checkTransition_CoastOnset() {
    if (millis() - coastOnsetEntryMs_ >= static_cast<unsigned long>(COAST_TIMER_SECONDS * 1000)) {
        return FlightState::COAST;
    }
    return FlightState::COAST_ONSET;
}

FlightState StateMachine::checkTransition_Coast(const SensorData& data) {
    if (data.baro[0].altitude < previousAltitude_) {
        altitudeDecreasingCount_++;
    } else {
        altitudeDecreasingCount_ = 0;
    }
    if (altitudeDecreasingCount_ >= APOGEE_CONFIRM_SAMPLES) {
        return FlightState::RECOVERY;
    }
    return FlightState::COAST;
}

void StateMachine::onEnter_OnPad() {
    Serial.println("[STATE] ON_PAD: Waiting for launch. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
}

void StateMachine::onEnter_Boost(const SensorData& data) {
    Serial.println("[STATE] BOOST: Motor burning. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
    flushPreLaunchBuffer();
}

void StateMachine::onEnter_CoastOnset() {
    Serial.println("[STATE] COAST_ONSET: Burnout detected. Airbrakes permitted.");
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

void StateMachine::setAirbrakeStatus(AirbrakeStatus status) {
    Serial.print("[AIRBRAKE] Status set to: ");
    switch (status) {
        case AirbrakeStatus::LOCKED:      Serial.println("LOCKED");      break;
        case AirbrakeStatus::PERMITTED:   Serial.println("PERMITTED");   break;
        case AirbrakeStatus::ACTIVE_CONT: Serial.println("ACTIVE_CONT"); break;
    }
}

float StateMachine::accelMagnitude(const IMUData& imu) {
    return sqrtf(imu.accel.x * imu.accel.x +
                 imu.accel.y * imu.accel.y +
                 imu.accel.z * imu.accel.z);
}

void StateMachine::storePreLaunchSample(const SensorData& data) {
    preLaunchBuffer_[bufferHead_].data = data;
    bufferHead_ = (bufferHead_ + 1) % PRE_LAUNCH_BUFFER_SIZE;
    if (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) {
        bufferCount_++;
    }
}

void StateMachine::flushPreLaunchBuffer() {
    int start = (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) ? 0 : bufferHead_;
    for (int i = 0; i < bufferCount_; i++) {
        int idx = (start + i) % PRE_LAUNCH_BUFFER_SIZE;
        sdLog_.log(preLaunchBuffer_[idx].data);
    }
    Serial.print("[STATE] Flushed ");
    Serial.print(bufferCount_);
    Serial.println(" pre-launch samples to SD.");
}
```

- [ ] **Step 3: Commit**

```bash
git add src/state_machine.hpp src/state_machine.cpp
git commit -m "update state machine to use SensorData"
```

---

### Task 9: Rewrite main.cpp

Wire everything together. Add DEBUG_MODE routing.

**Files:**
- Rewrite: `src/main.cpp`

- [ ] **Step 1: Write `src/main.cpp`**

```cpp
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
TempSensor   tmp117;
sd_log       sdLog;
StateMachine stateMachine(sdLog);

// track which sensors initialized
uint8_t sensorStatus = 0;
constexpr uint8_t SENSOR_IMU1  = (1 << 0);
constexpr uint8_t SENSOR_IMU2  = (1 << 1);
constexpr uint8_t SENSOR_IMU3  = (1 << 2);
constexpr uint8_t SENSOR_IMU4  = (1 << 3);
constexpr uint8_t SENSOR_BARO1 = (1 << 4);
constexpr uint8_t SENSOR_BARO2 = (1 << 5);
constexpr uint8_t SENSOR_MAG   = (1 << 6);
constexpr uint8_t SENSOR_TMP   = (1 << 7);

void setup() {
    Serial.begin(115200);
    delay(500);

    SPI.begin();
    SPI1.begin();
    Wire.begin();
    Wire.setClock(400000);

    #ifdef DEBUG_MODE
    auto imuCfg = IMU::debugConfig;
    auto baroCfg = Barometer::debugConfig;
    #else
    auto imuCfg = IMU::flightConfig;
    auto baroCfg = Barometer::flightConfig;
    #endif

    if (imu1.init(imuCfg(IMU1_CS, &SPI)))       sensorStatus |= SENSOR_IMU1;
    else Serial.println("IMU1 init failed");

    if (imu2.init(imuCfg(IMU2_CS, &SPI)))        sensorStatus |= SENSOR_IMU2;
    else Serial.println("IMU2 init failed");

    if (imu3.init(imuCfg(IMU3_CS, &SPI1)))       sensorStatus |= SENSOR_IMU3;
    else Serial.println("IMU3 init failed");

    if (imu4.init(imuCfg(IMU4_CS, &SPI1)))       sensorStatus |= SENSOR_IMU4;
    else Serial.println("IMU4 init failed");

    if (baro1.init(baroCfg(BARO1_CS, &SPI)))     sensorStatus |= SENSOR_BARO1;
    else Serial.println("Baro1 init failed");

    if (baro2.init(baroCfg(BARO2_CS, &SPI1)))    sensorStatus |= SENSOR_BARO2;
    else Serial.println("Baro2 init failed");

    if (mag.init())                                sensorStatus |= SENSOR_MAG;
    else Serial.println("Magnetometer init failed");

    if (tmp117.init())                             sensorStatus |= SENSOR_TMP;
    else Serial.println("TMP117 init failed");

    if (!sdLog.init()) {
        Serial.println("SD card init failed");
    }

    Serial.print("Sensor status: 0x");
    Serial.println(sensorStatus, HEX);
    Serial.println("GNC-Airbrakes firmware initialized");
}

static SensorData readAllSensors() {
    SensorData data{};
    data.timestamp_us = micros();

    if (sensorStatus & SENSOR_IMU1) { imu1.update(); data.imu[0] = imu1.readAll(); }
    else data.imu[0] = nanIMU();

    if (sensorStatus & SENSOR_IMU2) { imu2.update(); data.imu[1] = imu2.readAll(); }
    else data.imu[1] = nanIMU();

    if (sensorStatus & SENSOR_IMU3) { imu3.update(); data.imu[2] = imu3.readAll(); }
    else data.imu[2] = nanIMU();

    if (sensorStatus & SENSOR_IMU4) { imu4.update(); data.imu[3] = imu4.readAll(); }
    else data.imu[3] = nanIMU();

    if (sensorStatus & SENSOR_BARO1) { baro1.update(); data.baro[0] = baro1.readAll(); }
    else data.baro[0] = nanBaro();

    if (sensorStatus & SENSOR_BARO2) { baro2.update(); data.baro[1] = baro2.readAll(); }
    else data.baro[1] = nanBaro();

    if (sensorStatus & SENSOR_MAG) { mag.update(); data.mag = mag.readAll(); }
    else data.mag = nanMag();

    if (sensorStatus & SENSOR_TMP) { tmp117.update(); data.tmp = tmp117.readAll(); }
    else data.tmp = nanTemp();

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
```

- [ ] **Step 2: Commit**

```bash
git add src/main.cpp
git commit -m "rewrite main.cpp for 8-sensor PCB"
```

---

### Task 10: Add debug mode serial output

Tagged serial protocol for the Python visualizer.

**Files:**
- Create: `src/debug_mode.hpp`
- Create: `src/debug_mode.cpp`

- [ ] **Step 1: Write `src/debug_mode.hpp`**

```cpp
#ifndef DEBUG_MODE_HPP
#define DEBUG_MODE_HPP

#include "sensor_data.hpp"

// print all sensor data as tagged CSV lines over serial
void debugPrint(const SensorData& data);

#endif // DEBUG_MODE_HPP
```

- [ ] **Step 2: Write `src/debug_mode.cpp`**

```cpp
#ifdef DEBUG_MODE

#include "debug_mode.hpp"
#include <Arduino.h>

static void printIMU(const char* tag, const IMUData& imu) {
    Serial.printf("%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
        tag,
        imu.accel.x, imu.accel.y, imu.accel.z,
        imu.gyro.x, imu.gyro.y, imu.gyro.z,
        imu.temp);
}

static void printBaro(const char* tag, const BarometerData& baro) {
    Serial.printf("%s,%.2f,%.1f,%.2f\n",
        tag,
        baro.temperature, baro.pressure, baro.altitude);
}

static void printMag(const char* tag, const MagData& m) {
    Serial.printf("%s,%.4f,%.4f,%.4f\n",
        tag,
        m.field.x, m.field.y, m.field.z);
}

static void printTemp(const char* tag, const TempData& t) {
    Serial.printf("%s,%.4f\n", tag, t.temperature);
}

void debugPrint(const SensorData& data) {
    printIMU("$IMU1", data.imu[0]);
    printIMU("$IMU2", data.imu[1]);
    printIMU("$IMU3", data.imu[2]);
    printIMU("$IMU4", data.imu[3]);
    printBaro("$BARO1", data.baro[0]);
    printBaro("$BARO2", data.baro[1]);
    printMag("$MAG", data.mag);
    printTemp("$TMP", data.tmp);
    Serial.printf("$TICK,%lu\n", data.timestamp_us);
}

#endif // DEBUG_MODE
```

- [ ] **Step 3: Commit**

```bash
git add src/debug_mode.hpp src/debug_mode.cpp
git commit -m "add debug mode tagged serial output"
```

---

### Task 11: Update Makefile with debug target

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Add debug target**

Add after the existing `build` target (around line 122):

```makefile
# Debug build: streams sensor data over serial instead of running flight state machine
.PHONY: debug
debug: DEFINES += -DDEBUG_MODE
debug: clean_src build
```

The `clean_src` dependency ensures source files are recompiled with the new define, since the Makefile doesn't track define changes in dependencies.

- [ ] **Step 2: Verify both build targets parse**

```bash
make build
make clean_src
make debug
```

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "add make debug target"
```

---

### Task 12: Create Python live visualizer

**Files:**
- Create: `tools/sensor_monitor.py`

- [ ] **Step 1: Write `tools/sensor_monitor.py`**

```python
#!/usr/bin/env python3
"""
GNC-Airbrakes sensor monitor.

Reads tagged serial data from Teensy DEBUG_MODE build and plots live sensor data.

Usage:
    python tools/sensor_monitor.py /dev/cu.usbmodem*
    python tools/sensor_monitor.py COM3              # Windows

Dependencies:
    pip install pyserial matplotlib
"""

import sys
import serial
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

BAUD = 115200
WINDOW = 1200  # samples (~5s at 240 Hz)

# ring buffers for each sensor stream
buffers = {
    '$IMU1':  {'ax': collections.deque(maxlen=WINDOW), 'ay': collections.deque(maxlen=WINDOW), 'az': collections.deque(maxlen=WINDOW),
               'gx': collections.deque(maxlen=WINDOW), 'gy': collections.deque(maxlen=WINDOW), 'gz': collections.deque(maxlen=WINDOW)},
    '$IMU2':  {'ax': collections.deque(maxlen=WINDOW), 'ay': collections.deque(maxlen=WINDOW), 'az': collections.deque(maxlen=WINDOW),
               'gx': collections.deque(maxlen=WINDOW), 'gy': collections.deque(maxlen=WINDOW), 'gz': collections.deque(maxlen=WINDOW)},
    '$IMU3':  {'ax': collections.deque(maxlen=WINDOW), 'ay': collections.deque(maxlen=WINDOW), 'az': collections.deque(maxlen=WINDOW),
               'gx': collections.deque(maxlen=WINDOW), 'gy': collections.deque(maxlen=WINDOW), 'gz': collections.deque(maxlen=WINDOW)},
    '$IMU4':  {'ax': collections.deque(maxlen=WINDOW), 'ay': collections.deque(maxlen=WINDOW), 'az': collections.deque(maxlen=WINDOW),
               'gx': collections.deque(maxlen=WINDOW), 'gy': collections.deque(maxlen=WINDOW), 'gz': collections.deque(maxlen=WINDOW)},
    '$BARO1': {'alt': collections.deque(maxlen=WINDOW)},
    '$BARO2': {'alt': collections.deque(maxlen=WINDOW)},
    '$MAG':   {'x': collections.deque(maxlen=WINDOW), 'y': collections.deque(maxlen=WINDOW), 'z': collections.deque(maxlen=WINDOW)},
    '$TMP':   {'temp': collections.deque(maxlen=WINDOW)},
}

def parse_line(line):
    """Parse a tagged serial line into its buffer."""
    if not line.startswith('$') or line.startswith('$TICK'):
        return
    parts = line.split(',')
    tag = parts[0]
    if tag not in buffers:
        return
    try:
        vals = [float(v) for v in parts[1:]]
    except ValueError:
        return

    if tag.startswith('$IMU') and len(vals) >= 6:
        buf = buffers[tag]
        buf['ax'].append(vals[0]); buf['ay'].append(vals[1]); buf['az'].append(vals[2])
        buf['gx'].append(vals[3]); buf['gy'].append(vals[4]); buf['gz'].append(vals[5])
    elif tag.startswith('$BARO') and len(vals) >= 3:
        buffers[tag]['alt'].append(vals[2])  # altitude is 3rd field
    elif tag == '$MAG' and len(vals) >= 3:
        buf = buffers[tag]
        buf['x'].append(vals[0]); buf['y'].append(vals[1]); buf['z'].append(vals[2])
    elif tag == '$TMP' and len(vals) >= 1:
        buffers[tag]['temp'].append(vals[0])

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port>")
        sys.exit(1)

    port = sys.argv[1]
    ser = serial.Serial(port, BAUD, timeout=0.01)

    fig, axes = plt.subplots(3, 4, figsize=(16, 9))
    fig.suptitle('GNC-Airbrakes Sensor Monitor', fontsize=14)
    plt.subplots_adjust(hspace=0.4, wspace=0.3)

    # row 0: accel (4 IMUs)
    accel_lines = {}
    for i, tag in enumerate(['$IMU1', '$IMU2', '$IMU3', '$IMU4']):
        ax = axes[0][i]
        ax.set_title(f'{tag[1:]} Accel (m/s^2)')
        ax.set_ylim(-100, 100)
        lx, = ax.plot([], [], 'r-', lw=0.8, label='X')
        ly, = ax.plot([], [], 'g-', lw=0.8, label='Y')
        lz, = ax.plot([], [], 'b-', lw=0.8, label='Z')
        ax.legend(loc='upper right', fontsize=7)
        accel_lines[tag] = (lx, ly, lz)

    # row 1: gyro (4 IMUs)
    gyro_lines = {}
    for i, tag in enumerate(['$IMU1', '$IMU2', '$IMU3', '$IMU4']):
        ax = axes[1][i]
        ax.set_title(f'{tag[1:]} Gyro (rad/s)')
        ax.set_ylim(-10, 10)
        lx, = ax.plot([], [], 'r-', lw=0.8, label='X')
        ly, = ax.plot([], [], 'g-', lw=0.8, label='Y')
        lz, = ax.plot([], [], 'b-', lw=0.8, label='Z')
        ax.legend(loc='upper right', fontsize=7)
        gyro_lines[tag] = (lx, ly, lz)

    # row 2: baro1 alt, baro2 alt, mag, tmp
    baro1_line, = axes[2][0].plot([], [], 'b-', lw=1)
    axes[2][0].set_title('Baro1 Alt (m)')
    axes[2][0].set_ylim(-10, 500)

    baro2_line, = axes[2][1].plot([], [], 'b-', lw=1)
    axes[2][1].set_title('Baro2 Alt (m)')
    axes[2][1].set_ylim(-10, 500)

    mag_lx, = axes[2][2].plot([], [], 'r-', lw=0.8, label='X')
    mag_ly, = axes[2][2].plot([], [], 'g-', lw=0.8, label='Y')
    mag_lz, = axes[2][2].plot([], [], 'b-', lw=0.8, label='Z')
    axes[2][2].set_title('Mag (Gauss)')
    axes[2][2].set_ylim(-8, 8)
    axes[2][2].legend(loc='upper right', fontsize=7)

    tmp_line, = axes[2][3].plot([], [], 'r-', lw=1)
    axes[2][3].set_title('TMP117 (C)')
    axes[2][3].set_ylim(0, 50)

    def animate(_frame):
        # drain serial buffer
        while ser.in_waiting:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                parse_line(line)
            except Exception:
                continue

        artists = []

        # update IMU plots
        for tag in ['$IMU1', '$IMU2', '$IMU3', '$IMU4']:
            buf = buffers[tag]
            lx, ly, lz = accel_lines[tag]
            lx.set_data(range(len(buf['ax'])), list(buf['ax']))
            ly.set_data(range(len(buf['ay'])), list(buf['ay']))
            lz.set_data(range(len(buf['az'])), list(buf['az']))
            artists.extend([lx, ly, lz])

            lx, ly, lz = gyro_lines[tag]
            lx.set_data(range(len(buf['gx'])), list(buf['gx']))
            ly.set_data(range(len(buf['gy'])), list(buf['gy']))
            lz.set_data(range(len(buf['gz'])), list(buf['gz']))
            artists.extend([lx, ly, lz])

        # baro
        b1 = buffers['$BARO1']['alt']
        baro1_line.set_data(range(len(b1)), list(b1))
        artists.append(baro1_line)

        b2 = buffers['$BARO2']['alt']
        baro2_line.set_data(range(len(b2)), list(b2))
        artists.append(baro2_line)

        # mag
        mb = buffers['$MAG']
        mag_lx.set_data(range(len(mb['x'])), list(mb['x']))
        mag_ly.set_data(range(len(mb['y'])), list(mb['y']))
        mag_lz.set_data(range(len(mb['z'])), list(mb['z']))
        artists.extend([mag_lx, mag_ly, mag_lz])

        # tmp
        tb = buffers['$TMP']['temp']
        tmp_line.set_data(range(len(tb)), list(tb))
        artists.append(tmp_line)

        # auto-scale x axes
        for row in axes:
            for ax in row:
                ax.set_xlim(0, WINDOW)
                ax.relim()

        return artists

    _ani = animation.FuncAnimation(fig, animate, interval=33, blit=False)
    plt.show()

if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Make executable**

```bash
chmod +x tools/sensor_monitor.py
```

- [ ] **Step 3: Commit**

```bash
git add tools/sensor_monitor.py
git commit -m "add python live sensor visualizer"
```

---

### Task 13: Full build verification

**Files:** None (verification only)

- [ ] **Step 1: Clean build**

```bash
make clean && make build
```

Fix any compile errors. Common issues:
- Missing library headers (check `libraries/` structure)
- SparkFun library API mismatches (check actual function signatures after cloning)
- Adafruit_LSM6DSV enum names may differ from spec (check actual header)

- [ ] **Step 2: Debug build**

```bash
make clean && make debug
```

Verify it compiles with `-DDEBUG_MODE`.

- [ ] **Step 3: Commit any fixes**

```bash
git add -u
git commit -m "fix build errors from integration"
```

- [ ] **Step 4: Verify CSV header is MATLAB-compatible**

Open `src/sd_log_file.cpp`, count the columns in the header string. Should be 38 data columns + 1 timestamp = 39 total. Each must be a valid MATLAB variable name (letters, digits, underscores only, starts with letter).
