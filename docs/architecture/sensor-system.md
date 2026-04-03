# Sensor System Architecture

Target: Teensy 4.1 (Cortex-M7 @ 600 MHz)

## Hardware Layout

### SPI0 (MOSI=11, MISO=12, SCK=13)
| Sensor | CS Pin | Class | Status |
|--------|--------|-------|--------|
| LSM6DSV16X IMU 1 | 37 | `IMU` | Working |
| LSM6DSV16X IMU 2 | 38 | `IMU` | Working |
| BMP581 Barometer 1 | 10 | `Barometer` | Working |

### SPI1 (MOSI=26, MISO=39, SCK=27)
Note: MISO is remapped from the default pin 1 to pin 39 (`SPI1.setMISO(39)`) to match PCB routing.

| Sensor | CS Pin | Class | Status |
|--------|--------|-------|--------|
| LSM6DSV16X IMU 3 | 28 | `IMU` | Not working (SPI1 issue on PCB) |
| LSM6DSV16X IMU 4 | 33 | `IMU` | Not working (SPI1 issue on PCB) |
| BMP581 Barometer 2 | 34 | `Barometer` | Not working (SPI1 issue on PCB) |

### I2C (Wire, SDA=18, SCL=19, 400 kHz)
| Sensor | Address | Class |
|--------|---------|-------|
| MMC5983MA Magnetometer | 0x30 | `Magnetometer` |
| TMP117 Temperature 1 | 0x48 | `TempSensor` |
| TMP117 Temperature 2 | 0x49 | `TempSensor` |

Pin definitions live in `src/pins.hpp`.

---

## Driver Class Interface

All sensor drivers follow the same pattern:

```cpp
bool init(const Config& config, uint8_t cs_pin, SPIClass* bus);  // SPI sensors
bool init();                                                       // I2C sensors
bool update();    // read new data from sensor into internal buffer; returns true on success
T    readAll();   // return cached data from last update()
```

`init()` returns `false` if the sensor doesn't respond. `update()` returns `false` on a read failure. The `readAll()` call always returns the last successfully read data — it never blocks.

### Flight and Debug Configs

Each driver class has static `const` config presets:

```cpp
static const IMUConfig       IMU::flightConfig;   // ±8g, 500dps, 240 Hz
static const IMUConfig       IMU::debugConfig;    // ±4g, 250dps, 60 Hz
static const BarometerConfig Barometer::flightConfig;  // 8x OSR, 50 Hz
static const BarometerConfig Barometer::debugConfig;   // 4x OSR, 25 Hz
```

`make build` uses `flightConfig`. `make debug` uses `debugConfig` (lower noise bandwidth, easier to read over serial).

---

## SensorData Struct

`src/sensor_data.hpp` defines the unified data struct passed everywhere:

```cpp
struct SensorData {
    IMUData        imu[4];        // 4x LSM6DSV16X
    BarometerData  baro[2];       // 2x BMP581
    MagData        mag;           // 1x MMC5983MA
    TempData       tmp[2];        // 2x TMP117
    unsigned long  timestamp_us;  // micros() at time of readAllSensors()
};
```

Individual data structs (with byte sizes for SIM.BIN reference):

```cpp
struct IMUData {          // 28 bytes
    Vec3  accel;          //  0: x/y/z in m/s^2
    Vec3  gyro;           // 12: x/y/z in rad/s
    float temp;           // 24: on-die temperature in °C
};

struct BarometerData {    // 12 bytes
    float temperature;    //  0: °C
    float pressure;       //  4: Pa
    float altitude;       //  8: m (barometric formula, relative to sea_level_hpa)
};

struct MagData {          // 12 bytes
    Vec3 field;           //  0: x/y/z in Gauss
};

struct TempData {         //  4 bytes
    float temperature;    //  0: °C
};
```

`Vec3` is `{float x, y, z}` — 12 bytes, no padding.

---

## NaN Convention

If a sensor fails `init()` or `update()`, its slot is filled with `nanIMU()` / `nanBaro()` / `nanMag()` / `nanTemp()` (all fields set to `NAN`). This propagates cleanly through the state machine and any future Kalman filter.

Downstream code should check `std::isnan(data.imu[i].accel.x)` before using a slot. The state machine currently only reads `imu[0]` and `baro[0]` and doesn't check for NaN (safe because IMU1 and Baro1 are on SPI0 which works).

---

## sensorStatus Bitmask

`main.cpp` tracks which sensors initialized successfully:

```cpp
uint16_t sensorStatus = 0;
// Bits: 0=IMU1, 1=IMU2, 2=IMU3, 3=IMU4, 4=Baro1, 5=Baro2, 6=Mag, 7=Tmp1, 8=Tmp2
```

`readAllSensors()` in `main.cpp` checks each bit before calling `update()` on a sensor. Sensors that didn't init are skipped and NaN-filled. On startup, `sensorStatus` is printed over serial as hex — e.g., `0x033` means IMU1+2 and Baro1 are up (bits 0,1,4 = 0b000110011).

---

## Data Flow

```
Hardware (SPI/I2C)
    ↓
sensor.update()         (one call per sensor per loop tick)
    ↓
sensor.readAll()        (cached, no hardware access)
    ↓
SensorData              (assembled in readAllSensors())
    ↓
stateMachine.update(data)
    ├── reads imu[0].accel magnitude for BOOST/BURNOUT detection
    └── reads baro[0].altitude for APOGEE detection
```

For future state estimator integration, a filter step will sit between `readAllSensors()` and `stateMachine.update()`, consuming all active IMU and baro channels.

---

## Build Modes

| Command | Defines | Behavior |
|---------|---------|----------|
| `make build` | `FIRMWARE` | Normal flight firmware |
| `make debug` | `FIRMWARE` + `DEBUG_MODE` | Streams tagged CSV over serial instead of running state machine |
| `make diagnostic` | `FIRMWARE` + `DIAGNOSTIC_MODE` | Probes each sensor individually |
| `make sim` | `FIRMWARE` + `SIM_MODE` | Replays SIM.BIN from SD card instead of reading real sensors |

See `docs/architecture/sim-mode.md` for details on sim mode.
