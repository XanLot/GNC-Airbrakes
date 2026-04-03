# Sim Mode

Sim mode replays a pre-generated flight profile from a binary file on the SD card (`SIM.BIN`) instead of reading real sensors. The state machine receives the same `SensorData` struct it would in a real flight — no code changes between sim and real builds.

## Quick Start

```bash
# 1. Generate SIM.BIN in MATLAB
matlab -r "run('tools/generate_sim_data.m')"

# 2. Copy SIM.BIN to the root of the SD card

# 3. Build and upload
make sim
make upload   # or: make sim && make upload
make monitor  # watch the state machine run
```

On startup, the firmware prints `SIM.BIN opened: N frames` and then runs the state machine at 225 Hz (4 ms `delay()` per tick). When the profile is exhausted it prints `=== SIM_MODE: profile complete ===` and halts.

---

## SIM.BIN Binary Format

All values are IEEE 754 little-endian. No padding between fields.

### Header (16 bytes)

| Offset | Type | Field | Description |
|--------|------|-------|-------------|
| 0 | uint32 | magic | `0x424D4953` ("SIMB" in ASCII, little-endian) |
| 4 | uint32 | count | Number of frames |
| 8 | float32 | rate | Sample rate in Hz |
| 12 | uint8 | num_imus | Active IMU channels (1, 2, or 4) |
| 13 | uint8 | num_baros | Active baro channels (1 or 2) |
| 14 | uint8 | has_mag | 0 or 1 |
| 15 | uint8 | num_temps | Active temp channels (0, 1, or 2) |

### Per-Frame Layout

Frame stride = `num_imus*28 + num_baros*12 + has_mag*12 + num_temps*4` bytes.

Random access: `file_offset = 16 + tick * frame_stride`

| Slot | Size | Field order (all float32) |
|------|------|--------------------------|
| IMUData × num_imus | 28 bytes each | accel.x/y/z (m/s²), gyro.x/y/z (rad/s), temp (°C) |
| BarometerData × num_baros | 12 bytes each | temperature (°C), pressure (Pa), altitude (m) |
| MagData (if has_mag=1) | 12 bytes | field.x/y/z (Gauss) |
| TempData × num_temps | 4 bytes each | temperature (°C) |

Field order matches the C++ struct memory layout in `src/sensor_data.hpp` — the firmware reads these with direct `simFile.read(&d.imu[i], sizeof(IMUData))` calls.

**Inactive sensor slots** (e.g., `imu[2]` and `imu[3]` when `num_imus=2`) are NaN-filled by `sim_data.cpp`, matching the behavior of a failed hardware init.

---

## How the Firmware Uses SIM.BIN

`src/sim_data.cpp` (compiled only when `-DSIM_MODE` is defined):

- `simInit()`: Opens `SIM.BIN`, reads the 16-byte header, computes `frameStride`.
- `getSimData(tick)`: Returns a full `SensorData` for the given tick. Uses a one-frame cache — only one SD read per loop iteration even though the state machine accesses multiple sensor slots.
- Seeks only when `tick != lastTick + 1` (rare — normally reads sequentially).

`main.cpp` in SIM_MODE:
- Skips all sensor globals (`imu1`, `imu2`, ...) and sensor init code entirely.
- SD card is still initialized (`sdLog.init()`) since `simInit()` needs `SD.begin()` to have been called.
- Loop replaces `readAllSensors()` with `getSimData(simTick++)` + `delay(4)`.

---

## Generating SIM.BIN

`tools/generate_sim_data.m` takes an OpenRocket CSV export and produces `SIM.BIN`.

**Pipeline:**
1. Load OpenRocket CSV (15 columns — see script header for required column order)
2. Compute body-frame specific force from trajectory (removes gravity, rotates by zenith angle)
3. Add per-channel bias (constant offset drawn from uniform distribution)
4. Add white noise (independent RNG seed per channel for uncorrelated sensor noise)
5. Apply LP filter model (2nd-order Butterworth at ODR/4 Hz, approximating LSM6DSV16X LP2/LP1)
6. Quantize to 16-bit ADC (LSM6DSV16X sensitivity: 0.244 mg/LSB at ±8g, 17.50 mdps/LSB at ±500dps)
7. Downsample to sensor ODR (240 Hz IMU, 50 Hz baro, resampled to common IMU tick rate)
8. Write SIM.BIN

**Noise model (LSM6DSV16X, Table 3 of DS13510 Rev 4):**
- Accel noise density: 60 µg/√Hz (high-performance mode)
- Accel zero-g offset: ±12 mg
- Gyro noise density: 2.8 mdps/√Hz (high-performance mode)
- Gyro zero-rate level: ±1 dps

**Key config at the top of the script:**
```matlab
num_imus  = 2;   % match what's working on the board (SPI1 not yet functional)
num_baros = 1;
has_mag   = 0;
num_temps = 0;
```

The script also saves `truth_data.mat` with the ground-truth trajectory at IMU rate — used for validating a future Kalman filter (compare estimated state vs. truth).

---

## Variable Sensor Counts

The `num_imus` field in the header lets you generate SIM.BIN with fewer sensors than the board has. This is useful because:

- SPI1 (IMU3/4, Baro2) is currently non-functional on the PCB. Generating with `num_imus=2` exactly mirrors the real hardware behavior — `imu[2]` and `imu[3]` will be NaN in both sim and real builds.
- For state estimator development, you can test degraded-mode performance by generating with `num_imus=1` and verifying the filter handles a single-channel input.

---

## Timing

`delay(4)` in the sim loop targets 225 Hz (1000ms / 4ms = 250 Hz, close enough). The state machine uses `millis()` for COAST_ONSET timer and burnout confirmation, so this timing matters. If the IMU ODR ever changes from 240 Hz, update the `delay()` call in `main.cpp` accordingly.

---

## Future: Kalman Filter Integration

Sim mode is the primary validation tool for the state estimator. The workflow will be:

1. Tune filter parameters offline in MATLAB using `truth_data.mat` as ground truth
2. Implement the filter in firmware (`src/state_estimator.cpp`)
3. Insert the estimator between `readAllSensors()` / `getSimData()` and `stateMachine.update()`
4. Run `make sim` to verify the estimator converges correctly on the known flight profile
5. The state machine will eventually consume estimated state (velocity, altitude) rather than raw `SensorData`
