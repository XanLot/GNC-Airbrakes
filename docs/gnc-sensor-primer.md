# GNC Sensor Primer for Rocket Airbrakes
*A practical guide to what the sensors are doing and what good data looks like*

---

## What Are We Actually Measuring?

The airbrake GNC system needs to answer two questions in real time:
1. **How fast is the rocket going?** (to know how much drag to add)
2. **Where is the rocket?** (to know if we're on track for the target apogee)

We have two sensors to answer these:
- **ICM-20948 IMU** — measures acceleration and rotation rate
- **BMP388 Barometer** — measures air pressure, which we convert to altitude

Neither sensor directly gives us velocity or position. We have to integrate and combine
them. That's the job of the state estimator (future work).

---

## The ICM-20948 IMU

### What it contains
The ICM-20948 is actually three sensors in one chip:
- **Accelerometer** — measures specific force (acceleration minus gravity)
- **Gyroscope** — measures rotation rate around each axis
- **Magnetometer** (AK09916) — measures magnetic field direction

For airbrake GNC, accelerometer and gyro are the critical ones. Magnetometer helps
with attitude estimation but is secondary.

### What the accelerometer actually measures

This is the most important thing to understand: **the accelerometer does NOT measure
velocity or position directly — it measures specific force.**

Specific force = (total force on the sensor) / mass = acceleration - gravity

What this means practically:
- A rocket sitting still on the pad reads **~9.81 m/s² upward** (1g), because the
  ground is pushing up and gravity is pulling down. This is correct behavior.
- During boost, the accelerometer reads the net thrust minus drag, not the total speed.
- At burnout, when thrust drops to zero, it reads near 0g — just aerodynamic drag.
- This is why the BOOST → COAST_ONSET transition uses `‖accel‖ <= 0g` — it detects
  when the motor stops contributing.

### What good IMU data looks like

**On the pad (healthy sensor):**
```
Accel: x ≈ 0, y ≈ 0, z ≈ +9.81 m/s²   (1g upward, assuming z is up)
Gyro:  x ≈ 0, y ≈ 0, z ≈ 0             (no rotation)
```
Any gyro bias of more than ~0.1 rad/s at rest is worth noting — that's a calibration target.

**During boost (healthy sensor):**
```
Accel magnitude: 5g–7g (49–69 m/s²)
Gyro: small values unless rocket is rotating — watch for spin rates > 5°/s
```
If you see accel magnitude spiking wildly (e.g., 50g then 0g in adjacent samples),
that's a vibration aliasing problem — the LPF bandwidth needs to be reduced.

**At burnout (healthy sensor):**
```
Accel magnitude: drops to ~0g or slightly negative (drag only)
This transition should be smooth, not a step function
```
A sudden noisy drop to near 0 followed by oscillation means the LPF was too wide.

**During coast (healthy sensor):**
```
Accel: small negative value (drag decelerating the rocket)
Gyro: near zero (rocket should be flying stable)
```

### Vibration: the main enemy

Rocket motors vibrate at high frequencies (100s of Hz). The accelerometer samples these
as if they were real acceleration. If the sampling rate is 1125 Hz but the LPF cutoff
is 246 Hz, vibrations between 246 Hz and 562 Hz (Nyquist) get **aliased** — folded down
into lower frequencies that look like real acceleration.

This is why we enable the on-chip digital low-pass filter (DLPF) at 23.9 Hz. Any
vibration above ~12 Hz gets attenuated. The trade-off is that real acceleration events
faster than 23.9 Hz also get smoothed, but airbrake actuation is slow enough that this
doesn't matter.

**Rule of thumb:** If you see accel data that looks noisy at rest but smooth during
a motor burn, that's likely vibration aliasing. Tighten the LPF.

### Gyro for attitude estimation

For the airbrakes, we primarily care about whether the rocket is flying straight.
The gyro measures rotation rate (rad/s). If you integrate it over time you get angle.

**What good gyro data looks like:**
- At rest: < 0.05 rad/s on all axes (any more = bias, needs calibration)
- During flight: mostly small values (< 0.5 rad/s) unless the rocket is spinning or
  correcting its trajectory
- Large sustained gyro readings during coast means the rocket is tumbling — that's bad
  and the airbrakes probably can't help at that point

---

## The BMP388 Barometer

### How pressure becomes altitude

The BMP388 measures air pressure in Pascals. Pressure decreases with altitude in a
predictable way described by the barometric formula:

```
altitude = 44330 * (1 - (pressure / sea_level_pressure)^0.1903)
```

This is what `barometer.cpp` already uses. The key input is `sea_level_pressure` —
if you get this wrong, all your altitudes will be offset. Set it fresh on the pad
before each flight using the average of your pre-launch buffer readings.

### What good barometer data looks like

**On the pad:**
```
Pressure: depends on your launch site altitude.
  At sea level: ~101325 Pa (1013.25 hPa)
  At 5000 ft (e.g., Colorado): ~84000 Pa (~840 hPa)
  At 8000 ft: ~75000 Pa (~750 hPa)
Altitude: should read ~0 m AGL if sea level reference is set correctly
Temperature: ambient air temp in Celsius
```

**During ascent:**
```
Pressure: decreasing steadily
Altitude: increasing ~150–300 m/s depending on rocket speed
```
If altitude is jumping around by more than ±5m between readings at 50 Hz, the IIR
filter coefficient is too low — increase it.

**At apogee:**
```
Altitude: reaches maximum value and starts decreasing
This is the COAST → RECOVERY transition trigger
```
Important: use a rolling average (e.g., compare current altitude to the average of
the last 5 readings) to detect "altitude starts decreasing" — a single noisy sample
below the peak is not a reliable trigger.

**During descent:**
```
Pressure: increasing
Altitude: decreasing
```

### Barometer limitations

**Latency:** Oversampling takes time. At 8x pressure oversampling, each reading takes
~10 ms. At 50 Hz (20 ms between readings), this leaves 10 ms of margin. Don't combine
high oversampling with too-high ODR or the sensor won't keep up.

**Turbulence noise:** The pressure port on the sensor is exposed to airflow. At high
speeds, dynamic pressure (ram pressure from moving air) adds to the static pressure
reading, making the altitude read lower than actual. This effect is small at typical
amateur rocket speeds but worth knowing about. The IIR filter helps here.

**Temperature compensation:** The BMP388 applies internal temperature compensation to
pressure readings automatically. The temperature reading itself is not primary data for
GNC — it's mostly for the compensation calculation.

---

## Sensor Fusion: Why We Need Both

The IMU alone has **drift** — integrating noisy accelerometer data accumulates error
over time. After 10 seconds of flight, the integrated position can be off by tens of meters.

The barometer alone has **noise and latency** — it gives absolute altitude but with
some lag and noise, and can't tell you velocity directly.

Together:
- Barometer gives absolute altitude reference (prevents IMU drift from accumulating)
- IMU gives high-rate velocity/acceleration data between barometer readings
- A Kalman filter or complementary filter fuses them to get the best of both

This is the state estimator that will be built in a future sprint. The state machine
and sensor configs in this sprint lay the groundwork for it.

---

## Quick Reference: "Is This Reading Healthy?"

| Sensor | Reading | Healthy | Suspicious |
|--------|---------|---------|------------|
| Accel | At-rest magnitude | 9.5–10.1 m/s² | < 9 or > 10.5 |
| Accel | Boost peak | 49–75 m/s² (5–7.5g) | > 90 m/s² or noisy |
| Accel | At burnout | < 5 m/s² and falling | Stays high or spikes |
| Gyro | At rest | < 0.05 rad/s all axes | > 0.1 rad/s |
| Gyro | During flight | < 0.5 rad/s | > 2 rad/s (tumbling?) |
| Baro pressure | On pad (CO) | 75000–90000 Pa | Outside this range |
| Baro altitude | During ascent | Increasing smoothly | Jumping > ±5 m/sample |
| Baro altitude | At apogee | Reverses direction | Never reverses |

---

## Further Reading

- **Invensense ICM-20948 Datasheet** — register map with all DLPF, ODR, range settings
- **Bosch BMP388 Datasheet** — oversampling, IIR filter, and ODR interaction tables
- **"Fundamentals of Kalman Filtering"** — Roger Labbe's free online book (great intro)
- **OpenRocket** — simulate your rocket's expected altitude and acceleration profile
  to sanity-check sensor readings against expected values
