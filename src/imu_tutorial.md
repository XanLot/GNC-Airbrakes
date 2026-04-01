# IMU Module Tutorial

This guide will help you understand how to use the IMU (Inertial Measurement Unit) module in the GNC-Airbrakes project.

## What is an IMU?

An IMU is a sensor that measures:
- **Gyroscope** - How fast the rocket is rotating (degrees per second)
- **Accelerometer** - Forces acting on the rocket including gravity (in g's)
- **Magnetometer** - Magnetic field direction, like a compass (in microtesla)
- **Quaternion** - The rocket's orientation in 3D space

## Quick Start

```cpp
#include "imu.hpp"

IMU imu;

void setup() {
    imu.init(IMU::defaultConfig());
}

void loop() {
    imu.update();  // Always call this first!

    if (imu.accelReady()) {
        Vec3 accel = imu.readAccel();
        Serial.println(accel.z);  // Print vertical acceleration
    }
}
```

## Step-by-Step Guide

### 1. Include the Header

At the top of your file, add:

```cpp
#include "imu.hpp"
```

### 2. Create an IMU Object

Create a global IMU variable:

```cpp
IMU imu;
```

### 3. Initialize in setup()

Call `init()` with a configuration:

```cpp
void setup() {
    Serial.begin(115200);
    imu.init(IMU::defaultConfig());
}
```

### 4. Update in loop()

You **must** call `update()` frequently to get new sensor data:

```cpp
void loop() {
    imu.update();
    // ... rest of your code
}
```

### 5. Read Sensor Data

Check if data is ready, then read it:

```cpp
if (imu.gyroReady()) {
    Vec3 gyro = imu.readGyro();
    // Use gyro.x, gyro.y, gyro.z
}
```

## Data Types

### Vec3

A simple 3D vector with x, y, z components:

```cpp
Vec3 accel = imu.readAccel();
float vertical = accel.z;    // Z points up/down
float forward = accel.x;     // X points forward
float sideways = accel.y;    // Y points sideways
```

### Quaternion

Represents 3D rotation with w, x, y, z components:

```cpp
Quaternion quat = imu.readQuat();
// quat.w, quat.x, quat.y, quat.z
```

### IMUData

Contains all sensor readings at once:

```cpp
IMUData data = imu.readAll();
data.accel.x;   // Accelerometer X
data.gyro.y;    // Gyroscope Y
data.mag.z;     // Magnetometer Z
data.quat.w;    // Quaternion W
```

## Available Functions

| Function | Returns | Description |
|----------|---------|-------------|
| `init(config)` | void | Initialize the IMU |
| `update()` | void | Poll for new data (call every loop) |
| `gyroReady()` | bool | Is new gyro data available? |
| `accelReady()` | bool | Is new accel data available? |
| `magReady()` | bool | Is new mag data available? |
| `quatReady()` | bool | Is new quaternion data available? |
| `readGyro()` | Vec3 | Get gyroscope reading (deg/s) |
| `readAccel()` | Vec3 | Get accelerometer reading (g) |
| `readMag()` | Vec3 | Get magnetometer reading (uT) |
| `readQuat()` | Quaternion | Get orientation |
| `readAll()` | IMUData | Get all sensor data at once |

## Custom Configuration

You can customize the IMU settings:

```cpp
IMUConfig config;
config.cs_pin = 24;                   // SPI chip select pin
config.spi_speed = 1000000;           // 1 MHz
config.mode = 1;                      // 1 = high performance
config.enable_gyroscope = true;
config.enable_accelerometer = true;
config.enable_magnetometer = true;
config.enable_quaternion = true;
config.gyroscope_frequency = 100;     // Hz (1-225)
config.accelerometer_frequency = 100; // Hz (1-225)
config.magnetometer_frequency = 50;   // Hz (1-70)
config.quaternion_frequency = 50;     // Hz (50-225)

imu.init(config);
```

## Common Patterns

### Detect Launch

```cpp
void loop() {
    imu.update();

    if (imu.accelReady()) {
        Vec3 accel = imu.readAccel();
        float total = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);

        if (total > 3.0) {
            Serial.println("Launch detected!");
        }
    }
}
```

### Log All Data

```cpp
void loop() {
    imu.update();

    IMUData data = imu.readAll();

    Serial.print(data.accel.x); Serial.print(",");
    Serial.print(data.accel.y); Serial.print(",");
    Serial.println(data.accel.z);
}
```

## Troubleshooting

**No data coming in?**
- Make sure you're calling `imu.update()` in your loop
- Check that `init()` was called in setup
- Verify SPI wiring matches the cs_pin in your config

**Data looks wrong?**
- Accelerometer reads ~1g on Z axis when stationary (that's gravity!)
- Gyro should read near zero when not moving
- Make sure you're checking `*Ready()` before reading
