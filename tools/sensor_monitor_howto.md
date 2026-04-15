# sensor_monitor usage

Live plots all sensor channels from the Teensy over serial.

## 1. Build and flash debug firmware

```sh
make debug
make upload
```

The debug build streams tagged CSV at ~240 Hz instead of running the state machine.
Format: `$IMU1,ax,ay,az,gx,gy,gz,temp`, `$BARO1,temp,pressure,altitude`, `$MAG,x,y,z`, `$TMP1,temp`

## 2. Run the monitor

```sh
visualiser/.venv/bin/python tools/sensor_monitor.py /dev/cu.usbmodem*
```

On Windows: replace `/dev/cu.usbmodem*` with `COM3` (or whatever port Device Manager shows).

The window shows a 3×4 grid:
- Row 0: accel XYZ for each IMU (m/s²)
- Row 1: gyro XYZ for each IMU (rad/s)
- Row 2: baro altitude, baro altitude, magnetometer XYZ, temperature (both TMP117s)

IMU3/4 and Baro2 will be flat/NaN if the SPI1 hardware issue isn't fixed yet.

## 3. Venv setup (first time only)

```sh
python3 -m venv visualiser/.venv
visualiser/.venv/bin/pip install pyserial matplotlib
```
