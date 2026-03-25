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
        buffers[tag]['alt'].append(vals[2])
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
        while ser.in_waiting:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                parse_line(line)
            except Exception:
                continue

        artists = []

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

        b1 = buffers['$BARO1']['alt']
        baro1_line.set_data(range(len(b1)), list(b1))
        artists.append(baro1_line)

        b2 = buffers['$BARO2']['alt']
        baro2_line.set_data(range(len(b2)), list(b2))
        artists.append(baro2_line)

        mb = buffers['$MAG']
        mag_lx.set_data(range(len(mb['x'])), list(mb['x']))
        mag_ly.set_data(range(len(mb['y'])), list(mb['y']))
        mag_lz.set_data(range(len(mb['z'])), list(mb['z']))
        artists.extend([mag_lx, mag_ly, mag_lz])

        tb = buffers['$TMP']['temp']
        tmp_line.set_data(range(len(tb)), list(tb))
        artists.append(tmp_line)

        for row in axes:
            for ax in row:
                ax.set_xlim(0, WINDOW)
                ax.relim()

        return artists

    _ani = animation.FuncAnimation(fig, animate, interval=33, blit=False)
    plt.show()


if __name__ == '__main__':
    main()
