#!/usr/bin/env python3
"""
GNC-Airbrakes Sensor Visualiser
Reads Teensy serial output and displays 3D orientation + sensor stats.
"""

import sys
import re
import threading
import time
import math

import serial
import serial.tools.list_ports
import pygame
import pygame.freetype
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# ── Window layout ──────────────────────────────────────────────────────────────
WINDOW_W = 1200
WINDOW_H = 700
PANEL_X = 700  # left edge of stats panel / width of 3D viewport


# ── Sensor data ────────────────────────────────────────────────────────────────
class SensorData:
    """Thread-safe store for the latest sensor readings."""

    def __init__(self):
        self.lock = threading.Lock()
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.quat = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.temperature = 0.0
        self.pressure = 0.0
        self.altitude = 0.0
        self.connected = False
        self.last_update = 0.0


# ── Serial reader ─────────────────────────────────────────────────────────────
class SerialReader:
    """Background thread that reads and parses Teensy serial output."""

    PATTERNS = {
        "gyro": re.compile(r"Gyro \(deg/s\): \[([^,]+),([^,]+),([^\]]+)\]"),
        "accel": re.compile(r"Accel \(g\): \[([^,]+),([^,]+),([^\]]+)\]"),
        "mag": re.compile(r"Mag \(uT\): \[([^,]+),([^,]+),([^\]]+)\]"),
        "quat": re.compile(r"Quat: \[([^,]+),([^,]+),([^,]+),([^\]]+)\]"),
        "temp": re.compile(r"Temp \(C\): ([0-9.\-]+)"),
        "pressure": re.compile(r"Pressure \(hPa\): ([0-9.\-]+)"),
        "altitude": re.compile(r"Altitude \(m\): ([0-9.\-]+)"),
    }

    def __init__(self, port, baud, data):
        self.port = port
        self.baud = baud
        self.data = data
        self.running = False
        self._thread = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self):
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    with self.data.lock:
                        self.data.connected = True
                    while self.running:
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self._parse(line)
            except (serial.SerialException, OSError):
                with self.data.lock:
                    self.data.connected = False
                if self.running:
                    time.sleep(1)

    def _parse(self, line):
        with self.data.lock:
            self.data.last_update = time.time()

            for key, pat in self.PATTERNS.items():
                m = pat.search(line)
                if not m:
                    continue
                if key == "gyro":
                    self.data.gyro = [float(m.group(i)) for i in (1, 2, 3)]
                elif key == "accel":
                    self.data.accel = [float(m.group(i)) for i in (1, 2, 3)]
                elif key == "mag":
                    self.data.mag = [float(m.group(i)) for i in (1, 2, 3)]
                elif key == "quat":
                    self.data.quat = [float(m.group(i)) for i in (1, 2, 3, 4)]
                elif key == "temp":
                    self.data.temperature = float(m.group(1))
                elif key == "pressure":
                    self.data.pressure = float(m.group(1))
                elif key == "altitude":
                    self.data.altitude = float(m.group(1))
                return


# ── 3D helpers ─────────────────────────────────────────────────────────────────
def quat_to_matrix(w, x, y, z):
    """Quaternion to 4x4 column-major OpenGL rotation matrix.

    Remaps from IMU coordinates (Z-up) to OpenGL coordinates (Y-up):
      IMU X -> GL X,  IMU Y -> GL -Z,  IMU Z -> GL Y
    """
    # Remap quaternion components for coordinate system change
    x, y, z = x, z, -y

    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-10:
        return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    w, x, y, z = w / n, x / n, y / n, z / n

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return [
        1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy), 0,
        2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx), 0,
        2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy), 0,
        0, 0, 0, 1,
    ]


def draw_rocket():
    """Draw a rocket shape at the origin, nose pointing +Y."""
    body_r, body_h, nose_h, fin_sz, seg = 0.3, 2.0, 0.8, 0.5, 24

    # Body cylinder
    glColor3f(0.8, 0.8, 0.85)
    glBegin(GL_QUAD_STRIP)
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg
        cx, cz = body_r * math.cos(a), body_r * math.sin(a)
        glNormal3f(math.cos(a), 0, math.sin(a))
        glVertex3f(cx, -body_h / 2, cz)
        glVertex3f(cx, body_h / 2, cz)
    glEnd()

    # Nose cone
    glColor3f(0.9, 0.3, 0.2)
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, 1, 0)
    glVertex3f(0, body_h / 2 + nose_h, 0)
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg
        cx, cz = body_r * math.cos(a), body_r * math.sin(a)
        glNormal3f(math.cos(a) * 0.7, 0.7, math.sin(a) * 0.7)
        glVertex3f(cx, body_h / 2, cz)
    glEnd()

    # Bottom cap
    glColor3f(0.3, 0.3, 0.35)
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, -1, 0)
    glVertex3f(0, -body_h / 2, 0)
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg
        glVertex3f(body_r * math.cos(a), -body_h / 2, body_r * math.sin(a))
    glEnd()

    # Four fins
    glColor3f(0.2, 0.5, 0.9)
    for fa in (0, 90, 180, 270):
        glPushMatrix()
        glRotatef(fa, 0, 1, 0)
        glBegin(GL_TRIANGLES)
        glNormal3f(0, 0, 1)
        glVertex3f(body_r, -body_h / 2, 0)
        glVertex3f(body_r + fin_sz, -body_h / 2, 0)
        glVertex3f(body_r, -body_h / 2 + fin_sz * 1.5, 0)
        glNormal3f(0, 0, -1)
        glVertex3f(body_r, -body_h / 2, 0)
        glVertex3f(body_r, -body_h / 2 + fin_sz * 1.5, 0)
        glVertex3f(body_r + fin_sz, -body_h / 2, 0)
        glEnd()
        glPopMatrix()


def draw_axes(length=1.5):
    """Draw RGB XYZ axis lines."""
    glLineWidth(2)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(length, 0, 0)
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, length, 0)
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, length)
    glEnd()
    glLineWidth(1)


def draw_grid():
    """Reference grid on the XZ plane at y=-3."""
    glColor3f(0.25, 0.25, 0.3)
    glBegin(GL_LINES)
    for i in range(-5, 6):
        glVertex3f(i, -3, -5); glVertex3f(i, -3, 5)
        glVertex3f(-5, -3, i); glVertex3f(5, -3, i)
    glEnd()


# ── Port detection ─────────────────────────────────────────────────────────────
def find_teensy():
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").lower()
        m = (p.manufacturer or "").lower()
        if "teensy" in d or "teensy" in m or "usbmodem" in p.device.lower():
            return p.device
    return None


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    port = None
    baud = 115200

    # Parse CLI args
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] in ("-p", "--port") and i + 1 < len(args):
            port = args[i + 1]; i += 2
        elif args[i] in ("-b", "--baud") and i + 1 < len(args):
            baud = int(args[i + 1]); i += 2
        elif args[i] in ("-h", "--help"):
            print("Usage: visualiser.py [-p PORT] [-b BAUD]")
            sys.exit(0)
        else:
            port = args[i]; i += 1

    if not port:
        port = find_teensy()
        if port:
            print(f"Auto-detected Teensy on {port}")
        else:
            print("No Teensy found. Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device} -- {p.description}")
            sys.exit(1)

    # ── Pygame + OpenGL init ───────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("GNC-Airbrakes Visualiser")

    font_hdr = pygame.freetype.SysFont("monospace", 22)
    font_hdr.strong = True
    font_lbl = pygame.freetype.SysFont("monospace", 16)
    font_val = pygame.freetype.SysFont("monospace", 18)
    font_val.strong = True
    font_sm  = pygame.freetype.SysFont("monospace", 14)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_NORMALIZE)

    # Lighting
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glLightfv(GL_LIGHT0, GL_POSITION, [5, 10, 5, 0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1])
    glLightfv(GL_LIGHT1, GL_POSITION, [-5, 5, -3, 0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.3, 0.3, 0.4, 1])
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # Pre-allocate panel texture (reused every frame)
    panel_w = WINDOW_W - PANEL_X
    panel_tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, panel_tex)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA, panel_w, WINDOW_H, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, None,
    )

    # ── Serial reader ──────────────────────────────────────────────────────────
    data = SensorData()
    reader = SerialReader(port, baud, data)
    reader.start()

    clock = pygame.time.Clock()
    running = True

    while running:
        for ev in pygame.event.get():
            if ev.type == QUIT:
                running = False
            elif ev.type == KEYDOWN and ev.key in (K_ESCAPE, K_q):
                running = False

        # Snapshot sensor data under lock
        with data.lock:
            gyro  = list(data.gyro)
            accel = list(data.accel)
            mag   = list(data.mag)
            quat  = list(data.quat)
            temp  = data.temperature
            pres  = data.pressure
            alt   = data.altitude
            conn  = data.connected
            last  = data.last_update

        # ── 3D viewport (left) ─────────────────────────────────────────────────
        glViewport(0, 0, PANEL_X, WINDOW_H)
        glClearColor(0.12, 0.12, 0.15, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, PANEL_X / WINDOW_H, 0.1, 100)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(6, 4, 6, 0, 0, 0, 0, 1, 0)

        # World reference
        glDisable(GL_LIGHTING)
        draw_grid()
        draw_axes()
        glEnable(GL_LIGHTING)

        # Rocket with quaternion rotation
        glPushMatrix()
        glMultMatrixf(quat_to_matrix(*quat))
        draw_rocket()
        glDisable(GL_LIGHTING)
        draw_axes(2.0)
        glEnable(GL_LIGHTING)
        glPopMatrix()

        # ── Stats panel (right, rendered as texture) ───────────────────────────
        panel = pygame.Surface((panel_w, WINDOW_H))
        panel.fill((26, 26, 33))

        C_HDR = (100, 180, 255)
        C_LBL = (150, 150, 170)
        C_VAL = (240, 240, 240)

        px, py = 20, 15

        # Helper: render text onto panel using freetype (returns Surface, Rect)
        def draw_text(font, text, pos, color):
            surf, rect = font.render(text, fgcolor=color)
            panel.blit(surf, pos)

        # Connection status
        if conn:
            stale = last > 0 and (time.time() - last) > 2
            if stale:
                draw_text(font_sm, "STALE DATA", (px, py), (255, 180, 0))
            else:
                draw_text(font_sm, f"CONNECTED  {port}", (px, py), (0, 200, 100))
        else:
            draw_text(font_sm, f"CONNECTING  {port}", (px, py), (255, 80, 80))
        py += 35

        # ── IMU section ────────────────────────────────────────────────────────
        draw_text(font_hdr, "IMU", (px, py), C_HDR)
        py += 28

        def vec_block(label, unit, vals, fmt):
            nonlocal py
            draw_text(font_lbl, f"{label} ({unit})", (px, py), C_LBL)
            py += 20
            for axis, v in zip("XYZ", vals):
                draw_text(font_val, f" {axis}: {v:{fmt}}", (px, py), C_VAL)
                py += 19
            py += 8

        vec_block("Gyroscope", "deg/s", gyro, ">9.2f")
        vec_block("Accelerometer", "g", accel, ">9.4f")
        vec_block("Magnetometer", "uT", mag, ">9.2f")

        # Quaternion
        draw_text(font_lbl, "Quaternion", (px, py), C_LBL)
        py += 20
        for axis, v in zip("WXYZ", quat):
            draw_text(font_val, f" {axis}: {v:>9.4f}", (px, py), C_VAL)
            py += 19
        py += 16

        # ── Barometer section ──────────────────────────────────────────────────
        draw_text(font_hdr, "Barometer", (px, py), C_HDR)
        py += 28

        for lbl, val, unit, fmt in [
            ("Temperature", temp, "C", ".2f"),
            ("Pressure", pres, "hPa", ".2f"),
            ("Altitude", alt, "m", ".2f"),
        ]:
            draw_text(font_lbl, lbl, (px, py), C_LBL)
            py += 20
            draw_text(font_val, f" {val:{fmt}} {unit}", (px, py), C_VAL)
            py += 26

        # Upload panel surface as OpenGL texture
        tex_data = pygame.image.tostring(panel, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, panel_tex)
        glTexSubImage2D(
            GL_TEXTURE_2D, 0, 0, 0, panel_w, WINDOW_H,
            GL_RGBA, GL_UNSIGNED_BYTE, tex_data,
        )

        # Draw the panel texture as a 2D quad
        glViewport(0, 0, WINDOW_W, WINDOW_H)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, WINDOW_W, 0, WINDOW_H, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_TEXTURE_2D)

        glColor3f(1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(PANEL_X, 0)
        glTexCoord2f(1, 0); glVertex2f(WINDOW_W, 0)
        glTexCoord2f(1, 1); glVertex2f(WINDOW_W, WINDOW_H)
        glTexCoord2f(0, 1); glVertex2f(PANEL_X, WINDOW_H)
        glEnd()

        # Separator line
        glDisable(GL_TEXTURE_2D)
        glColor3f(0.3, 0.4, 0.6)
        glLineWidth(2)
        glBegin(GL_LINES)
        glVertex2f(PANEL_X, 0)
        glVertex2f(PANEL_X, WINDOW_H)
        glEnd()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

        pygame.display.flip()
        clock.tick(60)

    reader.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
