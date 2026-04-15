#!/usr/bin/env python3
"""
GNC-Airbrakes Sensor Visualiser
Reads Teensy DEBUG_MODE serial output and displays 3D orientation + sensor stats.

Build debug firmware first:  make debug && make upload
Run:  visualiser/.venv/bin/python visualiser/visualiser.py
"""

import sys
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
        self.gyro  = [0.0, 0.0, 0.0]   # rad/s  (IMU1)
        self.accel = [0.0, 0.0, 0.0]   # m/s²   (IMU1)
        self.mag   = [0.0, 0.0, 0.0]   # Gauss
        self.temperature = 0.0
        self.pressure    = 0.0
        self.altitude    = 0.0
        self.quat = [1.0, 0.0, 0.0, 0.0]  # w x y z
        self.connected  = False
        self.last_update = 0.0


# ── Serial reader ─────────────────────────────────────────────────────────────
class SerialReader:
    """Background thread that reads and parses Teensy DEBUG_MODE CSV output.

    Format per tick:
        $IMU1,ax,ay,az,gx,gy,gz,temp
        $BARO1,temp,pressure,altitude
        $MAG,x,y,z
        $TMP1,temp
    """

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
                        line = ser.readline().decode("ascii", errors="ignore").strip()
                        if line:
                            self._parse(line)
            except (serial.SerialException, OSError):
                with self.data.lock:
                    self.data.connected = False
                if self.running:
                    time.sleep(1)

    @staticmethod
    def _floats(parts):
        out = []
        for p in parts:
            try:
                out.append(float(p) if p not in ('', 'nan', 'NaN') else float('nan'))
            except ValueError:
                out.append(float('nan'))
        return out

    def _parse(self, line):
        if not line.startswith('$') or line.startswith('$TICK'):
            return
        parts = line.split(',')
        tag = parts[0]
        vals = self._floats(parts[1:])

        with self.data.lock:
            self.data.last_update = time.time()

            if tag == '$IMU1' and len(vals) >= 6:
                self.data.accel = vals[0:3]
                self.data.gyro  = vals[3:6]
                if not any(math.isnan(v) for v in vals[0:3]):
                    self.data.quat = compute_quat(
                        vals[0], vals[1], vals[2],
                        *self.data.mag
                    )

            elif tag == '$BARO1' and len(vals) >= 3:
                self.data.temperature = vals[0]
                self.data.pressure    = vals[1]
                self.data.altitude    = vals[2]

            elif tag == '$MAG' and len(vals) >= 3:
                self.data.mag = vals[0:3]


# ── Orientation math ───────────────────────────────────────────────────────────
def compute_quat(ax, ay, az, mx, my, mz):
    """Full orientation quaternion from accelerometer + magnetometer.

    Accel gives roll/pitch (tilt from gravity).
    Tilt-compensated mag gives yaw (heading).
    Result is in ZYX Euler convention (w, x, y, z).
    Falls back to accel-only (yaw=0) if mag is unusable.
    """
    an = math.sqrt(ax*ax + ay*ay + az*az)
    if an < 1e-6:
        return [1.0, 0.0, 0.0, 0.0]
    ax, ay, az = ax/an, ay/an, az/an

    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    # tilt-compensated magnetic heading
    yaw = 0.0
    mn = math.sqrt(mx*mx + my*my + mz*mz)
    if mn > 1e-6 and not any(math.isnan(v) for v in (mx, my, mz)):
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        mx2 =  mx*cp           + mz*sp
        my2 =  mx*sr*sp + my*cr - mz*sr*cp
        yaw = math.atan2(-my2, mx2)

    # ZYX Euler → quaternion
    cr2, sr2 = math.cos(roll/2),  math.sin(roll/2)
    cp2, sp2 = math.cos(pitch/2), math.sin(pitch/2)
    cy2, sy2 = math.cos(yaw/2),   math.sin(yaw/2)

    w = cr2*cp2*cy2 + sr2*sp2*sy2
    x = sr2*cp2*cy2 - cr2*sp2*sy2
    y = cr2*sp2*cy2 + sr2*cp2*sy2
    z = cr2*cp2*sy2 - sr2*sp2*cy2
    return [w, x, y, z]


def quat_to_gl_matrix(w, x, y, z):
    """Quaternion → column-major 4×4 OpenGL rotation matrix.

    Also remaps IMU frame (Z-up) to OpenGL frame (Y-up):
      IMU X → GL X,  IMU Y → GL -Z,  IMU Z → GL Y
    """
    # rotation matrix from quaternion (row-major first)
    r00 = 1 - 2*(y*y + z*z);  r01 = 2*(x*y - w*z);  r02 = 2*(x*z + w*y)
    r10 = 2*(x*y + w*z);      r11 = 1 - 2*(x*x+z*z); r12 = 2*(y*z - w*x)
    r20 = 2*(x*z - w*y);      r21 = 2*(y*z + w*x);  r22 = 1 - 2*(x*x+y*y)

    # IMU→GL axis remap: new_col = [R_imu_x, R_imu_z, -R_imu_y]
    # column 0 (GL X ← IMU X): r*0
    # column 1 (GL Y ← IMU Z): r*2
    # column 2 (GL Z ← -IMU Y): -r*1
    return [
        r00,  r20, -r10, 0,
        r01,  r21, -r11, 0,
        r02,  r22, -r12, 0,
        0,    0,    0,   1,
    ]


# ── 3D drawing ─────────────────────────────────────────────────────────────────
def draw_rocket():
    """Draw a rocket shape at the origin, nose pointing +Y."""
    body_r, body_h, nose_h, fin_sz, seg = 0.3, 2.0, 0.8, 0.5, 24

    glColor3f(0.8, 0.8, 0.85)
    glBegin(GL_QUAD_STRIP)
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg
        cx, cz = body_r * math.cos(a), body_r * math.sin(a)
        glNormal3f(math.cos(a), 0, math.sin(a))
        glVertex3f(cx, -body_h / 2, cz)
        glVertex3f(cx,  body_h / 2, cz)
    glEnd()

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

    glColor3f(0.3, 0.3, 0.35)
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, -1, 0)
    glVertex3f(0, -body_h / 2, 0)
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg
        glVertex3f(body_r * math.cos(a), -body_h / 2, body_r * math.sin(a))
    glEnd()

    glColor3f(0.2, 0.5, 0.9)
    for fa in (0, 90, 180, 270):
        glPushMatrix()
        glRotatef(fa, 0, 1, 0)
        glBegin(GL_TRIANGLES)
        glNormal3f(0, 0, 1)
        glVertex3f(body_r,           -body_h / 2, 0)
        glVertex3f(body_r + fin_sz,  -body_h / 2, 0)
        glVertex3f(body_r,           -body_h / 2 + fin_sz * 1.5, 0)
        glNormal3f(0, 0, -1)
        glVertex3f(body_r,           -body_h / 2, 0)
        glVertex3f(body_r,           -body_h / 2 + fin_sz * 1.5, 0)
        glVertex3f(body_r + fin_sz,  -body_h / 2, 0)
        glEnd()
        glPopMatrix()


def draw_axes(length=1.5):
    glLineWidth(2)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3f(0,0,0); glVertex3f(length,0,0)
    glColor3f(0, 1, 0); glVertex3f(0,0,0); glVertex3f(0,length,0)
    glColor3f(0, 0, 1); glVertex3f(0,0,0); glVertex3f(0,0,length)
    glEnd()
    glLineWidth(1)


def draw_grid():
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
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glLightfv(GL_LIGHT0, GL_POSITION, [5, 10, 5, 0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.8, 0.8, 0.8, 1])
    glLightfv(GL_LIGHT1, GL_POSITION, [-5, 5, -3, 0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.3, 0.3, 0.4, 1])
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    panel_w = WINDOW_W - PANEL_X
    panel_tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, panel_tex)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, panel_w, WINDOW_H, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, None)

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

        with data.lock:
            gyro  = list(data.gyro)
            accel = list(data.accel)
            mag   = list(data.mag)
            temp  = data.temperature
            pres  = data.pressure
            alt   = data.altitude
            quat  = list(data.quat)
            conn  = data.connected
            last  = data.last_update

        # ── 3D viewport ────────────────────────────────────────────────────────
        glViewport(0, 0, PANEL_X, WINDOW_H)
        glClearColor(0.12, 0.12, 0.15, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, PANEL_X / WINDOW_H, 0.1, 100)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(6, 4, 6, 0, 0, 0, 0, 1, 0)

        glDisable(GL_LIGHTING)
        draw_grid()
        draw_axes()
        glEnable(GL_LIGHTING)

        glPushMatrix()
        glMultMatrixf(quat_to_gl_matrix(*quat))
        draw_rocket()
        glDisable(GL_LIGHTING)
        draw_axes(2.0)
        glEnable(GL_LIGHTING)
        glPopMatrix()

        # ── Stats panel ────────────────────────────────────────────────────────
        panel = pygame.Surface((panel_w, WINDOW_H))
        panel.fill((26, 26, 33))

        C_HDR = (100, 180, 255)
        C_LBL = (150, 150, 170)
        C_VAL = (240, 240, 240)
        C_DIM = (100, 100, 120)

        px, py = 20, 15

        def draw_text(font, text, pos, color):
            surf, _ = font.render(text, fgcolor=color)
            panel.blit(surf, pos)

        # connection status
        if conn:
            stale = last > 0 and (time.time() - last) > 2
            if stale:
                draw_text(font_sm, "STALE DATA", (px, py), (255, 180, 0))
            else:
                draw_text(font_sm, f"CONNECTED  {port}", (px, py), (0, 200, 100))
        else:
            draw_text(font_sm, f"CONNECTING  {port}", (px, py), (255, 80, 80))
        py += 35

        # ── Quaternion ─────────────────────────────────────────────────────────
        draw_text(font_hdr, "Orientation", (px, py), C_HDR)
        py += 28
        for lbl, v in zip(("w", "x", "y", "z"), quat):
            draw_text(font_lbl, f" {lbl}:", (px, py), C_LBL)
            draw_text(font_val, f"{v:+.4f}", (px + 32, py), C_VAL)
            py += 22
        py += 10

        # ── IMU ────────────────────────────────────────────────────────────────
        draw_text(font_hdr, "IMU1", (px, py), C_HDR)
        py += 28

        def vec_block(label, unit, vals, fmt):
            nonlocal py
            draw_text(font_lbl, f"{label} ({unit})", (px, py), C_LBL)
            py += 20
            for axis, v in zip("XYZ", vals):
                color = C_DIM if math.isnan(v) else C_VAL
                txt = "nan" if math.isnan(v) else f"{v:{fmt}}"
                draw_text(font_val, f" {axis}: {txt:>9}", (px, py), color)
                py += 19
            py += 8

        vec_block("Accel",  "m/s\u00b2", accel, ">9.3f")
        vec_block("Gyro",   "rad/s",     gyro,  ">9.3f")
        vec_block("Mag",    "Gauss",     mag,   ">9.4f")

        # ── Barometer ──────────────────────────────────────────────────────────
        draw_text(font_hdr, "Barometer", (px, py), C_HDR)
        py += 28
        for lbl, val, unit, fmt in [
            ("Temperature", temp, "C",   ".2f"),
            ("Pressure",    pres, "Pa",  ".1f"),
            ("Altitude",    alt,  "m",   ".2f"),
        ]:
            draw_text(font_lbl, lbl, (px, py), C_LBL)
            py += 20
            color = C_DIM if math.isnan(val) else C_VAL
            txt = "nan" if math.isnan(val) else f"{val:{fmt}}"
            draw_text(font_val, f" {txt} {unit}", (px, py), color)
            py += 26

        # upload panel as OpenGL texture
        tex_data = pygame.image.tobytes(panel, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, panel_tex)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, panel_w, WINDOW_H,
                        GL_RGBA, GL_UNSIGNED_BYTE, tex_data)

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
