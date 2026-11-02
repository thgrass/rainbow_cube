#!/usr/bin/env python3
"""
Rainbow Cube 4x4x4 - PC control GUI (Python + Qt).

Controls a Seeed Studio Rainbowduino v3.0 (with the 4x4x4 RGB LED cube
attached) over a USB serial link, e.g. /dev/ttyUSB0.

The Rainbowduino must run the matching serial slave firmware
(firmware/RainbowCubeSerial/RainbowCubeSerial.ino, upload once with the
Arduino IDE). Protocol v2: 19200 baud 8N1 (default), binary commands,
each followed by an XOR checksum byte over cmd+payload:

    0x01 SET_PIXEL   z x y r g b     set one voxel (z,x,y in 0..3)
    0x02 FILL        r g b           fill all 64 voxels
    0x03 CLEAR                       all voxels off
    0x04 BRIGHTNESS  level           global brightness 0..255
    0x05 FRAME       192 bytes       whole frame, order z->x->y, each r g b
    0x06 PING                        device answers "PONG"
    0x07 STATS                       device answers "STAT"+u32 frames_ok+u32 cmds_err
    0x08 SET_BAUD    u32 LE baud     switch UART baud at runtime

Baud note: the Rainbowduino display ISR blocks interrupts for ~0.25 ms
every 625 us, so received bytes are lost if two arrive within one ISR
window. Measured on hardware: 19200 baud is 100% reliable (300/300
frames), 38400+ loses most bytes with the stock library. Hence 19200.

Features:
  * click/drag painting of individual voxels in a 4-layer view
    (left click = brush color, right click = erase, double click = pick
    that voxel's color)
  * fill / clear / random, global brightness, live or on-demand sending
  * built-in animations (rain, snake, plasma, ...) plus a keyframe
    animation editor with JSON save/load

Run:  python3 rainbow_cube_gui.py [--port /dev/ttyUSB0] [--baud 115200] [--connect]
Requires: pyserial, PyQt5 (or PyQt6).
"""

import argparse
import colorsys
import json
import math
import os
import random
import sys
import threading
import time
from datetime import datetime

# --------------------------------------------------------------------------
# Qt compatibility layer (PyQt5 preferred, PyQt6 works too)
# --------------------------------------------------------------------------
QT_VERSION = 5
try:
    from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, QSize
    from PyQt5.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QFileDialog,
        QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
        QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider,
        QSpinBox, QSplitter, QVBoxLayout, QWidget, QColorDialog,
    )
    ALIGN_CENTER = Qt.AlignCenter
    ALIGN_LEFT = Qt.AlignLeft | Qt.AlignVCenter
    TRANSPARENT = Qt.transparent
    ANTIALIAS = QPainter.Antialiasing
    LEFT_BUTTON = Qt.LeftButton
    RIGHT_BUTTON = Qt.RightButton
    HORIZONTAL = Qt.Horizontal
except ImportError:
    from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal, QSize
    from PyQt6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
    from PyQt6.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QFileDialog,
        QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
        QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider,
        QSpinBox, QSplitter, QVBoxLayout, QWidget, QColorDialog,
    )
    QT_VERSION = 6
    ALIGN_CENTER = Qt.AlignmentFlag.AlignCenter
    ALIGN_LEFT = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
    TRANSPARENT = Qt.GlobalColor.transparent
    ANTIALIAS = QPainter.RenderHint.Antialiasing
    LEFT_BUTTON = Qt.MouseButton.LeftButton
    RIGHT_BUTTON = Qt.MouseButton.RightButton
    HORIZONTAL = Qt.Orientation.Horizontal

import serial
import serial.tools.list_ports

# --------------------------------------------------------------------------
# Protocol
# --------------------------------------------------------------------------
CMD_SET_PIXEL = 0x01
CMD_FILL = 0x02
CMD_CLEAR = 0x03
CMD_BRIGHTNESS = 0x04
CMD_FRAME = 0x05
CMD_PING = 0x06
CMD_STATS = 0x07
CMD_SET_BAUD = 0x08
PONG = b"PONG"
STAT_MAGIC = b"STAT"

N = 4                    # cube is N x N x N
VOXELS = N * N * N       # 64
FRAME_BYTES = VOXELS * 3  # 192

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 19200      # highest baud that is 100% reliable with the stock library
MAX_RELIABLE_BAUD = 19200


def vidx(z, x, y):
    """Voxel index in the flat frame, order z -> x -> y."""
    return (z * N + x) * N + y


def _checksum(data):
    x = 0
    for b in data:
        x ^= b
    return x


def _cmd(op, payload=b""):
    d = bytes((op,)) + payload
    return d + bytes((_checksum(d),))


def enc_set_pixel(z, x, y, r, g, b):
    return _cmd(CMD_SET_PIXEL, bytes((z, x, y, r, g, b)))


def enc_fill(r, g, b):
    return _cmd(CMD_FILL, bytes((r, g, b)))


def enc_clear():
    return _cmd(CMD_CLEAR)


def enc_brightness(level):
    return _cmd(CMD_BRIGHTNESS, bytes((max(0, min(255, int(level))),)))


def enc_frame(voxels):
    """voxels: flat list of 64 [r, g, b] in z->x->y order."""
    out = bytearray()
    for v in voxels:
        out += bytes((v[0] & 0xFF, v[1] & 0xFF, v[2] & 0xFF))
    return _cmd(CMD_FRAME, bytes(out))


def enc_ping():
    return _cmd(CMD_PING)


def enc_stats():
    return _cmd(CMD_STATS)


def enc_set_baud(baud):
    return _cmd(CMD_SET_BAUD, int(baud).to_bytes(4, "little"))


# --------------------------------------------------------------------------
# Cube model
# --------------------------------------------------------------------------
def blank_frame():
    return [[0, 0, 0] for _ in range(VOXELS)]


class CubeModel:
    def __init__(self):
        self.voxels = blank_frame()

    def set(self, z, x, y, rgb):
        self.voxels[vidx(z, x, y)] = [rgb[0] & 0xFF, rgb[1] & 0xFF, rgb[2] & 0xFF]

    def get(self, z, x, y):
        return tuple(self.voxels[vidx(z, x, y)])

    def clear(self):
        self.voxels = blank_frame()

    def fill(self, rgb):
        c = [rgb[0] & 0xFF, rgb[1] & 0xFF, rgb[2] & 0xFF]
        self.voxels = [list(c) for _ in range(VOXELS)]

    def randomize(self, rng=None):
        rng = rng or random
        self.voxels = [[rng.randrange(256), rng.randrange(256), rng.randrange(256)]
                       for _ in range(VOXELS)]

    def from_flat(self, flat):
        assert len(flat) == VOXELS
        self.voxels = [[int(c[0]) & 0xFF, int(c[1]) & 0xFF, int(c[2]) & 0xFF] for c in flat]

    def copy(self):
        m = CubeModel()
        m.voxels = [list(v) for v in self.voxels]
        return m


# --------------------------------------------------------------------------
# Serial link
# --------------------------------------------------------------------------
class SerialLink:
    def __init__(self):
        self.ser = None

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud):
        self.close()
        s = serial.Serial()
        s.port = port
        s.baudrate = int(baud)
        s.bytesize = serial.EIGHTBITS
        s.parity = serial.PARITY_NONE
        s.stopbits = serial.STOPBITS_ONE
        s.timeout = 0.2        # read timeout (used by ping)
        s.write_timeout = 1.0
        try:
            s.exclusive = True
        except Exception:
            pass
        s.open()
        s.reset_input_buffer()
        self.ser = s

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write(self, data):
        if not self.is_open:
            return False
        self.ser.write(data)
        return True

    def ping(self, timeout=1.0):
        """Send PING, wait for PONG. Returns True if the device answered."""
        if not self.is_open:
            return False
        try:
            self.ser.reset_input_buffer()
            self.ser.write(enc_ping())
            self.ser.flush()
            deadline = time.monotonic() + timeout
            buf = b""
            while time.monotonic() < deadline:
                chunk = self.ser.read(64)
                if chunk:
                    buf += chunk
                    if PONG in buf:
                        return True
            return False
        except Exception:
            return False

    def stats(self, timeout=1.5):
        """Query STATS. Returns (frames_ok, cmds_err) or None."""
        if not self.is_open:
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(enc_stats())
            self.ser.flush()
            deadline = time.monotonic() + timeout
            buf = b""
            while time.monotonic() < deadline and len(buf) < 12:
                buf += self.ser.read(12 - len(buf))
            if len(buf) >= 12 and buf[:4] == STAT_MAGIC:
                return (int.from_bytes(buf[4:8], "little"),
                        int.from_bytes(buf[8:12], "little"))
            return None
        except Exception:
            return None

    def set_baud(self, baud):
        """Ask the device to switch UART baud, then switch the host side."""
        if not self.is_open:
            return False
        try:
            self.ser.write(enc_set_baud(baud))
            self.ser.flush()
            time.sleep(0.3)
            self.ser.baudrate = int(baud)
            time.sleep(0.1)
            return True
        except Exception:
            return False


class Handshake(QThread):
    """Runs in a worker thread after the port is opened: waits out the
    bootloader (opening the port toggles DTR -> ATmega reset), then finds
    the baud at which the device answers PING. Optionally switches the
    device to the user-requested baud. Emits done(baud) or failed(msg).
    The GUI must not write to the port until done() fired."""
    done = pyqtSignal(int)
    failed = pyqtSignal(str)

    def __init__(self, link, wanted, cancel, parent=None):
        super().__init__(parent)
        self.link = link
        self.wanted = wanted
        self.cancel = cancel  # threading.Event

    def run(self):
        time.sleep(2.0)  # bootloader window after the DTR reset
        candidates = list(dict.fromkeys(
            [self.wanted, DEFAULT_BAUD, 9600, 38400, 57600, 115200]))
        baud = None
        for _ in range(2):
            for b in candidates:
                if self.cancel.is_set() or not self.link.is_open:
                    return
                self.link.ser.baudrate = b
                time.sleep(0.05)
                if self.link.ping(0.8):
                    baud = b
                    break
            if baud is not None:
                break
        if baud is None:
            if not self.cancel.is_set():
                self.failed.emit("no PONG on any baud - is the RainbowCubeSerial firmware uploaded?")
            return
        if baud != self.wanted and self.wanted <= MAX_RELIABLE_BAUD:
            if self.link.set_baud(self.wanted):
                baud = self.wanted
        if self.cancel.is_set() or not self.link.is_open:
            return
        self.link.ser.baudrate = baud
        self.done.emit(baud)


# --------------------------------------------------------------------------
# Animations. Each animation yields frames as a flat list of 64 [r, g, b].
# --------------------------------------------------------------------------
def hsv255(h, s, v):
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, max(0.0, min(1.0, s)), max(0.0, min(1.0, v)))
    return [int(r * 255), int(g * 255), int(b * 255)]


class Anim:
    name = "?"

    def reset(self):
        pass

    def next_frame(self):
        """Return the next frame (flat list of 64 [r,g,b]) or None when done."""
        raise NotImplementedError


class RainAnim(Anim):
    name = "Rain"

    def reset(self):
        self.t = 0
        self.drops = []

    def next_frame(self):
        self.t += 1
        if len(self.drops) < 12 and random.random() < 0.6:
            self.drops.append({
                "x": random.randrange(N), "y": random.randrange(N),
                "z": float(N - 1) + random.random() * 2.0,
                "v": 0.15 + random.random() * 0.35,
                "c": hsv255(random.random(), 0.9, 1.0),
            })
        f = blank_frame()
        for d in self.drops:
            d["z"] -= d["v"]
            if d["z"] < 0:
                d["z"] = float(N - 1) + random.random() * 2.0
                d["x"] = random.randrange(N)
                d["y"] = random.randrange(N)
                d["c"] = hsv255(random.random(), 0.9, 1.0)
                continue
            zi = int(d["z"])
            if zi >= N:
                continue
            f[vidx(zi, d["x"], d["y"])] = list(d["c"])
            for k, w in ((1, 0.45), (2, 0.2)):
                zj = zi + k
                if zj < N:
                    f[vidx(zj, d["x"], d["y"])] = [int(c * w) for c in d["c"]]
        return f


def snake_path():
    """Hamiltonian path through all 64 voxels (serpentine)."""
    path = []
    for z in range(N):
        for x in range(N):
            ys = range(N) if (x + z) % 2 == 0 else range(N - 1, -1, -1)
            for y in ys:
                path.append((z, x, y))
    return path


class SnakeAnim(Anim):
    name = "Snake"
    TAIL = 22

    def reset(self):
        self.t = 0
        self.path = snake_path()

    def next_frame(self):
        f = blank_frame()
        head = self.t % len(self.path)
        hue = (self.t * 0.008) % 1.0
        for k in range(self.TAIL):
            p = self.path[(head - k) % len(self.path)]
            b = 1.0 - k / float(self.TAIL)
            f[vidx(*p)] = hsv255(hue, 0.85, b)
        self.t += 1
        return f


class ShellAnim(Anim):
    name = "Shell pulse"

    def reset(self):
        self.t = 0

    def next_frame(self):
        self.t += 1
        f = blank_frame()
        phase = self.t % 6
        col = hsv255((self.t * 0.03) % 1.0, 0.9, 1.0)
        for z in range(N):
            for x in range(N):
                for y in range(N):
                    d = math.sqrt((x - 1.5) ** 2 + (y - 1.5) ** 2 + (z - 1.5) ** 2)
                    if int(round(d * 2)) == phase:
                        f[vidx(z, x, y)] = list(col)
        return f


class PlasmaAnim(Anim):
    name = "Plasma wave"

    def reset(self):
        self.t = 0

    def next_frame(self):
        self.t += 1
        f = []
        for z in range(N):
            for x in range(N):
                for y in range(N):
                    v = (math.sin(x * 1.3 + self.t * 0.40) +
                         math.sin(y * 1.1 - self.t * 0.30) +
                         math.sin(z * 1.5 + self.t * 0.50) +
                         math.sin((x + y + z) * 0.8 + self.t * 0.20))
                    h = (v / 4.0) * 0.5 + 0.5 + self.t * 0.004
                    f.append(hsv255(h, 0.85, 0.25 + 0.75 * ((v + 4.0) / 8.0)))
        return f


class SparkleAnim(Anim):
    name = "Sparkle"

    def reset(self):
        self.t = 0
        self.energy = [0.0] * VOXELS
        self.col = blank_frame()

    def next_frame(self):
        self.t += 1
        f = blank_frame()
        for i in range(VOXELS):
            if random.random() < 0.07:
                self.energy[i] = 1.0
                self.col[i] = hsv255(random.random(), 0.8, 1.0)
            self.energy[i] *= 0.72
            f[i] = [int(c * self.energy[i]) for c in self.col[i]]
        return f


class SweepAnim(Anim):
    name = "Layer sweep"

    def reset(self):
        self.t = 0

    def next_frame(self):
        self.t += 1
        f = blank_frame()
        z = self.t % N
        col = hsv255((self.t * 0.04) % 1.0, 0.9, 1.0)
        for x in range(N):
            for y in range(N):
                f[vidx(z, x, y)] = list(col)
                f[vidx((z - 1) % N, x, y)] = [int(c * 0.25) for c in col]
        return f


class BounceAnim(Anim):
    name = "Bouncing ball"

    def reset(self):
        self.t = 0
        self.p = [2.0, 2.0, 3.0]       # x, y, z
        self.v = [0.35, -0.30, -0.50]
        self.trail = []

    def next_frame(self):
        self.t += 1
        self.v[2] -= 0.12              # gravity towards z = 0
        for i in range(3):
            self.p[i] += self.v[i]
        for i in range(3):
            if self.p[i] < 0.0:
                self.p[i] = 0.0
                self.v[i] = abs(self.v[i]) * 0.9
            if self.p[i] > N - 1:
                self.p[i] = float(N - 1)
                self.v[i] = -abs(self.v[i]) * 0.9
        self.trail.append(tuple(self.p))
        self.trail = self.trail[-6:]
        f = blank_frame()
        col = hsv255((self.t * 0.02) % 1.0, 0.9, 1.0)
        for k, tp in enumerate(self.trail):
            w = 1.0 - k / float(len(self.trail))
            f[vidx(int(round(tp[2])) % N, int(round(tp[0])) % N, int(round(tp[1])) % N)] = \
                [int(c * w) for c in col]
        return f


class HueFadeAnim(Anim):
    name = "Hue fade"

    def reset(self):
        self.t = 0

    def next_frame(self):
        self.t += 1
        c = hsv255((self.t * 0.01) % 1.0, 0.9, 1.0)
        return [list(c) for _ in range(VOXELS)]


class FrameSequenceAnim(Anim):
    """Plays a fixed list of keyframes."""

    def __init__(self, frames, loop=True, name="Custom"):
        self.frames = frames
        self.loop = loop
        self.name = name
        self.reset()

    def reset(self):
        self.i = 0

    def next_frame(self):
        if not self.frames:
            return None
        if self.i >= len(self.frames):
            if self.loop:
                self.i = 0
            else:
                return None
        f = self.frames[self.i]
        self.i += 1
        return [list(v) for v in f]


BUILTIN_ANIMATIONS = [RainAnim, SnakeAnim, ShellAnim, PlasmaAnim, SparkleAnim,
                      SweepAnim, BounceAnim, HueFadeAnim]


# --------------------------------------------------------------------------
# Cube view: paints the 4 layers (z = 0 bottom .. 3 top) as 4x4 grids.
# --------------------------------------------------------------------------
CELL = 46
PAD = 8
GAP = 22
LABEL_H = 26


class CubeView(QWidget):
    voxelChanged = pyqtSignal(int, int, int, int, int, int)  # z, x, y, r, g, b
    voxelPickRequested = pyqtSignal(int, int, int)            # z, x, y

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.brush = QColor(255, 0, 0)
        self._dragging = False
        self._drag_erase = False
        self._hover = None
        self.setMouseTracking(True)
        self.setMinimumSize(self._content_size())

    def _layer_w(self):
        return N * CELL + 2 * PAD

    def _content_size(self):
        w = N * self._layer_w() + (N - 1) * GAP
        h = LABEL_H + N * CELL + 2 * PAD + 6
        return QSize(w, h)

    def sizeHint(self):
        return self._content_size()

    def _cell_rect(self, z, x, y):
        ox = z * (self._layer_w() + GAP)
        return (ox + PAD + x * CELL, LABEL_H + PAD + y * CELL, CELL - 3, CELL - 3)

    def _cell_at(self, px, py):
        if py < LABEL_H:
            return None
        lw = self._layer_w()
        z = px // (lw + GAP)
        if z >= N:
            return None
        ox = z * (lw + GAP)
        lx, ly = px - ox, py - LABEL_H
        if not (PAD <= lx < PAD + N * CELL and PAD <= ly < PAD + N * CELL):
            return None
        x = (lx - PAD) // CELL
        y = (ly - PAD) // CELL
        if x >= N or y >= N:
            return None
        return (z, x, y)

    def _paint_cell(self, z, x, y, erase):
        if erase:
            rgb = (0, 0, 0)
        else:
            c = self.brush
            rgb = (c.red(), c.green(), c.blue())
        self.model.set(z, x, y, rgb)
        self.update()
        self.voxelChanged.emit(z, x, y, rgb[0], rgb[1], rgb[2])

    def mousePressEvent(self, ev):
        cell = self._cell_at(ev.position().x() if QT_VERSION == 6 else ev.x(),
                             ev.position().y() if QT_VERSION == 6 else ev.y())
        if cell is None:
            return
        btn = ev.button()
        if btn == LEFT_BUTTON:
            self._dragging = True
            self._drag_erase = False
            self._paint_cell(*cell, erase=False)
        elif btn == RIGHT_BUTTON:
            self._dragging = True
            self._drag_erase = True
            self._paint_cell(*cell, erase=True)

    def mouseMoveEvent(self, ev):
        px = ev.position().x() if QT_VERSION == 6 else ev.x()
        py = ev.position().y() if QT_VERSION == 6 else ev.y()
        cell = self._cell_at(px, py)
        self._hover = cell
        if self._dragging and cell is not None:
            self._paint_cell(*cell, erase=self._drag_erase)
        self.update()

    def mouseReleaseEvent(self, ev):
        self._dragging = False

    def mouseDoubleClickEvent(self, ev):
        px = ev.position().x() if QT_VERSION == 6 else ev.x()
        py = ev.position().y() if QT_VERSION == 6 else ev.y()
        cell = self._cell_at(px, py)
        if cell is not None:
            self.voxelPickRequested.emit(*cell)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(ANTIALIAS)
        lw = self._layer_w()
        for z in range(N):
            ox = z * (lw + GAP)
            label = "Z=%d%s" % (z, " (bottom)" if z == 0 else " (top)" if z == N - 1 else "")
            p.setPen(QPen(QColor(200, 200, 200)))
            p.drawText(ox, 4, lw, LABEL_H - 6, ALIGN_CENTER, label)
            for x in range(N):
                for y in range(N):
                    rx, ry, rw, rh = self._cell_rect(z, x, y)
                    r, g, b = self.model.get(z, x, y)
                    if r or g or b:
                        fill = QColor(r, g, b)
                    else:
                        fill = QColor(26, 26, 30)
                    p.setPen(QPen(QColor(0, 0, 0)))
                    p.setBrush(fill)
                    p.drawRoundedRect(rx, ry, rw, rh, 7, 7)
                    if self._hover == (z, x, y):
                        p.setPen(QPen(QColor(255, 255, 255), 2))
                        p.setBrush(Qt.NoBrush if QT_VERSION == 5 else Qt.BrushStyle.NoBrush)
                        p.drawRoundedRect(rx, ry, rw, rh, 7, 7)
        p.end()


def frame_icon(frame, cell=5, pad=2, gap=3):
    """Render a flat 64-voxel frame as a small 4-layer strip icon."""
    lw = N * cell + 2 * pad
    w = N * lw + (N - 1) * gap
    h = lw
    pm = QPixmap(w, h)
    pm.fill(TRANSPARENT)
    p = QPainter(pm)
    p.setRenderHint(ANTIALIAS)
    for z in range(N):
        ox = z * (lw + gap)
        for x in range(N):
            for y in range(N):
                r, g, b = frame[vidx(z, x, y)]
                p.setPen(QPen(QColor(r, g, b) if (r or g or b) else QColor(40, 40, 44)))
                p.setBrush(QColor(r, g, b) if (r or g or b) else QColor(40, 40, 44))
                p.drawRect(ox + pad + x * cell, pad + y * cell, cell - 1, cell - 1)
    p.end()
    return QIcon(pm)


def pick_color(parent, current):
    if QT_VERSION == 6:
        col, ok = QColorDialog.getColor(current, parent, "Pick voxel color")
        return col if ok and col.isValid() else None
    col = QColorDialog.getColor(current, parent, "Pick voxel color")
    return col if col.isValid() else None


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
PALETTE = [
    (255, 0, 0), (255, 128, 0), (255, 255, 0), (0, 255, 0), (0, 255, 128),
    (0, 255, 255), (0, 128, 255), (0, 0, 255), (128, 0, 255), (255, 0, 255),
    (255, 255, 255), (255, 255, 255),
]


class MainWindow(QMainWindow):
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, autostart=False):
        super().__init__()
        self.setWindowTitle("Rainbow Cube 4x4x4 Controller")
        self.model = CubeModel()
        self.link = SerialLink()
        self.port = port
        self.baud = baud

        self.anim = None            # active Anim instance
        self.keyframes = []         # list of flat frames
        self.player = QTimer(self)
        self.player.timeout.connect(self._on_tick)
        self.frame_no = 0
        self._wanted_baud = baud
        self._ready = False         # True only after the device answered PONG
        self._handshake = None      # Handshake QThread while probing
        self._hs_cancel = threading.Event()
        self._pattern_running = False
        self._last_drop_log = 0.0

        self._build_ui()
        self._refresh_ports()
        self.port_edit.setEditText(port)
        self.baud_box.setEditText(str(baud))
        self._update_conn_ui()

        if autostart:
            QTimer.singleShot(200, self.toggle_connect)

    # ---------------- UI construction ----------------
    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)

        splitter = QSplitter(HORIZONTAL)
        root.addWidget(splitter)

        # ---- left: connection + cube editor ----
        left = QWidget()
        lv = QVBoxLayout(left)

        conn = QGroupBox("Connection")
        ch = QHBoxLayout(conn)
        ch.addWidget(QLabel("Port:"))
        self.port_edit = QComboBox()
        self.port_edit.setEditable(True)
        self.port_edit.setMinimumWidth(180)
        ch.addWidget(self.port_edit, 1)
        btn = QPushButton("Refresh")
        btn.clicked.connect(self._refresh_ports)
        ch.addWidget(btn)
        ch.addWidget(QLabel("Baud:"))
        self.baud_box = QComboBox()
        self.baud_box.setEditable(True)
        for b in ("9600", "19200", "38400", "57600", "115200"):
            self.baud_box.addItem(b)
        # DEFAULT_BAUD is set below via setEditText
        self.baud_box.setEditText(str(DEFAULT_BAUD))
        ch.addWidget(self.baud_box)
        self.conn_btn = QPushButton("Connect")
        self.conn_btn.clicked.connect(self.toggle_connect)
        ch.addWidget(self.conn_btn)
        self.ping_btn = QPushButton("Ping")
        self.ping_btn.clicked.connect(self.do_ping)
        ch.addWidget(self.ping_btn)
        self.test_btn = QPushButton("Link test")
        self.test_btn.clicked.connect(self.link_test)
        ch.addWidget(self.test_btn)
        self.conn_status = QLabel("disconnected")
        ch.addWidget(self.conn_status, 1)
        lv.addWidget(conn)

        self.view = CubeView(self.model)
        self.view.voxelChanged.connect(self._on_voxel_changed)
        self.view.voxelPickRequested.connect(self._on_pick_voxel)
        lv.addWidget(self.view, 0, ALIGN_CENTER)

        brush = QGroupBox("Brush")
        bh = QHBoxLayout(brush)
        self.brush_btn = QPushButton("Color...")
        self.brush_btn.setFixedSize(90, 28)
        self.brush_btn.clicked.connect(self._pick_brush)
        bh.addWidget(self.brush_btn)
        for rgb in PALETTE:
            b = QPushButton()
            b.setFixedSize(26, 26)
            b.setStyleSheet("background-color: rgb(%d,%d,%d); border: 1px solid #555;"
                            % rgb)
            b.clicked.connect(lambda _=False, c=rgb: self._set_brush(QColor(*c)))
            bh.addWidget(b)
        bh.addWidget(QLabel("R"))
        self.sp_r = QSpinBox()
        self.sp_r.setRange(0, 255)
        self.sp_r.valueChanged.connect(self._spin_changed)
        bh.addWidget(self.sp_r)
        bh.addWidget(QLabel("G"))
        self.sp_g = QSpinBox()
        self.sp_g.setRange(0, 255)
        self.sp_g.valueChanged.connect(self._spin_changed)
        bh.addWidget(self.sp_g)
        bh.addWidget(QLabel("B"))
        self.sp_b = QSpinBox()
        self.sp_b.setRange(0, 255)
        self.sp_b.valueChanged.connect(self._spin_changed)
        bh.addWidget(self.sp_b)
        self._set_brush(QColor(255, 0, 0))
        lv.addWidget(brush)

        draw = QGroupBox("Draw")
        dh = QHBoxLayout(draw)
        for text, fn in (("Clear", self.do_clear), ("Fill with brush", self.do_fill),
                         ("Random", self.do_random), ("Send frame", self.send_frame),
                         ("Test pattern", self.test_pattern)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            dh.addWidget(b)
        self.live_box = QCheckBox("Live update")
        self.live_box.setChecked(True)
        dh.addWidget(self.live_box)
        dh.addWidget(QLabel("Brightness"))
        self.bright_slider = QSlider(HORIZONTAL)
        self.bright_slider.setRange(0, 255)
        self.bright_slider.setValue(255)
        self.bright_slider.setFixedWidth(140)
        self.bright_slider.sliderReleased.connect(self._brightness_changed)
        dh.addWidget(self.bright_slider)
        self.bright_spin = QSpinBox()
        self.bright_spin.setRange(0, 255)
        self.bright_spin.setValue(255)
        self.bright_spin.valueChanged.connect(self._brightness_spin)
        dh.addWidget(self.bright_spin)
        lv.addWidget(draw)

        raw = QGroupBox("Raw command (hex bytes, e.g. '05 ff0000 ...')")
        rh = QHBoxLayout(raw)
        self.raw_edit = QLineEdit()
        self.raw_edit.setPlaceholderText("hex bytes...")
        rh.addWidget(self.raw_edit, 1)
        b = QPushButton("Send raw")
        b.clicked.connect(self._send_raw)
        rh.addWidget(b)
        lv.addWidget(raw)

        splitter.addWidget(left)

        # ---- right: animations ----
        right = QWidget()
        rv = QVBoxLayout(right)
        right.setMaximumWidth(360)

        ag = QGroupBox("Built-in animations")
        av = QVBoxLayout(ag)
        self.anim_list = QListWidget()
        for cls in BUILTIN_ANIMATIONS:
            self.anim_list.addItem(cls.name)
        self.anim_list.currentRowChanged.connect(self._anim_selected)
        av.addWidget(self.anim_list)
        pc = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self.toggle_play)
        pc.addWidget(self.play_btn)
        b = QPushButton("Stop")
        b.clicked.connect(self.stop_play)
        pc.addWidget(b)
        pc.addWidget(QLabel("FPS"))
        self.fps_spin = QSpinBox()
        self.fps_spin.setRange(1, 60)
        self.fps_spin.setValue(15)
        self.fps_spin.valueChanged.connect(self._fps_changed)
        pc.addWidget(self.fps_spin)
        self.loop_box = QCheckBox("Loop")
        self.loop_box.setChecked(True)
        pc.addWidget(self.loop_box)
        av.addLayout(pc)
        rv.addWidget(ag)

        kg = QGroupBox("Custom keyframe animation")
        kv = QVBoxLayout(kg)
        self.kf_list = QListWidget()
        self.kf_list.setIconSize(QSize(110, 26))
        kv.addWidget(self.kf_list)
        kb1 = QHBoxLayout()
        for text, fn in (("Add current frame", self.kf_add), ("Remove", self.kf_remove),
                         ("Up", self.kf_up), ("Down", self.kf_down)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            kb1.addWidget(b)
        kv.addLayout(kb1)
        kb2 = QHBoxLayout()
        for text, fn in (("Play keyframes", self.play_keyframes), ("Clear", self.kf_clear),
                         ("Load...", self.load_animation), ("Save...", self.save_animation)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            kb2.addWidget(b)
        kv.addLayout(kb2)
        rv.addWidget(kg)

        sg = QGroupBox("Scene")
        sv = QVBoxLayout(sg)
        sb = QHBoxLayout()
        for text, fn in (("Load scene...", self.load_scene), ("Save scene...", self.save_scene)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            sb.addWidget(b)
        sv.addLayout(sb)
        rv.addWidget(sg)

        splitter.addWidget(right)
        splitter.setSizes([900, 340])

        # ---- log ----
        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumHeight(110)
        root.addWidget(self.log_edit)

        self.setCentralWidget(central)
        self.statusBar().showMessage("ready")
        self.resize(1280, 760)

    # ---------------- helpers ----------------
    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_edit.appendPlainText("[%s] %s" % (ts, msg))
        # keep the log bounded
        if self.log_edit.document().blockCount() > 400:
            lines = self.log_edit.toPlainText().splitlines()
            self.log_edit.setPlainText("\n".join(lines[-300:]))

    def _refresh_ports(self):
        current = self.port_edit.currentText()
        self.port_edit.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            self.port_edit.addItem("%s (%s)" % (p.device, p.description), p.device)
        if current:
            self.port_edit.setEditText(current)

    def _port_device(self):
        text = self.port_edit.currentText().strip()
        # combo items carry the raw device path as user data
        idx = self.port_edit.findText(text)
        if idx >= 0 and self.port_edit.itemData(idx):
            return self.port_edit.itemData(idx)
        return text.split(" ")[0]

    def _set_brush(self, color):
        self.view.brush = color
        self.brush_btn.setStyleSheet(
            "background-color: %s; border: 1px solid #888; color: %s;"
            % (color.name(), "#000" if (color.red() + color.green() + color.blue()) > 380 else "#fff"))
        self.brush_btn.setText("Color...")
        self.sp_r.blockSignals(True)
        self.sp_g.blockSignals(True)
        self.sp_b.blockSignals(True)
        self.sp_r.setValue(color.red())
        self.sp_g.setValue(color.green())
        self.sp_b.setValue(color.blue())
        self.sp_r.blockSignals(False)
        self.sp_g.blockSignals(False)
        self.sp_b.blockSignals(False)

    def _pick_brush(self):
        col = pick_color(self, self.view.brush)
        if col is not None:
            self._set_brush(col)

    def _spin_changed(self, _v):
        self._set_brush(QColor(self.sp_r.value(), self.sp_g.value(), self.sp_b.value()))

    def _brightness_changed(self):
        v = self.bright_slider.value()
        self.bright_spin.blockSignals(True)
        self.bright_spin.setValue(v)
        self.bright_spin.blockSignals(False)
        self._send(enc_brightness(v), "brightness=%d" % v)

    def _brightness_spin(self, v):
        self.bright_slider.blockSignals(True)
        self.bright_slider.setValue(v)
        self.bright_slider.blockSignals(False)
        self._send(enc_brightness(v), "brightness=%d" % v)

    def _send(self, data, what):
        if self.link.is_open and self._ready:
            self.link.write(data)
            self.log("sent %s (%d bytes)" % (what, len(data)))
        else:
            # never write before the device answered PING: during the
            # bootloader window the bytes would be lost/garbled and freeze
            # the GUI on blocking writes
            now = time.monotonic()
            if now - self._last_drop_log > 1.0:
                self._last_drop_log = now
                self.log("device not ready - dropped %s (connect and wait for PONG)" % what)

    def _send_raw(self):
        text = self.raw_edit.text().strip().replace(" ", "").replace(",", "")
        if not text:
            return
        try:
            data = bytes.fromhex(text)
        except ValueError:
            self.log("raw: invalid hex")
            return
        self._send(data, "raw")
        self.raw_edit.clear()

    # ---------------- connection ----------------
    def toggle_connect(self):
        if self.link.is_open or (self._handshake is not None and self._handshake.isRunning()):
            self._hs_cancel.set()
            if self._handshake is not None and self._handshake.isRunning():
                self._handshake.wait(8000)
            self._handshake = None
            self._ready = False
            self.link.close()
            self.log("disconnected")
            self._update_conn_ui()
            return
        port = self._port_device()
        try:
            self._wanted_baud = int(self.baud_box.currentText().strip())
        except ValueError:
            self.log("invalid baud rate")
            return
        try:
            self.link.connect(port, self._wanted_baud)
        except Exception as e:
            self.log("connect failed: %s" % e)
            QMessageBox.warning(self, "Connect", "Could not open %s:\n%s" % (port, e))
            self._update_conn_ui()
            return
        self.log("opened %s - waiting for device (bootloader)..." % port)
        self._update_conn_ui()
        # Opening the port toggles DTR -> the ATmega resets and the
        # bootloader runs for ~1 s. The handshake runs in a worker thread
        # so the GUI stays responsive; nothing is sent to the port until
        # the device has answered PING (self._ready).
        self._hs_cancel.clear()
        self._handshake = Handshake(self.link, self._wanted_baud, self._hs_cancel)
        self._handshake.done.connect(self._on_handshake_done)
        self._handshake.failed.connect(self._on_handshake_failed)
        self._handshake.start()

    def _on_handshake_done(self, baud):
        if not self.link.is_open:
            return
        self._ready = True
        self.baud_box.setEditText(str(baud))
        self.log("ping OK @ %d baud (PONG) - firmware is running" % baud)
        if self._wanted_baud != baud:
            self.log("note: requested %d baud, device answered at %d baud"
                     % (self._wanted_baud, baud))
        if self._wanted_baud > MAX_RELIABLE_BAUD:
            self.log("warning: %d baud is unreliable on the Rainbowduino (the display ISR "
                     "starves the UART and commands get lost); using %d baud instead"
                     % (self._wanted_baud, baud))
        elif baud > MAX_RELIABLE_BAUD:
            self.log("warning: %d baud is above the reliable maximum (%d) - commands may be lost"
                     % (baud, MAX_RELIABLE_BAUD))
        max_fps = max(1, baud // 2000)  # a 194-byte frame is 1940 bits on the wire
        self.fps_spin.setMaximum(max_fps)
        if self.fps_spin.value() > max_fps:
            self.fps_spin.setValue(max_fps)
        self._update_conn_ui()
        self._send(enc_brightness(self.bright_slider.value()), "brightness")
        self.send_frame()
        self._start_hello()

    def _on_handshake_failed(self, msg):
        self._ready = False
        self.log("ping: %s" % msg)
        self.statusBar().showMessage("device not answering - see log", 6000)
        self._update_conn_ui()

    def _start_hello(self):
        """Brief low-brightness all-red flash: instant visual proof that the
        whole chain (GUI -> UART -> firmware -> LEDs) works. Non-blocking:
        steps are scheduled on the event loop."""
        b0 = self.bright_slider.value()
        self._send(enc_brightness(48), "brightness")
        steps = [enc_fill(255, 0, 0), enc_clear(), enc_fill(255, 0, 0), enc_clear()]

        def step(i=0):
            if not self._ready:
                return
            if i < len(steps):
                self._send(steps[i], "hello")
                QTimer.singleShot(150, lambda: step(i + 1))
            else:
                self._send(enc_brightness(b0), "brightness")
                self.send_frame()

        step()

    def do_ping(self):
        if not (self.link.is_open and self._ready):
            self.log("ping: device not ready (connect first)")
            return
        ok = self.link.ping(1.0)
        self.log("ping: %s" % ("PONG received" if ok else "no answer"))
        self.statusBar().showMessage("ping OK" if ok else "ping failed", 3000)

    def link_test(self):
        """Send a burst of frames + set-pixels, then compare STATS counters
        to measure whether any bytes were lost on the device."""
        if not (self.link.is_open and self._ready):
            self.log("link test: device not ready (connect first)")
            return
        s0 = self.link.stats()
        if s0 is None:
            self.log("link test: no STATS reply")
            return
        ok0, err0 = s0
        for i in range(50):
            fr = blank_frame()
            fr[i % VOXELS] = [255, 255, 255]
            fr[(i * 7 + 13) % VOXELS] = [255, 0, 0]
            self.link.write(enc_frame(fr))
        for i in range(20):
            self.link.write(enc_set_pixel(i % 4, (i * 3) % 4, (i * 5) % 4, 200, 100, 50))
        time.sleep(0.3)
        s1 = self.link.stats()
        self.send_frame()  # restore the frame shown before the test
        if s1 is None:
            self.log("link test: no STATS reply after burst")
            return
        ok1, err1 = s1
        self.log("link test @ %s baud: frames %d/50 ok, cmds_err delta=%d"
                 % (self.baud_box.currentText(), ok1 - ok0, err1 - err0))
        self.statusBar().showMessage(
            "link test: %d/50 frames ok, %d errors" % (ok1 - ok0, err1 - err0), 6000)

    def _update_conn_ui(self):
        open_ = self.link.is_open
        self.conn_btn.setText("Disconnect" if open_ else "Connect")
        self.ping_btn.setEnabled(open_)
        self.conn_status.setText(
            ("%s @ %s" % (self._port_device(), self.baud_box.currentText())) if open_
            else "disconnected")
        self.statusBar().showMessage("connected" if open_ else "disconnected")

    # ---------------- drawing actions ----------------
    def _on_voxel_changed(self, z, x, y, r, g, b):
        if self.live_box.isChecked():
            self._send(enc_set_pixel(z, x, y, r, g, b), "set(%d,%d,%d)=#%02x%02x%02x" % (z, x, y, r, g, b))

    def _on_pick_voxel(self, z, x, y):
        cur = QColor(*self.model.get(z, x, y))
        col = pick_color(self, cur if cur.isValid() else self.view.brush)
        if col is not None:
            self.model.set(z, x, y, (col.red(), col.green(), col.blue()))
            self.view.update()
            self._send(enc_set_pixel(z, x, y, col.red(), col.green(), col.blue()),
                       "set(%d,%d,%d)" % (z, x, y))

    def send_frame(self):
        self._send(enc_frame(self.model.voxels), "frame")

    def do_clear(self):
        self.model.clear()
        self.view.update()
        self._send(enc_clear(), "clear")

    def do_fill(self):
        c = self.view.brush
        self.model.fill((c.red(), c.green(), c.blue()))
        self.view.update()
        self._send(enc_fill(c.red(), c.green(), c.blue()), "fill")

    def do_random(self):
        self.model.randomize()
        self.view.update()
        self._send(enc_frame(self.model.voxels), "random frame")

    def test_pattern(self):
        """Obvious visual self-test: each layer in turn (Z0 red, Z1 green,
        Z2 blue, Z3 white), then a single-voxel sweep through all 64.
        Non-blocking: one frame is sent per event-loop timer tick."""
        if not (self.link.is_open and self._ready):
            self.log("test pattern: device not ready (connect first)")
            return
        if self._pattern_running:
            return
        self._pattern_running = True
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
        steps = []
        for z in range(N):
            fr = blank_frame()
            for x in range(N):
                for y in range(N):
                    fr[vidx(z, x, y)] = list(colors[z])
            steps.append((enc_frame(fr), 500))
        for i in range(VOXELS):
            fr = blank_frame()
            fr[i] = [255, 255, 255]
            steps.append((enc_frame(fr), 40))

        def step(i=0):
            if not self._ready:
                self._pattern_running = False
                return
            if i < len(steps):
                data, delay = steps[i]
                self._send(data, "test pattern")
                QTimer.singleShot(delay, lambda: step(i + 1))
            else:
                self._pattern_running = False
                self.send_frame()  # restore the editor's frame
                self.log("test pattern done")

        step()

    # ---------------- animations ----------------
    def _anim_selected(self, row):
        if 0 <= row < len(BUILTIN_ANIMATIONS):
            self.anim = BUILTIN_ANIMATIONS[row]()
            self.anim.reset()
            self.log("selected animation: %s" % self.anim.name)

    def toggle_play(self):
        if not (self.link.is_open and self._ready):
            self.log("play: device not ready (connect first)")
            return
        if self.player.isActive():
            self.player.stop()
            self.play_btn.setText("Play")
            return
        if self.anim is None:
            row = self.anim_list.currentRow()
            if row < 0:
                row = 0
                self.anim_list.setCurrentRow(0)
            self._anim_selected(row)
        self._start_play()

    def _start_play(self):
        self.anim.reset()
        self.frame_no = 0
        self.player.start(max(1, int(1000.0 / self.fps_spin.value())))
        self.play_btn.setText("Pause")
        self.log("playing: %s" % self.anim.name)

    def stop_play(self):
        self.player.stop()
        self.play_btn.setText("Play")
        if self.anim is not None:
            self.anim.reset()
        self.statusBar().showMessage("stopped", 2000)

    def _fps_changed(self, v):
        if self.player.isActive():
            self.player.start(max(1, int(1000.0 / v)))

    def _on_tick(self):
        if self.anim is None:
            self.stop_play()
            return
        frame = self.anim.next_frame()
        if frame is None:
            if self.loop_box.isChecked():
                self.anim.reset()
                frame = self.anim.next_frame()
            else:
                self.stop_play()
                return
        self.frame_no += 1
        self.model.from_flat(frame)
        self.view.update()
        self._send(enc_frame(frame), "anim frame %d" % self.frame_no)
        self.statusBar().showMessage(
            "playing %s - frame %d @ %d fps" % (self.anim.name, self.frame_no, self.fps_spin.value()))

    # ---------------- keyframes ----------------
    def _kf_refresh(self):
        self.kf_list.clear()
        for i, fr in enumerate(self.keyframes):
            item = QListWidgetItem("Frame %d" % (i + 1))
            item.setIcon(frame_icon(fr))
            self.kf_list.addItem(item)

    def kf_add(self):
        self.keyframes.append([list(v) for v in self.model.voxels])
        self._kf_refresh()
        self.log("keyframe %d added" % len(self.keyframes))

    def kf_remove(self):
        row = self.kf_list.currentRow()
        if 0 <= row < len(self.keyframes):
            del self.keyframes[row]
            self._kf_refresh()

    def _kf_move(self, delta):
        row = self.kf_list.currentRow()
        nrow = row + delta
        if 0 <= row < len(self.keyframes) and 0 <= nrow < len(self.keyframes):
            self.keyframes[row], self.keyframes[nrow] = self.keyframes[nrow], self.keyframes[row]
            self._kf_refresh()
            self.kf_list.setCurrentRow(nrow)

    def kf_up(self):
        self._kf_move(-1)

    def kf_down(self):
        self._kf_move(1)

    def kf_clear(self):
        self.keyframes = []
        self._kf_refresh()

    def play_keyframes(self):
        if not self.keyframes:
            self.log("no keyframes - add some first")
            return
        if not (self.link.is_open and self._ready):
            self.log("play: device not ready (connect first)")
            return
        self.anim = FrameSequenceAnim(self.keyframes, loop=self.loop_box.isChecked(),
                                      name="keyframes")
        if self.player.isActive():
            self.player.stop()
        self._start_play()

    # ---------------- file IO ----------------
    def save_scene(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save scene", "scene.json",
                                              "JSON (*.json)")
        if not path:
            return
        data = {"type": "scene", "size": N, "voxels": self.model.voxels}
        with open(path, "w") as f:
            json.dump(data, f)
        self.log("scene saved to %s" % path)

    def load_scene(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load scene", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            self.model.from_flat(data["voxels"])
            self.view.update()
            self.send_frame()
            self.log("scene loaded from %s" % path)
        except Exception as e:
            self.log("load scene failed: %s" % e)

    def save_animation(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save animation", "animation.json",
                                              "JSON (*.json)")
        if not path:
            return
        if not self.keyframes:
            self.log("no keyframes to save")
            return
        data = {"type": "animation", "size": N, "fps": self.fps_spin.value(),
                "loop": self.loop_box.isChecked(), "frames": self.keyframes}
        with open(path, "w") as f:
            json.dump(data, f)
        self.log("animation (%d frames) saved to %s" % (len(self.keyframes), path))

    def load_animation(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load animation", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            frames = data["frames"]
            assert len(frames) > 0 and all(len(fr) == VOXELS for fr in frames)
            self.keyframes = [[[int(c) for c in v] for v in fr] for fr in frames]
            self._kf_refresh()
            if "fps" in data:
                self.fps_spin.setValue(int(data["fps"]))
            if "loop" in data:
                self.loop_box.setChecked(bool(data["loop"]))
            self.log("animation loaded from %s (%d frames)" % (path, len(self.keyframes)))
        except Exception as e:
            self.log("load animation failed: %s" % e)

    # ---------------- close ----------------
    def closeEvent(self, ev):
        self.player.stop()
        self._hs_cancel.set()
        if self._handshake is not None and self._handshake.isRunning():
            self._handshake.wait(8000)
        self.link.close()
        super().closeEvent(ev)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Rainbow Cube 4x4x4 GUI controller")
    ap.add_argument("--port", default=DEFAULT_PORT, help="serial port (default %(default)s)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="baud rate (default %(default)s)")
    ap.add_argument("--connect", action="store_true", help="connect automatically on start")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(port=args.port, baud=args.baud, autostart=args.connect)
    win.show()
    if QT_VERSION == 6:
        sys.exit(app.exec())
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
