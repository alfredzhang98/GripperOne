#!/usr/bin/env python3
"""PySide6 bring-up dashboard for two PaXini CP-M3025 tactile sensors.

Left = ring finger (device address 1), right = little finger (address 2).
The ESP32 polls both 7 x 11 modules at 921600 baud and forwards one complete
77-point map per app_proto frame. USB reporting is opt-in.

    python host/2_haptic_uart.py
    python host/2_haptic_uart.py --selftest
    python host/2_haptic_uart.py --render tmp/haptic_gui.png
"""

import argparse
import functools
import json
import math
import os
import pathlib
import queue
import struct
import sys
import threading
import time
from collections import deque


# app_proto -----------------------------------------------------------------

SOF = b"\xA5\x5A"
VERSION = 0x01
HDR_LEN = 7
MAX_PAYLOAD = 400

T_COMMAND, T_RESPONSE, T_TELEMETRY, T_ACK, T_ERROR = range(5)
TYPE_NAME = {
    T_COMMAND: "CMD", T_RESPONSE: "RESP", T_TELEMETRY: "TELE",
    T_ACK: "ACK", T_ERROR: "ERR",
}
STATUS_NAME = {
    0x00: "OK", 0x01: "ERR_CRC", 0x02: "ERR_VERSION",
    0x03: "ERR_OPCODE", 0x04: "ERR_LOCKED", 0x05: "ERR_STATE",
    0x06: "ERR_ARG",
}
STATE_NAME = {0: "INIT", 1: "IDLE", 2: "ACTIVE", 3: "ERROR", 4: "SAFE_STOP"}
OP = {
    "PING": 0x01,
    "GET_VERSION": 0x02,
    "GET_STATE": 0x03,
    "HAPTIC_STREAM_EN": 0x30,
    "HAPTIC_STREAM_DIS": 0x31,
    "HAPTIC_GET": 0x32,
    "HAPTIC_CALIBRATE": 0x34,
    "HAPTIC_ACQ_EN": 0x35,
    "HAPTIC_ACQ_DIS": 0x36,
}
OP_NAME = {value: name for name, value in OP.items()}

from point_map_m3025 import POINT_XYZ_MM

POINT_COUNT = len(POINT_XYZ_MM)          # 77
POINT_XY = [(x, y) for x, y, _z in POINT_XYZ_MM]
# Slots in the integration board's fixed 28-module order (通信盒协议 4.6):
# thumb 0-3, index 4-7, middle 8-11, ring 12-15, little 16-19, palm 20-27,
# each finger as proximal/middle/fingertip/nail. A CP-M3025 is proximal.
RING_MODULE = 12
LITTLE_MODULE = 16
# haptic_slot_status_t, mirrored from components/app_protocol/include/app_protocol.h
def module_registers(module: int) -> dict:
    """Where one module's data lives on the board (通信盒协议 4.6)."""
    return {
        "conn": (0x0010 + module // 8, module % 8),
        "points": 0x0030 + module * 2,
        "force": 0x0500 + module * 6,
        "dist": 0x1000 + module * 0x200,
    }


SLOT_STATUS = {
    0: "",
    1: "⚠ 该接口未插传感器",
    2: "⚠ 该接口不是 7×11 模组",
    3: "⚠ 板卡识别未完成",
}

# Where each wire value sits on the pad now comes from the vendor coordinate
# sheet (point_map_m3025.py), so nothing about the layout is guessed any more.
# What is still a physical unknown is how the module is MOUNTED relative to the
# viewer, and that is what these three flags cover: mirror in x, mirror in y,
# and swap the axes. They are applied to the real millimetre coordinates.
ORIENT_PATH = pathlib.Path(__file__).with_name("haptic_orientation.json")
DEFAULT_ORIENT = {"transpose": False, "flip_h": False, "flip_v": False}


def load_orientation() -> dict:
    orient = dict(DEFAULT_ORIENT)
    try:
        stored = json.loads(ORIENT_PATH.read_text(encoding="utf-8-sig"))
        for key in DEFAULT_ORIENT:
            orient[key] = bool(stored.get(key, DEFAULT_ORIENT[key]))
    except (OSError, ValueError):
        pass
    return orient


def save_orientation(orient: dict) -> None:
    try:
        ORIENT_PATH.write_text(json.dumps(orient, indent=2), encoding="utf-8")
    except OSError:
        pass


def oriented_xy(index: int, orient: dict) -> tuple:
    """Millimetre position of wire `index` as the viewer sees it."""
    x, y = POINT_XY[index]
    return transform_point(x, y, orient)


def oriented_points(orient: dict) -> list:
    return [oriented_xy(i, orient) for i in range(POINT_COUNT)]


ALL_ORIENTS = [{"transpose": t, "flip_h": fh, "flip_v": fv}
                for t in (False, True) for fh in (False, True) for fv in (False, True)]


def force_centroid(fz) -> tuple:
    """Force-weighted centre of the contact patch, in source (row, col) space.

    The points sit on a ~2.6 mm pitch, so a fingertip always covers a patch of
    them and the single strongest point wanders around inside that patch from
    frame to frame. The centroid does not, which is what makes a press usable
    as a position measurement at all. Returns None if nothing is touching.
    """
    peak = max(fz)
    if peak <= 0:
        return None
    floor = peak * 0.35   # keep the patch, drop the skirt
    weight = x_sum = y_sum = 0.0
    for index, value in enumerate(fz):
        if value < floor:
            continue
        x, y = POINT_XY[index]
        weight += value
        x_sum += value * x
        y_sum += value * y
    if weight <= 0:
        return None
    return x_sum / weight, y_sum / weight


def transform_point(x: float, y: float, orient: dict) -> tuple:
    """Apply the mounting transform to a millimetre position on the pad."""
    if orient["flip_h"]:
        x = -x
    if orient["flip_v"]:
        y = -y
    return (y, x) if orient["transpose"] else (x, y)


def oriented_points(orient: dict) -> list:
    return [transform_point(x, y, orient) for x, y in POINT_XY]


@functools.lru_cache(maxsize=1)
def point_pitch_mm() -> float:
    """Closest spacing between any two points; sets a sensible dot size."""
    best = None
    for i, (x0, y0) in enumerate(POINT_XY):
        for x1, y1 in POINT_XY[i + 1:]:
            d = math.hypot(x1 - x0, y1 - y0)
            if d > 0 and (best is None or d < best):
                best = d
    return best or 1.0


def solve_orientation(corner_points: dict) -> dict:
    """Pick the mounting transform that puts each pressed patch in its corner.

    `corner_points` maps a corner name to the pad-space centroid (mm) measured
    while pressing there. Screen axes are u right, v down; scores are normalised
    by each candidate's own extent so a swapped-axis candidate competes fairly.
    """
    best, best_score = ALL_ORIENTS[0], None
    for orient in ALL_ORIENTS:
        pts = oriented_points(orient)
        min_u = min(u for u, _ in pts); max_u = max(u for u, _ in pts)
        min_v = min(v for _, v in pts); max_v = max(v for _, v in pts)
        span_u = max(max_u - min_u, 1e-6)
        span_v = max(max_v - min_v, 1e-6)
        targets = {"tl": (min_u, min_v), "tr": (max_u, min_v),
                   "bl": (min_u, max_v), "br": (max_u, max_v)}
        score = 0.0
        for name, (x, y) in corner_points.items():
            u, v = transform_point(x, y, orient)
            want_u, want_v = targets[name]
            score += abs(u - want_u) / span_u + abs(v - want_v) / span_v
        if best_score is None or score < best_score:
            best, best_score = orient, score
    return dict(best)


HAPTIC_PAYLOAD_SIZE = 25 + POINT_COUNT * 3


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(frame_type: int, seq: int, payload: bytes) -> bytes:
    body = bytes((VERSION, frame_type, seq & 0xFF)) + struct.pack("<H", len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16(body))


def build_command(op: int, seq: int) -> bytes:
    return build(T_COMMAND, seq, bytes((op,)))


def decode_haptic_telemetry(payload: bytes):
    """Decode one CP-M3025 module frame; return None when malformed."""
    if len(payload) != HAPTIC_PAYLOAD_SIZE or payload[0] != OP["HAPTIC_GET"]:
        return None
    sensor_index, device_addr, status, point_count = payload[1:5]
    if sensor_index not in (0, 1) or point_count != POINT_COUNT:
        return None
    seq, timestamp_ms, fx_sum, fy_sum, fz_sum = struct.unpack_from("<II3f", payload, 5)
    fx, fy, fz = [], [], []
    offset = 25
    for index in range(POINT_COUNT):
        x, y, z = struct.unpack_from("<bbB", payload, offset + index * 3)
        fx.append(x * 0.1)
        fy.append(y * 0.1)
        fz.append(z * 0.1)
    return {
        "sensor_index": sensor_index,
        "device_addr": device_addr,
        "status": status,
        "point_count": point_count,
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "resultant": (fx_sum, fy_sum, fz_sum),
        "fx": fx,
        "fy": fy,
        "fz": fz,
    }


class Parser:
    """Extract CRC-valid app_proto frames from a stream mixed with ESP logs."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames, text = [], bytearray()
        cursor = 0
        while cursor < len(self.buf):
            start = self.buf.find(SOF, cursor)
            if start < 0:
                keep = 1 if self.buf.endswith(b"\xA5") else 0
                text += self.buf[cursor:len(self.buf) - keep]
                cursor = len(self.buf) - keep
                break
            if len(self.buf) - start < HDR_LEN:
                text += self.buf[cursor:start]
                cursor = start
                break
            payload_len = self.buf[start + 5] | (self.buf[start + 6] << 8)
            if self.buf[start + 2] != VERSION or payload_len > MAX_PAYLOAD:
                text += self.buf[cursor:start + 1]
                cursor = start + 1
                continue
            total = HDR_LEN + payload_len + 2
            if len(self.buf) - start < total:
                text += self.buf[cursor:start]
                cursor = start
                break
            frame = bytes(self.buf[start:start + total])
            expected = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16(frame[2:HDR_LEN + payload_len]) != expected:
                text += self.buf[cursor:start + 1]
                cursor = start + 1
                continue
            text += self.buf[cursor:start]
            frames.append({
                "type": frame[3],
                "seq": frame[4],
                "payload": frame[HDR_LEN:HDR_LEN + payload_len],
            })
            cursor = start + total
        del self.buf[:cursor]
        return frames, bytes(text)


def make_demo_sample(sensor_index: int, tick: int = 0):
    """Deterministic contact patch for selftest and screenshot rendering."""
    fx, fy, fz = [], [], []
    # a fingertip pressed on the pad, in real millimetres
    centre_x = -2.0 + sensor_index * 4.0
    centre_y = 11.0 + sensor_index * 5.0
    for x, y in POINT_XY:
        dx, dy = x - centre_x, y - centre_y
        pressure = 17.5 * math.exp(-(dx * dx + dy * dy) / 90.0)
        fz.append(pressure)
        fx.append(-dy * pressure * 0.012)
        fy.append(dx * pressure * 0.016)
    return {
        "sensor_index": sensor_index,
        "device_addr": RING_MODULE if sensor_index == 0 else LITTLE_MODULE,
        "status": 0,
        "point_count": POINT_COUNT,
        "seq": tick + 1,
        "timestamp_ms": tick * 12,
        "resultant": (
            -1.4 + sensor_index * 2.1,
            3.2 - sensor_index * 1.0,
            16.8 - sensor_index * 3.4,
        ),
        "fx": fx,
        "fy": fy,
        "fz": fz,
    }


def encode_demo_payload(sample) -> bytes:
    payload = bytearray((
        OP["HAPTIC_GET"], sample["sensor_index"], sample["device_addr"],
        sample["status"], POINT_COUNT,
    ))
    payload += struct.pack("<II3f", sample["seq"], sample["timestamp_ms"], *sample["resultant"])
    for x, y, z in zip(sample["fx"], sample["fy"], sample["fz"]):
        payload += struct.pack(
            "<bbB", max(-128, min(127, round(x * 10))),
            max(-128, min(127, round(y * 10))),
            max(0, min(255, round(z * 10))),
        )
    return bytes(payload)


# Integration-board wire protocol -------------------------------------------
# The ESP, not this GUI, speaks to the 高速通信集成板. These helpers exist so
# the self-test can pin the firmware's frame layout and register arithmetic to
# the vendor's own documented example frames (通信盒通信协议 4.5.3 / Hand_UI).

def box_frame(func: int, reg: int, num: int, data: bytes = b"") -> str:
    body = (bytes([0x55, 0xAA, 0x00, func]) + reg.to_bytes(2, "little")
            + num.to_bytes(2, "little") + data)
    return (body + bytes([(-sum(body)) & 0xFF])).hex(" ").upper()


def box_addrs(module: int) -> dict:
    """Register block of one module, 通信盒通信协议 4.6."""
    return {
        "force": 0x0500 + module * 6,      # 3 x int16, low byte used, 0.1 N/LSB
        "dist": 0x1000 + module * 0x200,   # POINT_COUNT x 3 bytes
        "points": 0x0030 + module * 2,     # uint16 point count
        "presence": (0x0010 + module // 8, module % 8),
    }


BOX_VECTORS = [
    ("read version", box_frame(0x03, 0x0000, 15), "55 AA 00 03 00 00 0F 00 EF"),
    ("read module map", box_frame(0x03, 0x0010, 4), "55 AA 00 03 10 00 04 00 EA"),
    ("read point counts", box_frame(0x03, 0x0030, 0x38), "55 AA 00 03 30 00 38 00 96"),
    ("thumb proximal force", box_frame(0x03, 0x0500, 6), "55 AA 00 03 00 05 06 00 F3"),
    ("thumb middle force", box_frame(0x03, 0x0506, 6), "55 AA 00 03 06 05 06 00 ED"),
    ("data type force+dist", box_frame(0x10, 0x0016, 1, bytes([0x03])),
     "55 AA 00 10 16 00 01 00 03 D7"),
    ("auto-post on", box_frame(0x10, 0x0017, 1, bytes([0x01])),
     "55 AA 00 10 17 00 01 00 01 D8"),
    ("auto-post off", box_frame(0x10, 0x0017, 1, bytes([0x00])),
     "55 AA 00 10 17 00 01 00 00 D9"),
    ("calibrate", box_frame(0x17, 0x0002, 1, bytes([0x01])),
     "55 AA 00 17 02 00 01 00 01 E6"),
]


def selftest() -> int:
    failures = []

    def check(name, got, expected):
        if got != expected:
            failures.append(f"{name}: got {got!r}, expected {expected!r}")

    check("CRC vector", crc16(b"123456789"), 0x29B1)
    check("payload size", HAPTIC_PAYLOAD_SIZE, 256)
    parser = Parser()
    wire = b"I (1) boot\n" + build(T_TELEMETRY, 9, encode_demo_payload(make_demo_sample(0, 8)))
    frames = []
    for chunk in (wire[:17], wire[17:101], wire[101:]):
        new_frames, _ = parser.feed(chunk)
        frames.extend(new_frames)
    check("fragmented frame count", len(frames), 1)
    if frames:
        sample = decode_haptic_telemetry(frames[0]["payload"])
        check("sensor index", sample["sensor_index"] if sample else None, 0)
        check("module slot", sample["device_addr"] if sample else None, RING_MODULE)
        check("point count", len(sample["fz"]) if sample else 0, 77)
        check("signed shear", min(sample["fx"]) < 0 if sample else False, True)

    bad = bytearray(build_command(OP["PING"], 7))
    bad[-1] ^= 0xFF
    parser = Parser()
    frames, _ = parser.feed(bytes(bad) + build_command(OP["PING"], 8))
    check("bad CRC resync", [frame["seq"] for frame in frames], [8])

    for name, got, expected in BOX_VECTORS:
        check(f"box frame {name}", got, expected)
    ring, little = box_addrs(RING_MODULE), box_addrs(LITTLE_MODULE)
    check("ring dist addr", ring["dist"], 0x2800)
    check("ring force addr", ring["force"], 0x0548)
    check("ring points addr", ring["points"], 0x0048)
    check("ring presence bit", ring["presence"], (0x0011, 4))
    check("little dist addr", little["dist"], 0x3000)
    check("little force addr", little["force"], 0x0560)
    check("little points addr", little["points"], 0x0050)
    check("little presence bit", little["presence"], (0x0012, 0))

    # The coordinate table is the ground truth for where each wire value sits.
    check("point count", POINT_COUNT, 77)
    check("distinct positions", len(set(POINT_XY)), POINT_COUNT)
    span_x = max(x for x, _ in POINT_XY) - min(x for x, _ in POINT_XY)
    span_y = max(y for _, y in POINT_XY) - min(y for _, y in POINT_XY)
    check("pad width mm", round(span_x, 1), 25.3)
    check("pad length mm", round(span_y, 1), 30.6)
    # nine columns: 7 points on each outer one, 9 on the seven inner ones
    columns = {}
    for x, _y in POINT_XY:
        columns[round(x, 1)] = columns.get(round(x, 1), 0) + 1
    check("column count", len(columns), 9)
    check("points per column", sorted(columns.values()), [7, 7] + [9] * 7)

    # Every mounting transform must keep the 77 positions distinct, and the
    # centroid of a patch must land inside the pad it was synthesised on.
    for orient in ALL_ORIENTS:
        pts = oriented_points(orient)
        check(f"transform keeps points distinct {orient}", len(set(pts)), POINT_COUNT)

    def press_patch(cx, cy, radius_mm=4.0):
        """fz field for a fingertip centred at (cx, cy) mm on the pad."""
        field = [0.0] * POINT_COUNT
        for index, (x, y) in enumerate(POINT_XY):
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if d2 <= (radius_mm * 2.5) ** 2:
                field[index] = 200.0 * math.exp(-d2 / (2 * radius_mm ** 2))
        return field

    centre = force_centroid(press_patch(0.0, 14.0))
    check("centroid near press", centre is not None
          and abs(centre[0]) < 2.0 and abs(centre[1] - 14.0) < 2.5, True)

    # Three-corner solve, in real millimetres, with the press deliberately set
    # in from each corner and spread over several points the way a finger is.
    min_x = min(x for x, _ in POINT_XY); max_x = max(x for x, _ in POINT_XY)
    min_y = min(y for _, y in POINT_XY); max_y = max(y for _, y in POINT_XY)
    for inset in (3.0, 5.0, 7.0):
        for radius_mm in (3.0, 5.0):
            for truth in ALL_ORIENTS:
                pts = oriented_points(truth)
                lo_u = min(u for u, _ in pts); hi_u = max(u for u, _ in pts)
                lo_v = min(v for _, v in pts); hi_v = max(v for _, v in pts)
                wanted = {"tl": (lo_u + inset, lo_v + inset),
                          "tr": (hi_u - inset, lo_v + inset),
                          "bl": (lo_u + inset, hi_v - inset)}
                measured = {}
                for name, (want_u, want_v) in wanted.items():
                    # find the pad point nearest that screen position, then
                    # synthesise the press around its true pad coordinates
                    best = min(range(POINT_COUNT),
                               key=lambda i: (pts[i][0] - want_u) ** 2
                               + (pts[i][1] - want_v) ** 2)
                    px, py = POINT_XY[best]
                    measured[name] = force_centroid(press_patch(px, py, radius_mm))
                check(f"solve inset={inset} r={radius_mm} {truth}",
                      solve_orientation(measured), truth)

    if failures:
        print("SELFTEST FAILED")
        for failure in failures:
            print(" -", failure)
        return 1
    print("selftest OK: CRC, stream parser, 256-byte/77-point tactile payload, "
          "integration-board frames, register map, the 77-point coordinate map "
          "and its 8 mounting transforms, patch-centroid corner solver")
    return 0


# Serial worker -------------------------------------------------------------

class SerialLink:
    def __init__(self, output_queue):
        self.output_queue = output_queue
        self.ser = None
        self.parser = Parser()
        self.thread = None
        self.running = False
        self.command_seq = 0
        self.write_lock = threading.Lock()

    @property
    def connected(self):
        return self.ser is not None and self.running

    def open(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.05, dsrdtr=False, rtscts=False)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except OSError:
            pass
        self.parser = Parser()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, name="haptic-usb-reader", daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.cancel_read()
            except (AttributeError, OSError):
                pass
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except OSError:
                pass
        self.ser = None

    def send(self, op):
        if not self.connected:
            return None
        with self.write_lock:
            self.command_seq = (self.command_seq + 1) & 0xFF
            self.ser.write(build_command(op, self.command_seq))
            return self.command_seq

    def _read_loop(self):
        while self.running and self.ser:
            try:
                data = self.ser.read(4096)
            except Exception as exc:
                if self.running:
                    self.output_queue.put(("error", str(exc)))
                return
            if not data:
                continue
            frames, text = self.parser.feed(data)
            received_at = time.monotonic()
            for frame in frames:
                self.output_queue.put(("frame", received_at, frame))
            if text:
                self.output_queue.put(("text", text.decode("utf-8", "replace")))


# PySide6 dashboard ---------------------------------------------------------

def run_gui(render_path=None) -> int:
    try:
        from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
        from PySide6.QtGui import (QColor, QFont, QFontDatabase, QPainter,
                                   QPainterPath, QPen)
        from PySide6.QtWidgets import (
            QApplication, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
            QCheckBox, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
            QScrollArea,
            QSizePolicy,
            QVBoxLayout, QWidget,
        )
        import serial.tools.list_ports
    except ImportError as exc:
        print(f"Missing GUI dependency: {exc}", file=sys.stderr)
        print("Install with: pip install -r host/requirements.txt", file=sys.stderr)
        return 1

    TEXT = QColor("#172033")
    MUTED = QColor("#64748b")
    CYAN = QColor("#0284c7")
    ORANGE = QColor("#e56f00")
    GREEN = QColor("#07875c")
    RED = QColor("#d83a52")

    def mix(c0, c1, amount):
        amount = max(0.0, min(1.0, amount))
        return QColor(
            round(c0.red() + (c1.red() - c0.red()) * amount),
            round(c0.green() + (c1.green() - c0.green()) * amount),
            round(c0.blue() + (c1.blue() - c0.blue()) * amount),
        )

    def force_color(value):
        t = max(0.0, min(1.0, value / 25.0))
        stops = [
            (0.0, QColor("#d9f2f4")), (0.08, QColor("#15aabf")),
            (0.35, QColor("#22c55e")), (0.62, QColor("#facc15")),
            (1.0, QColor("#ef4444")),
        ]
        for (a, ca), (b, cb) in zip(stops, stops[1:]):
            if t <= b:
                return mix(ca, cb, (t - a) / (b - a))
        return stops[-1][1]

    class MetricCard(QFrame):
        def __init__(self, caption, color):
            super().__init__()
            self.setObjectName("metricCard")
            layout = QVBoxLayout(self)
            layout.setContentsMargins(12, 7, 12, 7)
            caption_label = QLabel(caption)
            caption_label.setObjectName("metricCaption")
            self.value = QLabel("—")
            self.value.setStyleSheet(f"font-size: 18px; font-weight: 800; color: {color.name()};")
            layout.addWidget(caption_label)
            layout.addWidget(self.value)

    class ForceMap(QWidget):
        def __init__(self, orient):
            super().__init__()
            self.sample = None
            self.orient = orient
            self.setMinimumSize(230, 430)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        def set_sample(self, sample):
            self.sample = sample
            self.update()

        def paintEvent(self, _event):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor("#ffffff"))
            w, h = self.width(), self.height()

            # Points are drawn where the vendor coordinate sheet says they
            # physically sit, to scale. The pad is a curved 25.3 x 30.6 mm
            # fingertip surface of nine columns - 7 points on each outer column,
            # 9 on the inner seven - not a rectangular grid, so laying it out as
            # rows and columns is what put pressed positions in the wrong place.
            pts = oriented_points(self.orient)
            min_u = min(u for u, _ in pts)
            max_u = max(u for u, _ in pts)
            min_v = min(v for _, v in pts)
            max_v = max(v for _, v in pts)
            span_u = max(max_u - min_u, 1e-6)
            span_v = max(max_v - min_v, 1e-6)

            top, bottom, side = 26.0, 52.0, 12.0
            pitch = point_pitch_mm()
            scale = min((w - 2 * side) / (span_u + pitch),
                        (h - top - bottom) / (span_v + pitch))
            radius = max(2.0, scale * pitch * 0.34)
            half_u = scale * (span_u + pitch) / 2
            half_v = scale * (span_v + pitch) / 2
            cx0 = w / 2
            cy0 = top + (h - top - bottom) / 2
            mid_u = (min_u + max_u) / 2
            mid_v = (min_v + max_v) / 2

            def to_screen(u, v):
                return cx0 + (u - mid_u) * scale, cy0 + (v - mid_v) * scale

            area = QRectF(cx0 - half_u, cy0 - half_v, 2 * half_u, 2 * half_v)
            corner = min(area.width(), area.height()) * 0.22
            shape = QPainterPath()
            shape.addRoundedRect(area, corner, corner)
            p.fillPath(shape, QColor("#f7fafc"))
            p.setPen(QPen(QColor("#b9c8d8"), 2))
            p.drawPath(shape)

            p.setPen(QPen(MUTED, 1))
            p.setFont(QFont(UI_FONT, 8, QFont.DemiBold))
            p.drawText(QRectF(0, 4, w, 18), Qt.AlignCenter,
                       f"{POINT_COUNT} 点 · {span_u:.1f} × {span_v:.1f} mm · 实测坐标")
            p.drawText(QRectF(0, h - 24, w, 18), Qt.AlignCenter, "CABLE SIDE")

            sample = self.sample
            for index, (u, v) in enumerate(pts):
                cx, cy = to_screen(u, v)
                fz = sample["fz"][index] if sample else 0.0
                p.setPen(QPen(QColor("#ffffff"), 1))
                p.setBrush(force_color(fz))
                p.drawEllipse(QPointF(cx, cy), radius, radius)
                if sample and fz > 0.25:
                    sx, sy = transform_point(sample["fx"][index],
                                             sample["fy"][index], self.orient)
                    ex = cx + sx * radius * 0.30
                    ey = cy - sy * radius * 0.30
                    p.setPen(QPen(QColor("#273449"), 1.4))
                    p.drawLine(QPointF(cx, cy), QPointF(ex, ey))
                    angle = math.atan2(ey - cy, ex - cx)
                    wing = max(2.0, radius * 0.45)
                    p.drawLine(QPointF(ex, ey), QPointF(
                        ex - wing * math.cos(angle - 0.55),
                        ey - wing * math.sin(angle - 0.55)))
                    p.drawLine(QPointF(ex, ey), QPointF(
                        ex - wing * math.cos(angle + 0.55),
                        ey - wing * math.sin(angle + 0.55)))

            # Mark the force-weighted centre of contact rather than the single
            # strongest point: a fingertip covers a patch and the strongest
            # point jitters around inside it. This centre is a position in mm.
            centre = force_centroid(sample["fz"]) if sample else None
            if centre is not None and max(sample["fz"]) > 2.5:
                cu, cv = transform_point(centre[0], centre[1], self.orient)
                px, py = to_screen(cu, cv)
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(QColor("#d83a52"), 2))
                p.drawEllipse(QPointF(px, py), radius * 2.1, radius * 2.1)
                p.setPen(QPen(QColor("#d83a52"), 1))
                p.setFont(QFont(UI_FONT, 8, QFont.DemiBold))
                p.drawText(QRectF(0, 20, w - 6, 16), Qt.AlignRight,
                           f"接触质心 X{centre[0]:+.1f} Y{centre[1]:+.1f} mm · "
                           f"合计 {sum(sample['fz']) * 0.1:.1f} N")

            bar = QRectF(w * 0.18, h - 40, w * 0.64, 8)
            segments = 60
            for i in range(segments):
                x0 = bar.left() + bar.width() * i / segments
                p.fillRect(QRectF(x0, bar.top(), bar.width() / segments + 1, bar.height()),
                           force_color(25.0 * i / (segments - 1)))
            p.setPen(MUTED)
            p.drawText(QRectF(bar.left() - 14, bar.top() - 3, 12, 16), Qt.AlignRight, "0")
            p.drawText(QRectF(bar.right() + 3, bar.top() - 3, 24, 16), Qt.AlignLeft, "25 N")

    class Scope(QWidget):
        def __init__(self):
            super().__init__()
            self.times = deque(maxlen=900)
            self.values = [deque(maxlen=900) for _ in range(3)]
            self.setMinimumHeight(200)

        def append(self, stamp, resultant):
            self.times.append(stamp)
            for series, value in zip(self.values, resultant):
                series.append(value)
            self.update()

        def clear(self):
            self.times.clear()
            for series in self.values:
                series.clear()
            self.update()

        def paintEvent(self, _event):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor("#fbfdff"))
            plot = QRectF(42, 22, self.width() - 54, self.height() - 42)
            p.setPen(QPen(QColor("#e1e8f0"), 1))
            for i in range(5):
                y = plot.top() + plot.height() * i / 4
                p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setFont(QFont(UI_FONT, 8))
            p.setPen(MUTED)
            p.drawText(QRectF(8, 2, self.width() - 16, 18), Qt.AlignLeft,
                       "合力趋势   Fx     Fy     Fz")
            if len(self.times) < 2:
                p.drawText(plot, Qt.AlignCenter, "等待触觉数据")
                return
            latest = self.times[-1]
            first = max(self.times[0], latest - 10.0)
            visible = [i for i, t in enumerate(self.times) if t >= first]
            all_values = [self.values[j][i] for j in range(3) for i in visible]
            lo = min(min(all_values), 0.0)
            hi = max(max(all_values), 0.0)
            margin = max((hi - lo) * 0.12, 0.5)
            lo, hi = lo - margin, hi + margin
            span_t = max(latest - first, 0.01)
            colors = (CYAN, ORANGE, RED)
            for axis, color in enumerate(colors):
                path = QPainterPath()
                for count, index in enumerate(visible):
                    x = plot.left() + (self.times[index] - first) / span_t * plot.width()
                    y = plot.bottom() - (self.values[axis][index] - lo) / (hi - lo) * plot.height()
                    if count == 0:
                        path.moveTo(x, y)
                    else:
                        path.lineTo(x, y)
                p.setPen(QPen(color, 1.8))
                p.drawPath(path)
            p.setPen(MUTED)
            p.drawText(QRectF(2, plot.top() - 7, 37, 16), Qt.AlignRight, f"{hi:.1f}")
            p.drawText(QRectF(2, plot.bottom() - 8, 37, 16), Qt.AlignRight, f"{lo:.1f}")

    class SensorPanel(QFrame):
        def __init__(self, title, subtitle, accent, orient):
            super().__init__()
            self.setObjectName("sensorPanel")
            outer = QVBoxLayout(self)
            outer.setContentsMargins(14, 12, 14, 12)
            heading = QHBoxLayout()
            title_label = QLabel(title)
            title_label.setStyleSheet(f"font-size: 18px; font-weight: 800; color: {accent.name()};")
            self.status_label = QLabel(subtitle)
            self.status_label.setObjectName("sensorStatus")
            self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            heading.addWidget(title_label)
            heading.addStretch(1)
            heading.addWidget(self.status_label)
            outer.addLayout(heading)

            cards = QHBoxLayout()
            self.force_cards = []
            for axis, color in zip(("Fx", "Fy", "Fz"), (CYAN, ORANGE, RED)):
                card = MetricCard(f"合力 {axis}", color)
                card.value.setText("+0.0 N")
                self.force_cards.append(card)
                cards.addWidget(card)
            outer.addLayout(cards)
            body = QHBoxLayout()
            body.setSpacing(10)
            self.map = ForceMap(orient)
            body.addWidget(self.map, 3)
            self.scope = Scope()
            body.addWidget(self.scope, 4)
            outer.addLayout(body, 1)

        def set_sample(self, sample, host_stamp):
            reg = module_registers(sample["device_addr"])
            self.map.set_sample(sample)
            for card, value in zip(self.force_cards, sample["resultant"]):
                card.value.setText(f"{value:+.1f} N")
            self.status_label.setText(
                f"模组 {sample['device_addr']}  ·  分布力 0x{reg['dist']:04X}  ·  "
                f"合力 0x{reg['force']:04X}  ·  点数 0x{reg['points']:04X}  ·  "
                f"seq {sample['seq']}")
            warning = SLOT_STATUS.get(sample["status"], f"⚠ 状态 {sample['status']}")
            conn_reg, conn_bit = reg["conn"]
            self.status_label.setStyleSheet(
                "color: #d83a52; font-weight: 700;" if warning else "")
            if warning:
                self.status_label.setText(
                    f"模组 {sample['device_addr']}  ·  连接位 0x{conn_reg:04X} "
                    f"bit{conn_bit}  ·  {warning}")
            self.scope.append(host_stamp, sample["resultant"])

        def clear(self):
            self.map.set_sample(None)
            self.scope.clear()
            for card in self.force_cards:
                card.value.setText("+0.0 N")

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("CP-M3025 双指触觉验证")
            self.resize(1580, 1060)
            self.setMinimumSize(1100, 640)
            self.output_queue = queue.Queue()
            self.link = SerialLink(self.output_queue)
            self.text_buffer = ""
            self.orient = load_orientation()
            self.cal_steps = None   # guided point-map calibration, see _cal_*
            self.cal_step = 0
            self.cal_hits = {}
            self.cal_armed = False
            self.cal_best = None
            self.stream_active = False
            self.acq_active = None
            self.last_seq = [None, None]
            self.total = [0, 0]
            self.window_count = [0, 0]
            self.lost = [0, 0]
            self.rate_started = time.monotonic()
            self._build_ui()
            self._refresh_ports()

            self.pump_timer = QTimer(self)
            self.pump_timer.timeout.connect(self._pump)
            self.pump_timer.start(8)
            self.rate_timer = QTimer(self)
            self.rate_timer.timeout.connect(self._update_rates)
            self.rate_timer.start(1000)

        def _build_ui(self):
            root = QWidget()
            scroller = QScrollArea()
            scroller.setWidgetResizable(True)
            scroller.setFrameShape(QFrame.NoFrame)
            scroller.setWidget(root)
            self.setCentralWidget(scroller)
            outer = QVBoxLayout(root)
            outer.setContentsMargins(14, 12, 14, 12)
            outer.setSpacing(9)

            header = QHBoxLayout()
            title_box = QVBoxLayout()
            title = QLabel("CP-M3025 · DUAL TACTILE ARRAY")
            title.setObjectName("title")
            subtitle = QLabel("PaXini PXSR-STDCP03B  ·  2 × 7×11 points  ·  sensor UART 921600 baud")
            subtitle.setObjectName("subtitle")
            title_box.addWidget(title)
            title_box.addWidget(subtitle)
            header.addLayout(title_box)
            header.addStretch(1)
            self.port_combo = QComboBox()
            refresh = QPushButton("刷新串口")
            refresh.clicked.connect(self._refresh_ports)
            self.connect_button = QPushButton("连接")
            self.connect_button.setObjectName("primaryButton")
            self.connect_button.clicked.connect(self._toggle_connection)
            self.connection_label = QLabel("● DISCONNECTED")
            self.connection_label.setObjectName("disconnected")
            header.addWidget(self.port_combo)
            header.addWidget(refresh)
            header.addWidget(self.connect_button)
            header.addWidget(self.connection_label)
            outer.addLayout(header)

            info = QHBoxLayout()
            self.ring_rate = MetricCard("无名指实际帧率", GREEN)
            self.little_rate = MetricCard("小拇指实际帧率", CYAN)
            self.loss_card = MetricCard("序号丢帧（两指）", RED)
            self.total_card = MetricCard("累计完整点阵", ORANGE)
            for card in (self.ring_rate, self.little_rate, self.loss_card, self.total_card):
                info.addWidget(card)
            outer.addLayout(info)

            sensors = QHBoxLayout()
            sensors.setSpacing(10)
            self.panels = [
                SensorPanel("无名指 · RING", "左侧 · 模组 12 · 近节", GREEN, self.orient),
                SensorPanel("小拇指 · LITTLE", "右侧 · 模组 16 · 近节", CYAN, self.orient),
            ]
            for panel in self.panels:
                sensors.addWidget(panel, 1)
            outer.addLayout(sensors, 1)

            controls = QGridLayout()
            button_specs = [
                ("开始 USB 触觉上报", "HAPTIC_STREAM_EN"),
                ("停止 USB 触觉上报", "HAPTIC_STREAM_DIS"),
                ("开始传感器→ESP 轮询", "HAPTIC_ACQ_EN"),
                ("停止传感器→ESP 轮询", "HAPTIC_ACQ_DIS"),
                ("读取两指最近一帧", "HAPTIC_GET"),
                ("两指空载标定", "HAPTIC_CALIBRATE"),
                ("Ping", "PING"),
                ("读取系统状态", "GET_STATE"),
            ]
            for index, (label, name) in enumerate(button_specs):
                button = QPushButton(label)
                if name == "HAPTIC_CALIBRATE":
                    button.clicked.connect(self._calibrate)
                else:
                    button.clicked.connect(lambda _checked=False, op_name=name: self._send(op_name))
                controls.addWidget(button, index // 4, index % 4)
            clear = QPushButton("清空曲线与计数")
            clear.clicked.connect(self._clear)
            controls.addWidget(clear, 2, 0, 1, 2)
            self.acq_label = QLabel("传感器→ESP：等待状态")
            self.stream_label = QLabel("USB 触觉流：已停止")
            controls.addWidget(self.acq_label, 2, 2)
            controls.addWidget(self.stream_label, 2, 3)
            outer.addLayout(controls)

            # Point-map orientation. The manual never states how the 77 wire
            # values are laid out on the pad, so rather than hardcode a guess:
            # press one corner, watch the 峰值 marker, tick until they agree.
            orient_row = QHBoxLayout()
            orient_row.addWidget(QLabel("点阵映射："))
            self.orient_boxes = {}
            for key, text, tip in (
                ("transpose", "轴交换 (X ⇄ Y)", "把模组横过来看"),
                ("flip_h", "左右镜像", "沿垂直轴镜像"),
                ("flip_v", "上下镜像", "沿水平轴镜像"),
            ):
                box = QCheckBox(text)
                box.setToolTip(tip)
                box.setChecked(self.orient[key])
                box.toggled.connect(
                    lambda checked, k=key: self._set_orient(k, checked))
                self.orient_boxes[key] = box
                orient_row.addWidget(box)
            reset = QPushButton("复位映射")
            reset.setToolTip("回到坐标表原始朝向，不做任何镜像")
            reset.clicked.connect(self._reset_orient)
            orient_row.addWidget(reset)
            orient_row.addStretch(1)
            self.cal_button = QPushButton("三点自动标定映射")
            self.cal_button.setObjectName("primaryButton")
            self.cal_button.clicked.connect(self._cal_start)
            orient_row.addWidget(self.cal_button)
            self.cal_label = QLabel("按住一个角，红圈应落在同一个角")
            orient_row.addWidget(self.cal_label)
            outer.addLayout(orient_row)

            self.log = QPlainTextEdit()
            self.log.setFont(QFont(MONO_FONT, 9))
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(1000)
            self.log.setMaximumHeight(105)
            self.log.setPlaceholderText("设备日志、ACK 与错误信息")
            outer.addWidget(self.log)

            self.setStyleSheet("""
                QWidget { background: #ffffff; color: #172033; font-size: 10pt; }
                QFrame#sensorPanel, QFrame#metricCard { background: #ffffff; border: 1px solid #d8e1eb; border-radius: 9px; }
                QLabel#title { font-size: 21px; font-weight: 800; color: #10243e; letter-spacing: 1px; }
                QLabel#subtitle, QLabel#metricCaption, QLabel#sensorStatus { color: #64748b; font-size: 9pt; }
                QLabel#connected { color: #07875c; font-weight: 700; }
                QLabel#disconnected { color: #d83a52; font-weight: 700; }
                QPushButton, QComboBox { background: #ffffff; border: 1px solid #bdcad8; border-radius: 6px;
                                        padding: 7px 10px; min-height: 18px; }
                QPushButton:hover { background: #eef7ff; border-color: #0284c7; }
                QPushButton#primaryButton { background: #e0f2fe; border-color: #0284c7; color: #075985; font-weight: 700; }
                QPlainTextEdit { background: #fbfdff; border: 1px solid #d8e1eb; border-radius: 7px;
                                 color: #526579; font-size: 9pt; padding: 5px; }
            """)

        def _refresh_ports(self):
            previous = self.port_combo.currentData()
            self.port_combo.clear()
            for port in serial.tools.list_ports.comports():
                self.port_combo.addItem(f"{port.device}  ·  {port.description}", port.device)
            if previous:
                index = self.port_combo.findData(previous)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)

        def _toggle_connection(self):
            if self.link.connected:
                self._disconnect()
                return
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "没有串口", "请先选择 XIAO 对应的 COM 端口。")
                return
            try:
                self.link.open(port)
            except Exception as exc:
                QMessageBox.critical(self, "连接失败", f"{port}\n\n{exc}")
                return
            self.connect_button.setText("断开")
            self.connection_label.setText(f"● CONNECTED  {port}")
            self.connection_label.setObjectName("connected")
            self.connection_label.style().unpolish(self.connection_label)
            self.connection_label.style().polish(self.connection_label)
            self.stream_active = False
            self.stream_label.setText("USB 触觉流：已停止")
            self._append_log("--- 已连接；USB 触觉上报保持关闭，需手动点击开始 ---")

        def _disconnect(self):
            self.link.close()
            self.connect_button.setText("连接")
            self.connection_label.setText("● DISCONNECTED")
            self.connection_label.setObjectName("disconnected")
            self.connection_label.style().unpolish(self.connection_label)
            self.connection_label.style().polish(self.connection_label)
            self.stream_active = False
            self.acq_active = None
            self.stream_label.setText("USB 触觉流：已停止")
            self.acq_label.setText("传感器→ESP：等待状态")

        def _send(self, name):
            if not self.link.connected:
                QMessageBox.warning(self, "尚未连接", "请先连接 XIAO。")
                return
            if name == "HAPTIC_ACQ_DIS" and self.stream_active:
                self._append_log("REMINDER: USB 流开关仍为开启；停止轮询后不会再有新数据。")
            if name == "HAPTIC_STREAM_EN" and self.acq_active is False:
                self._append_log("REMINDER: 当前轮询已停止，没有两指新鲜数据时设备会返回 ERR_STATE。")
            seq = self.link.send(OP[name])
            self._append_log(f"TX seq={seq:3d}  {name}")

        def _calibrate(self):
            answer = QMessageBox.question(
                self, "两指空载标定",
                "请确保无名指和小拇指传感器都完全没有受力。\n\n"
                "将向两个模组写入手册规定的标定触发值，是否继续？")
            if answer == QMessageBox.Yes:
                self._send("HAPTIC_CALIBRATE")

        def _append_log(self, message):
            for line in message.splitlines():
                if line.strip():
                    self.log.appendPlainText(line.rstrip())

        def _accept_sample(self, sample, received_at):
            index = sample["sensor_index"]
            previous = self.last_seq[index]
            if previous is not None:
                delta = (sample["seq"] - previous) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    self.lost[index] += delta - 1
            self.last_seq[index] = sample["seq"]
            self.total[index] += 1
            self.window_count[index] += 1
            self.panels[index].set_sample(sample, received_at)
            if self.cal_steps is not None:
                self._cal_feed(sample)
            self.acq_active = True
            self.acq_label.setText("传感器→ESP：运行中")

        def _on_frame(self, received_at, frame):
            payload = frame["payload"]
            op = payload[0] if payload else None
            name = OP_NAME.get(op, f"0x{op:02X}" if op is not None else "?")
            if frame["type"] == T_TELEMETRY and op == OP["HAPTIC_GET"]:
                sample = decode_haptic_telemetry(payload)
                if sample is None:
                    self._append_log(f"RX malformed haptic telemetry ({len(payload)} bytes)")
                    return
                self._accept_sample(sample, received_at)
                return
            if frame["type"] in (T_ACK, T_ERROR):
                status = STATUS_NAME.get(payload[1], f"0x{payload[1]:02X}") if len(payload) > 1 else "?"
                self._append_log(f"RX seq={frame['seq']:3d} {TYPE_NAME[frame['type']]} {name} -> {status}")
                if frame["type"] == T_ACK and len(payload) > 1 and payload[1] == 0:
                    if op == OP["HAPTIC_STREAM_EN"]:
                        self.stream_active = True
                        self.stream_label.setText("USB 触觉流：运行中")
                    elif op == OP["HAPTIC_STREAM_DIS"]:
                        self.stream_active = False
                        self.stream_label.setText("USB 触觉流：已停止（轮询不变）")
                    elif op == OP["HAPTIC_ACQ_EN"]:
                        self.acq_active = True
                        self.acq_label.setText("传感器→ESP：已请求运行")
                    elif op == OP["HAPTIC_ACQ_DIS"]:
                        self.acq_active = False
                        self.acq_label.setText("传感器→ESP：已停止")
            elif frame["type"] == T_RESPONSE and op == OP["GET_STATE"] and len(payload) > 1:
                self._append_log(f"RX STATE = {STATE_NAME.get(payload[1], payload[1])}")
            elif frame["type"] == T_RESPONSE and op == OP["GET_VERSION"] and len(payload) >= 4:
                self._append_log(f"RX VERSION proto={payload[1]} firmware={payload[2]}.{payload[3]}")
            else:
                self._append_log(f"RX {TYPE_NAME.get(frame['type'], frame['type'])} {name}")

        def _on_text(self, chunk):
            self.text_buffer += chunk
            lines = self.text_buffer.split("\n")
            self.text_buffer = lines.pop()
            for line in lines:
                if line.strip():
                    self._append_log(line.rstrip("\r"))

        def _pump(self):
            for _ in range(5000):
                try:
                    item = self.output_queue.get_nowait()
                except queue.Empty:
                    break
                if item[0] == "frame":
                    self._on_frame(item[1], item[2])
                elif item[0] == "text":
                    self._on_text(item[1])
                elif item[0] == "error":
                    self._append_log(f"SERIAL ERROR: {item[1]}")
                    self._disconnect()

        def _update_rates(self):
            now = time.monotonic()
            elapsed = max(now - self.rate_started, 1e-3)
            self.ring_rate.value.setText(f"{self.window_count[0] / elapsed:5.1f} Hz")
            self.little_rate.value.setText(f"{self.window_count[1] / elapsed:5.1f} Hz")
            self.loss_card.value.setText(str(sum(self.lost)))
            self.total_card.value.setText(f"{sum(self.total):,}")
            self.window_count = [0, 0]
            self.rate_started = now

        # ---- guided point-map calibration --------------------------------
        # Neither the manual nor the vendor sample code says how the 77 wire
        # values sit on the pad, so it gets measured: press three known
        # corners, and the transform that puts them there is the answer.
        CAL_PRESS_N = 2.0      # clear of noise; a fingertip easily exceeds it
        CAL_RELEASE_N = 0.5

        def _cal_start(self):
            if self.cal_steps is not None:
                self._cal_finish(None)
                return
            self.cal_steps = [
                ("tl", "左上"), ("tr", "右上"), ("bl", "左下"),
            ]
            self.cal_step = 0
            self.cal_hits = {}
            self.cal_armed = False
            self.cal_best = None
            self.cal_button.setText("取消标定")
            self._append_log(
                "点阵标定开始：用指尖压住【无名指】模组的一个角区域（不用对准单个点），"
                f"力量 >{self.CAL_PRESS_N:.0f} N，然后完全松开；三个角依次做。")
            self._cal_prompt()

        def _cal_prompt(self):
            _, name = self.cal_steps[self.cal_step]
            self.cal_label.setText(
                f"标定 {self.cal_step + 1}/3：压住 <b>{name}角</b> 后松开")
            self.cal_label.setTextFormat(Qt.RichText)

        def _cal_feed(self, sample):
            if sample["sensor_index"] != 0:   # calibrate on the ring panel
                return
            total = sum(sample["fz"]) * 0.1
            if not self.cal_armed:
                if total < self.CAL_RELEASE_N:
                    self.cal_armed = True
                return

            if total >= self.CAL_PRESS_N:
                # Track the hardest frame of this press; its patch is the most
                # clearly defined, and taking it on release avoids committing
                # to whatever the finger touched on the way down.
                if self.cal_best is None or total > self.cal_best[0]:
                    centre = force_centroid(sample["fz"])
                    if centre is not None:
                        self.cal_best = (total, centre)
                return

            if self.cal_best is None or total > self.CAL_RELEASE_N:
                return                      # still pressing, or never pressed

            key, name = self.cal_steps[self.cal_step]
            peak_total, centre = self.cal_best
            self.cal_hits[key] = centre
            self._append_log(
                f"  {name}角 -> 质心 行{centre[0]:.1f} 列{centre[1]:.1f} "
                f"(峰值合计 {peak_total:.1f} N)")
            self.cal_step += 1
            self.cal_armed = False
            self.cal_best = None
            if self.cal_step >= len(self.cal_steps):
                self._cal_finish(solve_orientation(self.cal_hits))
            else:
                self._cal_prompt()

        def _cal_finish(self, orient):
            self.cal_steps = None
            self.cal_button.setText("三点自动标定映射")
            self.cal_label.setText("按住一个角，红圈应落在同一个角")
            self.cal_label.setTextFormat(Qt.PlainText)
            if orient is None:
                self._append_log("点阵标定已取消")
                return
            self._apply_orient(orient, "点阵标定完成（已保存）")

        def _reset_orient(self):
            self._apply_orient(dict(DEFAULT_ORIENT), "点阵映射已复位为默认")

        def _apply_orient(self, orient, message):
            for key, value in orient.items():
                self.orient_boxes[key].blockSignals(True)
                self.orient_boxes[key].setChecked(value)
                self.orient_boxes[key].blockSignals(False)
            self.orient.update(orient)
            save_orientation(self.orient)
            for panel in self.panels:
                panel.map.update()
            self._append_log(f"{message}：轴交换={orient['transpose']}  "
                             f"左右镜像={orient['flip_h']}  上下镜像={orient['flip_v']}")

        def _set_orient(self, key, checked):
            self.orient[key] = bool(checked)
            save_orientation(self.orient)
            for panel in self.panels:
                panel.map.update()
            self._append_log(
                f"点阵映射：轴交换={'是' if self.orient['transpose'] else '否'}  "
                f"左右镜像={'是' if self.orient['flip_h'] else '否'}  "
                f"上下镜像={'是' if self.orient['flip_v'] else '否'}")

        def _clear(self):
            self.last_seq = [None, None]
            self.total = [0, 0]
            self.window_count = [0, 0]
            self.lost = [0, 0]
            for panel in self.panels:
                panel.clear()

        def load_demo(self):
            base = time.monotonic() - 6.0
            for tick in range(120):
                for index in range(2):
                    sample = make_demo_sample(index, tick)
                    wobble = math.sin(tick * 0.13 + index) * 0.8
                    sample["resultant"] = (
                        sample["resultant"][0] + wobble,
                        sample["resultant"][1] - wobble * 0.4,
                        sample["resultant"][2] + wobble * 1.4,
                    )
                    self._accept_sample(sample, base + tick * 0.05)
            self.window_count = [80, 80]
            self.rate_started = time.monotonic() - 1.0
            self.stream_active = True
            self.stream_label.setText("USB 触觉流：演示数据")
            self._update_rates()

        def closeEvent(self, event):
            self.link.close()
            event.accept()

    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    installed = set(QFontDatabase.families())
    UI_FONT = next((f for f in ("Noto Sans SC", "Microsoft YaHei UI", "Microsoft YaHei",
                                "Source Han Sans SC", "SimHei") if f in installed),
                   app.font().family())
    MONO_FONT = next((f for f in ("JetBrains Mono", "Cascadia Mono", "Consolas")
                      if f in installed), UI_FONT)
    app.setFont(QFont(UI_FONT, 10))
    window = MainWindow()
    if render_path:
        window.load_demo()
        window.show()

        def save_and_quit():
            target = os.path.abspath(render_path)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            ok = window.grab().save(target)
            print(f"render {'OK' if ok else 'FAILED'}: {target}")
            app.quit()

        QTimer.singleShot(350, save_and_quit)
    else:
        window.show()
    return app.exec()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--selftest", action="store_true", help="test protocol without hardware")
    parser.add_argument("--render", metavar="PNG", help="render deterministic GUI demo and exit")
    args = parser.parse_args()
    if args.selftest:
        return selftest()
    if args.render:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    return run_gui(args.render)


if __name__ == "__main__":
    sys.exit(main())
