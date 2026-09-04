#!/usr/bin/env python3
"""PySide6 dashboard for the gripper_plus IMU-only firmware.

The firmware acquires the IMU at 100 Hz and batches samples into app_proto
telemetry packets on its 50 Hz supervisor tick. This program counts packet and
sample rates separately, expands every batch, plots every sample, and shows the
latest roll/pitch on a body-axis-aligned aircraft model.

    pip install -r host/requirements.txt
    python host/1_imu_gui.py
    python host/1_imu_gui.py --selftest
"""

import argparse
import math
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
    "IMU_STREAM_EN": 0x20,
    "IMU_STREAM_DIS": 0x21,
    "IMU_GET": 0x22,
    "IMU_CALIBRATE": 0x23,
    "IMU_SET_REF": 0x25,
    "IMU_POWON_SET": 0x27,
    "IMU_SET_RATE": 0x28,
    "IMU_ACQ_EN": 0x29,
    "IMU_ACQ_DIS": 0x2A,
}
OP_NAME = {value: name for name, value in OP.items()}
RATES_HZ = (10, 20, 50, 100, 200)

# One batched sample: device sequence, device millisecond tick, twelve floats.
IMU_SAMPLE_FMT = "<II12f"
IMU_SAMPLE_SIZE = struct.calcsize(IMU_SAMPLE_FMT)  # 56 bytes


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


def build_command(op: int, seq: int, u16=None) -> bytes:
    payload = bytes((op,))
    if u16 is not None:
        payload += struct.pack("<H", int(u16))
    return build(T_COMMAND, seq, payload)


def _sample_dict(seq, timestamp_ms, values):
    return {
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "roll": values[0],
        "pitch": values[1],
        "accel": values[2:5],
        "lin_accel": values[5:8],
        "gyro": values[8:11],
        "temp": values[11],
    }


def decode_imu_telemetry(payload: bytes):
    """Return all samples in one IMU telemetry packet, or None if malformed.

    Current: OP, COUNT, COUNT * [seq(u32), timestamp_ms(u32), 12*f32].
    The former one-sample layout is accepted to diagnose an unflashed board.
    """
    if not payload or payload[0] != OP["IMU_GET"]:
        return None
    if len(payload) == 1 + 12 * 4:  # legacy packet
        return [_sample_dict(None, None, struct.unpack_from("<12f", payload, 1))]
    if len(payload) < 2:
        return None
    count = payload[1]
    if count == 0 or len(payload) != 2 + count * IMU_SAMPLE_SIZE:
        return None
    samples = []
    offset = 2
    for _ in range(count):
        unpacked = struct.unpack_from(IMU_SAMPLE_FMT, payload, offset)
        samples.append(_sample_dict(unpacked[0], unpacked[1], unpacked[2:]))
        offset += IMU_SAMPLE_SIZE
    return samples


class Parser:
    """Extract CRC-valid app_proto frames from a stream mixed with ESP logs."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames, text = [], bytearray()
        i = 0
        while i < len(self.buf):
            start = self.buf.find(SOF, i)
            if start < 0:
                keep = 1 if self.buf.endswith(b"\xA5") else 0
                text += self.buf[i:len(self.buf) - keep]
                i = len(self.buf) - keep
                break
            if len(self.buf) - start < HDR_LEN:
                text += self.buf[i:start]
                i = start
                break
            version = self.buf[start + 2]
            payload_len = self.buf[start + 5] | (self.buf[start + 6] << 8)
            if version != VERSION or payload_len > MAX_PAYLOAD:
                text += self.buf[i:start + 1]
                i = start + 1
                continue
            total = HDR_LEN + payload_len + 2
            if len(self.buf) - start < total:
                text += self.buf[i:start]
                i = start
                break
            frame = bytes(self.buf[start:start + total])
            expected = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16(frame[2:HDR_LEN + payload_len]) != expected:
                text += self.buf[i:start + 1]
                i = start + 1
                continue
            text += self.buf[i:start]
            frames.append({
                "type": frame[3],
                "seq": frame[4],
                "payload": frame[HDR_LEN:HDR_LEN + payload_len],
            })
            i = start + total
        del self.buf[:i]
        return frames, bytes(text)


def selftest() -> int:
    failures = []

    def check(name, got, expected):
        if got != expected:
            failures.append(f"{name}: got {got!r}, expected {expected!r}")

    check("CRC vector", crc16(b"123456789"), 0x29B1)
    parser = Parser()
    frames, text = parser.feed(build_command(OP["IMU_SET_RATE"], 7, 100))
    check("command frame count", len(frames), 1)
    check("command text", text, b"")
    if frames:
        check("command payload", frames[0]["payload"], b"\x28\x64\x00")

    values1 = (12.5, -8.25, 0.1, 0.2, 0.9, 0.01, 0.02, 0.03, 1.0, 2.0, 3.0, 31.0)
    values2 = (13.0, -8.0, 0.2, 0.3, 0.8, 0.02, 0.03, 0.04, 4.0, 5.0, 6.0, 31.1)
    payload = bytes((OP["IMU_GET"], 2))
    payload += struct.pack(IMU_SAMPLE_FMT, 100, 2000, *values1)
    payload += struct.pack(IMU_SAMPLE_FMT, 101, 2010, *values2)
    parser = Parser()
    frames, _ = parser.feed(b"I (1) boot\n" + build(T_TELEMETRY, 101, payload))
    check("batch frame count", len(frames), 1)
    if frames:
        samples = decode_imu_telemetry(frames[0]["payload"])
        check("batch sample count", len(samples) if samples else 0, 2)
        if samples:
            check("batch sequences", [s["seq"] for s in samples], [100, 101])
            check("batch timestamps", [s["timestamp_ms"] for s in samples], [2000, 2010])
            check("batch roll", [s["roll"] for s in samples], [12.5, 13.0])

    bad = bytearray(build_command(OP["PING"], 9))
    bad[-1] ^= 0xFF
    parser = Parser()
    frames, _ = parser.feed(bytes(bad) + build_command(OP["PING"], 10))
    check("bad CRC resync", [f["seq"] for f in frames], [10])

    if failures:
        print("SELFTEST FAILED")
        for failure in failures:
            print(" -", failure)
        return 1
    print("selftest OK: CRC, parser and 2-sample IMU batch")
    return 0


# Serial worker -------------------------------------------------------------

class SerialLink:
    """One read thread; decoded objects cross to the Qt thread through a Queue."""

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
        # Native USB Serial/JTAG ignores this nominal baud value.
        self.ser = serial.Serial(port, 115200, timeout=0.05, dsrdtr=False, rtscts=False)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except OSError:
            pass
        self.parser = Parser()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, name="imu-usb-reader", daemon=True)
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

    def send(self, op, value=None):
        if not self.connected:
            return None
        with self.write_lock:
            self.command_seq = (self.command_seq + 1) & 0xFF
            self.ser.write(build_command(op, self.command_seq, value))
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

def run_gui() -> int:
    try:
        from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
        from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
        from PySide6.QtWidgets import (
            QApplication, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
            QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSizePolicy,
            QSplitter, QVBoxLayout, QWidget,
        )
        import serial.tools.list_ports
    except ImportError as exc:
        print(f"Missing GUI dependency: {exc}", file=sys.stderr)
        print("Install with: pip install -r host/requirements.txt", file=sys.stderr)
        return 1

    PANEL = QColor("#ffffff")
    TEXT = QColor("#172033")
    MUTED = QColor("#64748b")
    CYAN = QColor("#0284c7")
    BLUE = QColor("#2563eb")
    ORANGE = QColor("#e56f00")
    GREEN = QColor("#07875c")
    RED = QColor("#d83a52")

    class AttitudeModelWidget(QWidget):
        """Aircraft-style body model using the IMU's roll-X / pitch-Y axes.

        Body convention: +X points to the nose, +Y to the right wing and +Z
        downward. Therefore positive roll lowers the right wing, while positive
        pitch raises the nose. This matches the usual aerospace RPY convention.
        """

        def __init__(self):
            super().__init__()
            self.roll = 0.0
            self.pitch = 0.0
            self.setMinimumSize(390, 330)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        def set_attitude(self, roll, pitch):
            self.roll = float(roll)
            self.pitch = float(pitch)
            self.update()

        @staticmethod
        def _rot_x(point, angle):
            x, y, z = point
            c, s = math.cos(angle), math.sin(angle)
            return x, y * c - z * s, y * s + z * c

        @staticmethod
        def _rot_y(point, angle):
            x, y, z = point
            c, s = math.cos(angle), math.sin(angle)
            return x * c + z * s, y, -x * s + z * c

        def _transform(self, point):
            # R = Ry(pitch) Rx(roll), with yaw intentionally omitted.
            point = self._rot_x(point, math.radians(self.roll))
            return self._rot_y(point, math.radians(self.pitch))

        @staticmethod
        def _project(point, center, scale):
            x, y, z = point
            # Near-top/front camera: X-nose is up, Y-right is screen-right and
            # positive body Z is down. Both attitude motions stay easy to see.
            return QPointF(center.x() + y * scale,
                           center.y() + (z - 0.47 * x) * scale)

        @staticmethod
        def _view_depth(point):
            # Orthogonal to the two screen basis vectors used by _project().
            x, _y, z = point
            return x + 0.47 * z

        @staticmethod
        def _draw_arrow(painter, start, end, color, label):
            painter.setPen(QPen(color, 2.4))
            painter.setBrush(color)
            painter.drawLine(start, end)
            dx, dy = end.x() - start.x(), end.y() - start.y()
            length = max(math.hypot(dx, dy), 1.0)
            ux, uy = dx / length, dy / length
            px, py = -uy, ux
            base = QPointF(end.x() - ux * 10, end.y() - uy * 10)
            head = QPolygonF([
                end,
                QPointF(base.x() + px * 4.5, base.y() + py * 4.5),
                QPointF(base.x() - px * 4.5, base.y() - py * 4.5),
            ])
            painter.drawPolygon(head)
            painter.setFont(QFont("Segoe UI", 8, QFont.DemiBold))
            painter.drawText(QRectF(end.x() + 5, end.y() - 16, 80, 20), label)

        def paintEvent(self, _event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), PANEL)
            center_y = self.height() * 0.49
            center = QPointF(self.width() * 0.50, center_y)

            painter.setPen(QPen(QColor("#94a3b8"), 1, Qt.DashLine))
            painter.drawLine(28, int(center_y), self.width() - 28, int(center_y))
            painter.setPen(QColor("#94a3b8"))
            painter.setFont(QFont("Segoe UI", 8))
            painter.drawText(QRectF(self.width() - 112, center_y - 20, 84, 18),
                             Qt.AlignRight | Qt.AlignVCenter, "LEVEL 0°")

            # Directional aircraft silhouette: unlike a cube, nose and right
            # wing are unambiguous when comparing it with the physical IMU.
            outline = [
                (1.70, 0.00), (0.52, 0.17), (-0.12, 1.20), (-0.48, 1.20),
                (-0.58, 0.20), (-1.18, 0.16), (-1.43, 0.52), (-1.64, 0.52),
                (-1.52, 0.00), (-1.64, -0.52), (-1.43, -0.52),
                (-1.18, -0.16), (-0.58, -0.20), (-0.48, -1.20),
                (-0.12, -1.20), (0.52, -0.17),
            ]
            top = [self._transform((x, y, -0.10)) for x, y in outline]
            bottom = [self._transform((x, y, 0.18)) for x, y in outline]
            faces = [(bottom, QColor("#a8b7ca")), (top, QColor("#dbeafe"))]
            for index in range(len(outline)):
                following = (index + 1) % len(outline)
                faces.append((
                    [top[index], top[following], bottom[following], bottom[index]],
                    QColor("#9fc5e8" if index % 2 else "#bfdbfe"),
                ))
            # Larger view depth is farther from this camera, so paint it first.
            faces.sort(key=lambda face: sum(self._view_depth(p) for p in face[0]) /
                       len(face[0]), reverse=True)
            scale = min(self.width(), self.height()) * 0.23
            for points, color in faces:
                polygon = QPolygonF([self._project(p, center, scale) for p in points])
                painter.setBrush(color)
                painter.setPen(QPen(QColor("#334155"), 1.5))
                painter.drawPolygon(polygon)

            axis_origin = self._transform((0.0, 0.0, -0.14))
            origin_2d = self._project(axis_origin, center, scale)
            axes = [
                ((1.25, 0.0, -0.14), QColor("#dc2626"), "+X NOSE"),
                ((0.0, 0.92, -0.14), QColor("#059669"), "+Y RIGHT"),
                ((0.0, 0.0, 0.62), QColor("#2563eb"), "+Z DOWN"),
            ]
            for endpoint, color, label in axes:
                end_2d = self._project(self._transform(endpoint), center, scale)
                self._draw_arrow(painter, origin_2d, end_2d, color, label)
            painter.setPen(QPen(QColor("#172033"), 1.5))
            painter.setBrush(QColor("#ffffff"))
            painter.drawEllipse(origin_2d, 4.0, 4.0)

            painter.setPen(TEXT)
            painter.setFont(QFont("Segoe UI", 11, QFont.DemiBold))
            painter.drawText(QRectF(16, 13, self.width() - 32, 26),
                             Qt.AlignCenter, "BODY-AXIS ATTITUDE MODEL")
            painter.setFont(QFont("Segoe UI", 8, QFont.DemiBold))
            painter.setPen(MUTED)
            painter.drawText(QRectF(18, self.height() - 72, self.width() - 36, 20),
                             Qt.AlignCenter,
                             "ROLL + : RIGHT WING DOWN     PITCH + : NOSE UP")
            painter.setFont(QFont("JetBrains Mono", 13, QFont.DemiBold))
            painter.setPen(CYAN)
            painter.drawText(QRectF(18, self.height() - 46, self.width() / 2 - 20, 28),
                             Qt.AlignCenter, f"ROLL {self.roll:+07.2f}°")
            painter.setPen(ORANGE)
            painter.drawText(QRectF(self.width() / 2, self.height() - 46,
                                    self.width() / 2 - 18, 28),
                             Qt.AlignCenter, f"PITCH {self.pitch:+07.2f}°")

    class SignalPlot(QWidget):
        def __init__(self, title, unit, keys, colors, time_data, series_data, fixed_range=None):
            super().__init__()
            self.title = title
            self.unit = unit
            self.keys = keys
            self.colors = colors
            self.time_data = time_data
            self.series_data = series_data
            self.fixed_range = fixed_range
            self.setMinimumHeight(165)

        def paintEvent(self, _event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), PANEL)
            plot = QRectF(54, 31, max(10, self.width() - 70), max(10, self.height() - 58))
            painter.setFont(QFont("Segoe UI", 9, QFont.DemiBold))
            painter.setPen(TEXT)
            painter.drawText(14, 19, self.title)
            legend_x = self.width() - 15
            for key, color in reversed(list(zip(self.keys, self.colors))):
                legend_x -= 48
                painter.setPen(color)
                painter.drawText(QRectF(legend_x, 6, 48, 18), Qt.AlignRight, key)
            painter.setPen(QPen(QColor("#dbe4ee"), 1))
            for i in range(5):
                y = plot.top() + i * plot.height() / 4
                painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            for i in range(6):
                x = plot.left() + i * plot.width() / 5
                painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))
            if not self.time_data:
                painter.setPen(MUTED)
                painter.drawText(plot, Qt.AlignCenter, "waiting for IMU telemetry")
                return

            times = list(self.time_data)
            x_max = times[-1]
            x_min = max(times[0], x_max - 10.0)
            first = 0
            while first < len(times) - 1 and times[first] < x_min:
                first += 1
            visible = []
            for key in self.keys:
                visible.extend(list(self.series_data[key])[first:])
            if self.fixed_range:
                y_min, y_max = self.fixed_range
            elif visible:
                y_min, y_max = min(visible), max(visible)
                span = max(y_max - y_min, 0.2)
                y_min -= span * 0.12
                y_max += span * 0.12
            else:
                y_min, y_max = -1.0, 1.0

            painter.setFont(QFont("JetBrains Mono", 7))
            painter.setPen(MUTED)
            painter.drawText(QRectF(2, plot.top() - 7, 47, 16), Qt.AlignRight, f"{y_max:.1f}")
            painter.drawText(QRectF(2, plot.bottom() - 8, 47, 16), Qt.AlignRight, f"{y_min:.1f}")
            painter.drawText(QRectF(plot.left(), plot.bottom() + 5, 90, 16), Qt.AlignLeft, f"{x_min:.1f}s")
            painter.drawText(QRectF(plot.right() - 90, plot.bottom() + 5, 90, 16), Qt.AlignRight, f"{x_max:.1f}s")
            painter.save()
            painter.translate(13, plot.center().y())
            painter.rotate(-90)
            painter.drawText(QRectF(-55, -8, 110, 16), Qt.AlignCenter, self.unit)
            painter.restore()

            x_span = max(x_max - x_min, 0.01)
            y_span = max(y_max - y_min, 1e-6)
            painter.setClipRect(plot.adjusted(-1, -1, 1, 1))
            for key, color in zip(self.keys, self.colors):
                values = list(self.series_data[key])
                points = QPolygonF()
                for index in range(first, min(len(times), len(values))):
                    x = plot.left() + (times[index] - x_min) / x_span * plot.width()
                    y = plot.bottom() - (values[index] - y_min) / y_span * plot.height()
                    points.append(QPointF(x, y))
                painter.setPen(QPen(color, 1.5))
                if len(points) > 1:
                    painter.drawPolyline(points)
            painter.setClipping(False)

    class MetricCard(QFrame):
        def __init__(self, caption, color):
            super().__init__()
            self.setObjectName("metricCard")
            layout = QVBoxLayout(self)
            layout.setContentsMargins(12, 8, 12, 8)
            caption_label = QLabel(caption)
            caption_label.setObjectName("metricCaption")
            self.value = QLabel("—")
            self.value.setStyleSheet(f"color: {color.name()}; font: 700 18px 'JetBrains Mono';")
            layout.addWidget(caption_label)
            layout.addWidget(self.value)

    class MainWindow(QMainWindow):
        HISTORY = 2000

        def __init__(self):
            super().__init__()
            self.setWindowTitle("gripper_plus · IMU 100 Hz Inspector")
            self.resize(1480, 940)
            self.output_queue = queue.Queue()
            self.link = SerialLink(self.output_queue)
            self.text_buffer = ""
            self.times = deque(maxlen=self.HISTORY)
            self.series = {key: deque(maxlen=self.HISTORY) for key in
                           ("roll", "pitch", "ax", "ay", "az", "gx", "gy", "gz")}
            self.last_sample = None
            self.first_device_ms = None
            self.first_host_time = None
            self.last_sample_seq = None
            self.total_packets = 0
            self.total_samples = 0
            self.lost_samples = 0
            self.window_packets = 0
            self.window_samples = 0
            self.stream_active = False
            self.acq_active = None
            self.rate_started = time.monotonic()
            self._build_ui()
            self._refresh_ports()

            self.pump_timer = QTimer(self)
            self.pump_timer.timeout.connect(self._pump)
            self.pump_timer.start(10)
            self.draw_timer = QTimer(self)
            self.draw_timer.timeout.connect(self._redraw)
            self.draw_timer.start(20)
            self.rate_timer = QTimer(self)
            self.rate_timer.timeout.connect(self._update_rates)
            self.rate_timer.start(1000)

        def _build_ui(self):
            root = QWidget()
            self.setCentralWidget(root)
            outer = QVBoxLayout(root)
            outer.setContentsMargins(14, 12, 14, 12)
            outer.setSpacing(10)

            top = QHBoxLayout()
            title_box = QVBoxLayout()
            title = QLabel("IMU MOTION LAB")
            title.setObjectName("title")
            subtitle = QLabel("100 Hz acquisition · 50 Hz USB batching · zero sample decimation")
            subtitle.setObjectName("subtitle")
            title_box.addWidget(title)
            title_box.addWidget(subtitle)
            top.addLayout(title_box)
            top.addStretch()
            self.port_combo = QComboBox()
            self.port_combo.setMinimumWidth(290)
            refresh = QPushButton("刷新端口")
            refresh.clicked.connect(self._refresh_ports)
            self.connect_button = QPushButton("连接")
            self.connect_button.setObjectName("primaryButton")
            self.connect_button.clicked.connect(self._toggle_connection)
            self.connection_label = QLabel("● DISCONNECTED")
            self.connection_label.setObjectName("disconnected")
            top.addWidget(self.port_combo)
            top.addWidget(refresh)
            top.addWidget(self.connect_button)
            top.addWidget(self.connection_label)
            outer.addLayout(top)

            metrics = QHBoxLayout()
            self.sample_card = MetricCard("上位机样本率", GREEN)
            self.packet_card = MetricCard("USB 上报包率", CYAN)
            self.loss_card = MetricCard("序号丢样", RED)
            self.total_card = MetricCard("累计样本", ORANGE)
            for card in (self.sample_card, self.packet_card, self.loss_card, self.total_card):
                metrics.addWidget(card)
            outer.addLayout(metrics)

            splitter = QSplitter(Qt.Horizontal)
            left = QFrame()
            left.setObjectName("panel")
            left_layout = QVBoxLayout(left)
            left_layout.setContentsMargins(10, 10, 10, 10)
            self.attitude_model = AttitudeModelWidget()
            left_layout.addWidget(self.attitude_model, 1)
            self.readout = QLabel("等待数据")
            self.readout.setObjectName("readout")
            self.readout.setWordWrap(True)
            left_layout.addWidget(self.readout)

            controls = QGridLayout()
            self.command_buttons = {}
            buttons = [
                ("开始 USB 上报", "IMU_STREAM_EN"), ("停止 USB 上报", "IMU_STREAM_DIS"),
                ("开始 IMU→ESP 采集", "IMU_ACQ_EN"), ("停止 IMU→ESP 采集", "IMU_ACQ_DIS"),
                ("读取一个样本", "IMU_GET"), ("Ping", "PING"),
                ("读取状态", "GET_STATE"), ("读取版本", "GET_VERSION"),
            ]
            for index, (label, op_name) in enumerate(buttons):
                button = QPushButton(label)
                button.clicked.connect(lambda _checked=False, name=op_name: self._send(name))
                controls.addWidget(button, index // 2, index % 2)
                self.command_buttons[op_name] = button
            self.rate_combo = QComboBox()
            self.rate_combo.addItems([str(rate) for rate in RATES_HZ])
            self.rate_combo.setCurrentText("100")
            set_rate = QPushButton("设置 IMU Hz")
            set_rate.clicked.connect(lambda: self._send("IMU_SET_RATE", int(self.rate_combo.currentText())))
            controls.addWidget(self.rate_combo, 4, 0)
            controls.addWidget(set_rate, 4, 1)
            set_ref = QPushButton("当前位置设为零点")
            set_ref.clicked.connect(self._set_reference)
            calibrate = QPushButton("加速度计校准")
            calibrate.clicked.connect(self._calibrate)
            controls.addWidget(set_ref, 5, 0)
            controls.addWidget(calibrate, 5, 1)
            clear = QPushButton("清空曲线与计数")
            clear.clicked.connect(self._clear_data)
            controls.addWidget(clear, 6, 0, 1, 2)
            self.acq_status = QLabel()
            self.acq_status.setAlignment(Qt.AlignCenter)
            controls.addWidget(self.acq_status, 7, 0, 1, 2)
            self._set_acq_state(None)
            self.stream_status = QLabel()
            self.stream_status.setAlignment(Qt.AlignCenter)
            controls.addWidget(self.stream_status, 8, 0, 1, 2)
            self._set_stream_state(False)
            left_layout.addLayout(controls)

            right = QWidget()
            right_layout = QVBoxLayout(right)
            right_layout.setContentsMargins(0, 0, 0, 0)
            right_layout.setSpacing(8)
            self.plots = [
                SignalPlot("ROLL / PITCH", "degree", ("roll", "pitch"),
                           (CYAN, ORANGE), self.times, self.series, (-180.0, 180.0)),
                SignalPlot("ACCELERATION", "g", ("ax", "ay", "az"),
                           (RED, GREEN, BLUE), self.times, self.series),
                SignalPlot("ANGULAR VELOCITY", "deg/s", ("gx", "gy", "gz"),
                           (RED, GREEN, BLUE), self.times, self.series),
            ]
            for plot in self.plots:
                right_layout.addWidget(plot)
            splitter.addWidget(left)
            splitter.addWidget(right)
            splitter.setSizes([480, 900])
            outer.addWidget(splitter, 1)

            self.log = QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(1600)
            self.log.setMinimumHeight(130)
            self.log.setPlaceholderText("设备日志、ACK和错误信息")
            outer.addWidget(self.log)

            self.setStyleSheet("""
                QWidget { background: #ffffff; color: #172033; font: 10pt 'Segoe UI'; }
                QFrame#panel, QFrame#metricCard { background: #ffffff; border: 1px solid #d8e1eb; border-radius: 9px; }
                QLabel#title { font: 800 22px 'Segoe UI'; color: #10243e; letter-spacing: 2px; }
                QLabel#subtitle, QLabel#metricCaption { color: #64748b; font-size: 9pt; }
                QLabel#readout { background: #f8fafc; border: 1px solid #d8e1eb; border-radius: 7px;
                                 padding: 10px; font: 11pt 'JetBrains Mono'; color: #24364b; }
                QLabel#connected { color: #07875c; font-weight: 700; }
                QLabel#disconnected { color: #d83a52; font-weight: 700; }
                QLabel#streamActive { color: #07875c; font-weight: 700; padding: 4px; }
                QLabel#streamStopped { color: #64748b; font-weight: 700; padding: 4px; }
                QLabel#streamPending { color: #b45309; font-weight: 700; padding: 4px; }
                QLabel#acqActive { color: #2563eb; font-weight: 700; padding: 4px; }
                QLabel#acqStopped { color: #d83a52; font-weight: 700; padding: 4px; }
                QLabel#acqPending { color: #b45309; font-weight: 700; padding: 4px; }
                QPushButton, QComboBox { background: #ffffff; border: 1px solid #bdcad8; border-radius: 6px;
                                        padding: 7px 10px; min-height: 18px; }
                QPushButton:hover { background: #eef7ff; border-color: #0284c7; }
                QPushButton#primaryButton { background: #e0f2fe; border-color: #0284c7;
                                            color: #075985; font-weight: 700; }
                QPlainTextEdit { background: #ffffff; border: 1px solid #d8e1eb; border-radius: 7px;
                                 color: #526579; font: 9pt 'JetBrains Mono'; padding: 5px; }
                QSplitter::handle { background: #e8eef5; width: 7px; }
            """)

        def _set_acq_state(self, active):
            self.acq_active = active
            if active is True:
                text, object_name = "IMU→ESP 采集：运行中", "acqActive"
            elif active is False:
                text, object_name = "IMU→ESP 采集：已停止", "acqStopped"
            else:
                text, object_name = "IMU→ESP 采集：等待设备状态", "acqPending"
            self.acq_status.setText(text)
            self.acq_status.setObjectName(object_name)
            self.acq_status.style().unpolish(self.acq_status)
            self.acq_status.style().polish(self.acq_status)

        def _set_stream_state(self, active):
            self.stream_active = active
            if active is True:
                text, object_name = "USB 数据流：运行中", "streamActive"
            elif active is False:
                text, object_name = "USB 数据流：已停止（不改变 IMU 采集）", "streamStopped"
            else:
                text, object_name = "USB 数据流：等待设备确认…", "streamPending"
            self.stream_status.setText(text)
            self.stream_status.setObjectName(object_name)
            self.stream_status.style().unpolish(self.stream_status)
            self.stream_status.style().polish(self.stream_status)

        def _append_log(self, text):
            for line in text.splitlines():
                if line.strip():
                    self.log.appendPlainText(line.rstrip())

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
            self._set_acq_state(None)
            self._set_stream_state(False)
            self._append_log(
                f"--- connected {port}; USB stream remains stopped; click 开始 USB 上报 ---")

        def _disconnect(self):
            self.link.close()
            self.connect_button.setText("连接")
            self.connection_label.setText("● DISCONNECTED")
            self.connection_label.setObjectName("disconnected")
            self.connection_label.style().unpolish(self.connection_label)
            self.connection_label.style().polish(self.connection_label)
            self._set_acq_state(None)
            self._set_stream_state(False)
            self._append_log("--- disconnected ---")

        def _send(self, name, value=None, quiet=False):
            if not self.link.connected:
                if not quiet:
                    QMessageBox.warning(self, "尚未连接", "请先连接 XIAO。")
                return
            if name == "IMU_ACQ_DIS" and self.stream_active:
                self._append_log(
                    "REMINDER: USB 上报开关仍为开启；停止 IMU 采集后将暂时没有新数据。")
            elif name == "IMU_STREAM_EN" and self.acq_active is False:
                self._append_log(
                    "REMINDER: IMU 采集当前已停止；若没有近期有效样本，设备将返回 ERR_STATE。")
            try:
                seq = self.link.send(OP[name], value)
            except Exception as exc:
                self._append_log(f"TX ERROR: {exc}")
                return
            if not quiet:
                suffix = "" if value is None else f"  value={value}"
                self._append_log(f"TX seq={seq:3d}  {name}{suffix}")

        def _set_reference(self):
            answer = QMessageBox.question(
                self, "设置姿态零点",
                "这会把当前 roll/pitch 写为永久参考零点。\n\n"
                "请固定测试方块并保持静止约 3 秒。是否继续？")
            if answer == QMessageBox.Yes:
                self._send("IMU_SET_REF")

        def _calibrate(self):
            answer = QMessageBox.question(
                self, "加速度计校准",
                "校准会写入 IMU Flash。模块必须水平放置并保持静止至少 5.5 秒。\n\n是否继续？")
            if answer == QMessageBox.Yes:
                self._send("IMU_CALIBRATE")

        def _clear_data(self):
            self.times.clear()
            for values in self.series.values():
                values.clear()
            self.first_device_ms = None
            self.first_host_time = None
            self.last_sample_seq = None
            self.total_packets = self.total_samples = self.lost_samples = 0
            self.window_packets = self.window_samples = 0

        def _sample_time(self, sample, received_at, index, batch_count):
            stamp = sample["timestamp_ms"]
            if stamp is not None:
                if self.first_device_ms is None:
                    self.first_device_ms = stamp
                return ((stamp - self.first_device_ms) & 0xFFFFFFFF) / 1000.0
            if self.first_host_time is None:
                self.first_host_time = received_at
            return received_at - self.first_host_time - (batch_count - index - 1) * 0.01

        def _accept_sample(self, sample, received_at, index, batch_count):
            seq = sample["seq"]
            if seq is not None and self.last_sample_seq is not None:
                delta = (seq - self.last_sample_seq) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    self.lost_samples += delta - 1
            if seq is not None:
                self.last_sample_seq = seq
            self.times.append(self._sample_time(sample, received_at, index, batch_count))
            self.series["roll"].append(sample["roll"])
            self.series["pitch"].append(sample["pitch"])
            for key, value in zip(("ax", "ay", "az"), sample["accel"]):
                self.series[key].append(value)
            for key, value in zip(("gx", "gy", "gz"), sample["gyro"]):
                self.series[key].append(value)
            self.last_sample = sample

        def _on_frame(self, received_at, frame):
            payload = frame["payload"]
            op = payload[0] if payload else None
            name = OP_NAME.get(op, f"0x{op:02X}" if op is not None else "?")
            frame_type = frame["type"]
            if frame_type == T_TELEMETRY and op == OP["IMU_GET"]:
                samples = decode_imu_telemetry(payload)
                if samples is None:
                    self._append_log(f"RX malformed IMU telemetry ({len(payload)} bytes)")
                    return
                self.total_packets += 1
                self.total_samples += len(samples)
                self.window_packets += 1
                self.window_samples += len(samples)
                for index, sample in enumerate(samples):
                    self._accept_sample(sample, received_at, index, len(samples))
                if self.acq_status.objectName() != "acqActive":
                    self._set_acq_state(True)
                return
            if frame_type in (T_ACK, T_ERROR):
                status = STATUS_NAME.get(payload[1], f"0x{payload[1]:02X}") if len(payload) > 1 else "?"
                self._append_log(f"RX seq={frame['seq']:3d} {TYPE_NAME[frame_type]} {name} -> {status}")
                if frame_type == T_ACK and len(payload) > 1 and payload[1] == 0:
                    if op == OP["IMU_STREAM_EN"]:
                        self._set_acq_state(True)  # Firmware interlock guarantees this.
                        self._set_stream_state(True)
                    elif op == OP["IMU_STREAM_DIS"]:
                        self._set_stream_state(False)
                    elif op == OP["IMU_ACQ_EN"]:
                        self._set_acq_state(True)
                    elif op == OP["IMU_ACQ_DIS"]:
                        self._set_acq_state(False)
            elif frame_type == T_RESPONSE and op == OP["GET_STATE"] and len(payload) > 1:
                self._append_log(f"RX STATE = {STATE_NAME.get(payload[1], payload[1])}")
            elif frame_type == T_RESPONSE and op == OP["GET_VERSION"] and len(payload) >= 4:
                self._append_log(f"RX VERSION proto={payload[1]} firmware={payload[2]}.{payload[3]}")
            else:
                self._append_log(f"RX {TYPE_NAME.get(frame_type, frame_type)} {name} {payload[1:].hex(' ')}")

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

        def _redraw(self):
            if self.last_sample:
                sample = self.last_sample
                self.attitude_model.set_attitude(sample["roll"], sample["pitch"])
                accel = sample["accel"]
                gyro = sample["gyro"]
                self.readout.setText(
                    f"ROLL   {sample['roll']:+8.2f}°    PITCH {sample['pitch']:+8.2f}°\n"
                    f"ACC    X {accel[0]:+7.3f}  Y {accel[1]:+7.3f}  Z {accel[2]:+7.3f} g\n"
                    f"GYRO   X {gyro[0]:+7.2f}  Y {gyro[1]:+7.2f}  Z {gyro[2]:+7.2f} °/s\n"
                    f"TEMP   {sample['temp']:5.1f} °C    SEQ {sample['seq'] if sample['seq'] is not None else 'legacy'}")
            for plot in self.plots:
                plot.update()

        def _update_rates(self):
            now = time.monotonic()
            elapsed = max(now - self.rate_started, 1e-3)
            self.sample_card.value.setText(f"{self.window_samples / elapsed:5.1f} Hz")
            self.packet_card.value.setText(f"{self.window_packets / elapsed:5.1f} Hz")
            self.loss_card.value.setText(str(self.lost_samples))
            self.total_card.value.setText(f"{self.total_samples:,}")
            self.window_samples = self.window_packets = 0
            self.rate_started = now

        def closeEvent(self, event):
            self.link.close()
            event.accept()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Segoe UI", 10))
    window = MainWindow()
    window.show()
    return app.exec()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--selftest", action="store_true",
                        help="test framing and batched telemetry without hardware")
    args = parser.parse_args()
    return selftest() if args.selftest else run_gui()


if __name__ == "__main__":
    sys.exit(main())
