#!/usr/bin/env python3
"""PySide6 control panel for the gripper: one-click operations and live data.

Connects over USB, Bluetooth LE or Wi-Fi (TCP) with the same links as
host/3_gripper_cli.py. The firmware pushes state, position, motor and both
pads (GRIP_STREAM_EN: 30 Hz over Bluetooth, 50 Hz otherwise); older firmware
is polled up to 20 times a second instead. Plots the last 20 s as the data
arrives and exports what it recorded to CSV.

    python host/4_gripper_gui.py
    python host/4_gripper_gui.py --render tmp/gripper_gui.png   # offscreen screenshot with demo data

Esc = motor off at once (MOTOR_DISABLE), on any link.
"""

import argparse
import csv
import importlib.util
import math
import os
import pathlib
import queue
import struct
import sys
import threading
import time
from collections import deque

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("gripper_cli", HERE / "3_gripper_cli.py")
cli = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cli)

# The firmware pushes state, both pads and the motor (GRIP_STREAM_EN) at a
# display rate; its own force control runs on every tactile sample regardless.
STREAM_HZ = {"ble": 30, "usb": 50, "tcp": 50}
CALIB_S = 1.0         # with the stream: GRIP_GET_CALIB this often, for the calibration
                      # values and as the keepalive (the stream stops after 3 s silence)
# Older firmware without the stream is polled instead:
POLL_S = 0.05         # a round of state + motor + pads requests at most this often ...
ROUND_TIMEOUT_S = 0.5 # ... and only once the last round answered (or this passed)
STALE_S = 1.0         # no motor reply this long: shown as no feedback
HISTORY_S = 20.0      # plotted window
GAP_S = 0.6           # a plotted line breaks only where samples are further apart
REDRAW_MS = 30        # plots scroll with the clock, not only when data arrives
LIVE_MIN_S = 0.05     # dragged slider: at most one move command per this, one in flight
RECORD_MAX = 120000   # rows kept for CSV export (100 min at 20 Hz)
# home and calibration run for a long time after their OK; pressing either again restarts from the beginning
CAL_OPS = {"GRIP_HOME": "home 约 40 s（没有存储的标定时自动改做完整标定，约 1.5–3 分钟）",
           "GRIP_CALIBRATE": "完整标定约 1.5–3 分钟"}
CAL_REFUSAL_NEXT = "（拒绝原因见下一行）"   # home / calibration refused: the next poll names the fault


# link worker -----------------------------------------------------------------

def nice_axis(lo, hi, n=4):
    """Axis from lo to hi in about n steps of 1, 2, 2.5 or 5 x 10^k: (lo, hi, step)."""
    raw = max(hi - lo, 1e-6) / n
    mag = 10.0 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw * 0.999)
    return math.floor(lo / step + 1e-9) * step, math.ceil(hi / step - 1e-9) * step, step


class GuiLink(cli.Link):
    """cli.Link that hands firmware log lines and every status reply to the GUI
    the moment they are parsed, each with its own arrival time."""

    def __init__(self, io, events):
        super().__init__(io)
        self.events = events
        self.round_open = False   # a poll round still waits for its last reply
        self.tele_state = None    # gripper state in the last stream frame
        self.calib_wanted = False # calibration just ended: fetch its values now

    def _frame(self, ftype: int, seq: int, pl: bytes):
        if pl and pl[0] == cli.OP["GRIP_STREAM_EN"] and ftype == cli.T_TELEMETRY and len(pl) >= 48:
            self._tele(pl)
            return
        calib, motor = self.calib, self.motor
        super()._frame(ftype, seq, pl)
        t = time.time()
        if self.calib is not calib:
            self.events.put(("calib", t, self.calib, True))
        if self.motor is not motor:
            self.events.put(("motor", t, self.motor))
        if pl and pl[0] == cli.OP["HAPTIC_GET"]:
            if ftype in (cli.T_ACK, cli.T_ERROR):
                self.round_open = False          # tactile refused: the round is over
            elif len(pl) >= 25:
                self.events.put(("pad", t, pl[1], self.pads[pl[1]]))
                if pl[1] == 1:                   # the little pad answers last
                    self.round_open = False

    def _tele(self, pl: bytes):
        """One GRIP_STREAM_EN frame (emit_grip_tele() in the firmware) becomes the
        usual calib / motor / pad events. The calibration values in "calib" are
        those of the last GRIP_GET_CALIB reply, marked as not a full reply."""
        t = time.time()
        state, fault, flags, err = pl[1], pl[2], pl[3], pl[4]
        pos, opening, ftarget, ring, little, torque, vel, t_mos, t_rotor = struct.unpack_from("<9f", pl, 12)
        if self.tele_state == cli.GRIP_STATE.index("CALIBRATING") and state != self.tele_state:
            self.calib_wanted = True
        self.tele_state = state
        if self.calib is not None:
            c = dict(self.calib, valid=flags & 0x01, state=state, fault=fault, pos=pos, opening=opening,
                     force_target=ftarget)
            self.events.put(("calib", t, c, False))
        if flags & 0x04:   # motor feedback fresh
            self.events.put(("motor", t, {"enabled": (flags >> 1) & 1, "pos": pos, "vel": vel, "torque": torque,
                                          "t_mos": t_mos, "t_rotor": t_rotor, "err": err, "t": t}))
        for index, fz in ((0, ring), (1, little)):
            if flags & (0x08 << index):   # that pad fresh
                self.events.put(("pad", t, index, fz))

    def _log(self, raw: bytes):
        for line in cli.ANSI.sub(b"", raw).decode("utf-8", "replace").splitlines():
            line = line.rstrip()
            if line:
                self.events.put(("log", line))


class Worker(threading.Thread):
    """Owns the link. The GUI sends ("connect"|"disconnect"|"op", ...) on cmd_q;
    the worker answers with events on the GUI's queue."""

    def __init__(self, events: queue.Queue):
        super().__init__(daemon=True)
        self.events = events
        self.cmd_q = queue.Queue()
        self.link = None
        self.stream = False   # the firmware accepted GRIP_STREAM_EN on this link
        self.next_poll = 0.0
        self.next_calib = 0.0
        self.round_t = 0.0

    def run(self):
        while True:
            try:
                item = self.cmd_q.get(timeout=0.005 if self.link else 0.05)
            except queue.Empty:
                item = None
            try:
                if item:
                    self._handle(item)
                if self.link:
                    self.link.poll()   # one read (at most ~20 ms); replies go to the GUI as parsed
                    self._request()
            except (OSError, ConnectionError) as e:
                self._drop("connection lost: %s" % e)

    def _drop(self, why, stop_stream=False):
        if self.link:
            try:
                if stop_stream and self.stream:   # USB has no close event the firmware could see
                    self.link.send(cli.OP["GRIP_STREAM_DIS"])
                self.link.ser.close()
            except Exception:
                pass
        self.link = None
        self.stream = False
        self.events.put(("disconnected", why))

    def _handle(self, item):
        kind = item[0]
        if kind == "connect":
            _, link_kind, target = item
            if self.link:
                self._drop("reconnecting", stop_stream=True)
            ns = argparse.Namespace(port=None, tcp=None, ble=False, ble_name=None)
            if link_kind == "usb":
                ns.port = target
            elif link_kind == "ble":
                ns.ble, ns.ble_name = True, target
            else:
                ns.tcp = target
            self.events.put(("status", "connecting to %s ..." % target))
            try:
                self.link = GuiLink(cli.open_io(ns), self.events)
            except Exception as e:   # bleak raises several types
                self.events.put(("disconnected", "cannot connect: %s" % e))
                return
            st = self.link.command("SESSION_UNLOCK", struct.pack("<I", cli.SESSION_KEY))
            hz = STREAM_HZ[link_kind]
            st_stream = self.link.command("GRIP_STREAM_EN", struct.pack("<H", hz))
            self.stream = st_stream == "OK"
            self.next_calib = 0.0
            mode = ("固件推送 %d Hz" % hz if self.stream else
                    "查询，最快 %.0f Hz（固件不支持推送：%s）" % (1.0 / POLL_S, st_stream))
            self.events.put(("connected", "%s %s (unlock %s)，数据：%s" % (link_kind.upper(), target, st, mode)))
        elif kind == "disconnect":
            self._drop("disconnected", stop_stream=True)
        elif kind == "op":
            _, label, name, args, quiet = item
            if not self.link:   # the GUI waits for this result before it sends the next slider move
                self.events.put(("result", label, "not connected", name, quiet))
                return
            st = self.link.command(name, args, timeout=3.0 if name == "MOTOR_SAVE_CONFIG" else 1.0)
            if st == "ERR_LOCKED":   # session relocked (e.g. after a reconnect): unlock and retry once
                self.link.command("SESSION_UNLOCK", struct.pack("<I", cli.SESSION_KEY))
                st = self.link.command(name, args)
            self.events.put(("result", label, st, name, quiet))
        elif kind == "scan":
            self.events.put(("scan", ble_scan()))

    def _request(self):
        """Ask for state, motor and both pads every POLL_S, but not before the
        last round has answered (or ROUND_TIMEOUT_S passed): a slow link then
        polls more slowly instead of piling up requests. Nothing waits here."""
        now = time.time()
        link = self.link
        if self.stream:   # the firmware pushes the rest
            if now >= self.next_calib or link.calib_wanted:
                self.next_calib = now + CALIB_S
                link.calib_wanted = False
                link.send(cli.OP["GRIP_GET_CALIB"])
            return
        if now < self.next_poll or (link.round_open and now - self.round_t < ROUND_TIMEOUT_S):
            return
        self.next_poll = now + POLL_S
        self.round_t = now
        link.round_open = True
        link.send(cli.OP["GRIP_GET_CALIB"])
        link.send(cli.OP["MOTOR_GET_STATUS"])
        link.send(cli.OP["HAPTIC_GET"])


def ble_scan():
    try:
        import asyncio
        from bleak import BleakScanner
        devs = asyncio.run(BleakScanner.discover(timeout=5.0))
    except Exception as e:
        return ["scan failed: %s" % e]
    return ["%s  %s" % (d.address, d.name) for d in devs if (d.name or "").startswith(cli.BLE_NAME)]


# GUI -------------------------------------------------------------------------

def run_gui(render_path=None) -> int:
    try:
        from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
        from PySide6.QtGui import QColor, QFont, QFontDatabase, QKeySequence, QPainter, QPainterPath, QPen, QShortcut
        from PySide6.QtWidgets import (
            QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
            QLineEdit, QMainWindow, QPlainTextEdit, QPushButton, QSlider, QVBoxLayout, QWidget,
        )
        import serial.tools.list_ports
    except ImportError as exc:
        print(f"Missing GUI dependency: {exc}", file=sys.stderr)
        print("Install with: pip install -r host/requirements.txt", file=sys.stderr)
        return 1

    MUTED = QColor("#64748b")
    CYAN = QColor("#0284c7")
    ORANGE = QColor("#e56f00")
    GREEN = QColor("#07875c")
    RED = QColor("#d83a52")

    class Card(QFrame):
        def __init__(self, caption, color="#172033"):
            super().__init__()
            self.setObjectName("card")
            lay = QVBoxLayout(self)
            lay.setContentsMargins(10, 6, 10, 6)
            cap = QLabel(caption)
            cap.setObjectName("caption")
            self.value = QLabel("—")
            self.value.setStyleSheet(f"font-size: 15px; font-weight: 800; color: {color};")
            lay.addWidget(cap)
            lay.addWidget(self.value)

        def set(self, text):
            self.value.setText(text)

    class Plot(QWidget):
        """The last HISTORY_S seconds up to now. Each series keeps its own
        (time, value) samples, so one late reply leaves no hole in the others;
        a line breaks only at None or where samples are more than GAP_S apart."""

        def __init__(self, title, unit, series, fixed=None):
            super().__init__()
            self.title, self.unit, self.fixed = title, unit, fixed
            self.series = [(label, color, deque()) for label, color in series]
            self.empty_text = "等待数据"
            self.setMinimumHeight(170)

        def add(self, index, t, v):
            dq = self.series[index][2]
            dq.append((t, v))
            while dq and dq[0][0] < t - HISTORY_S - GAP_S:
                dq.popleft()

        def paintEvent(self, _e):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor("#fbfdff"))
            plot = QRectF(44, 24, self.width() - 56, self.height() - 42)
            p.setFont(QFont(UI_FONT, 8))
            fm = p.fontMetrics()
            p.setPen(MUTED)
            head = f"{self.title} ({self.unit})"
            p.drawText(QRectF(8, 3, 400, 18), Qt.AlignLeft, head)
            x = 8 + fm.horizontalAdvance(head) + 16
            for label, color, _ in self.series:
                p.fillRect(QRectF(x, 8, 8, 8), color)
                x += 12
                p.drawText(QRectF(x, 3, 200, 18), Qt.AlignLeft, label)
                x += fm.horizontalAdvance(label) + 14

            t1 = time.time()
            t0 = t1 - HISTORY_S
            vals = [v for _, _, dq in self.series for t, v in dq if v is not None and t >= t0]
            if self.fixed:
                lo, hi = self.fixed
                step = (hi - lo) / 4
            else:
                vmin = min(vals + [0.0])
                # a pad reads a few 0.1 N below zero after a tare: keep the axis at 0 for that
                lo, hi, step = nice_axis(vmin if vmin < -0.5 else 0.0, max(vals + [1.0]))
            # grid and value labels at round numbers
            n = int(round((hi - lo) / step))
            for i in range(n + 1):
                v = lo + i * step
                y = plot.bottom() - i / n * plot.height()
                p.setPen(QPen(QColor("#e1e8f0"), 1))
                p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
                p.setPen(MUTED)
                p.drawText(QRectF(2, y - 8, 38, 16), Qt.AlignRight, f"{v:g}")
            for s in range(0, int(HISTORY_S) + 1, 5):   # time labels every 5 s
                xs = plot.right() - s / HISTORY_S * plot.width()
                align = Qt.AlignRight if s == 0 else (Qt.AlignLeft if s == HISTORY_S else Qt.AlignHCenter)
                box = QRectF(xs - (60 if s == 0 else 0 if s == HISTORY_S else 30), plot.bottom() + 3, 60, 14)
                p.drawText(box, align, "现在" if s == 0 else f"-{s} s")
            if not vals:
                p.drawText(plot, Qt.AlignCenter, self.empty_text)
                return

            p.setClipRect(plot.adjusted(-2, -2, 2, 2))
            for _, color, dq in self.series:
                path = QPainterPath()
                prev = None
                for t, v in dq:
                    if v is None:
                        prev = None
                        continue
                    x = plot.left() + (t - t0) / HISTORY_S * plot.width()
                    y = plot.bottom() - (min(max(v, lo), hi) - lo) / (hi - lo) * plot.height()
                    if prev is None or t - prev > GAP_S:
                        path.moveTo(x, y)
                    else:
                        path.lineTo(x, y)
                    prev = t
                pen = QPen(color, 2.2)
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(pen)
                p.drawPath(path)

    def spin(lo, hi, value, step, decimals, suffix):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setValue(value)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        s.setSuffix(suffix)
        return s

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("GripperOne 夹爪控制台")
            self.resize(1180, 760)
            self.events = queue.Queue()
            self.worker = Worker(self.events)
            self.worker.start()
            self.connected = False
            self.records = deque(maxlen=RECORD_MAX)
            self.last_c = None       # latest GRIP_GET_CALIB reply, for refusal hints
            self.last_m = None       # latest motor status (None: no motor feedback)
            self.last_m_t = 0.0      # when it arrived
            self.m_stale = False     # "no feedback" shown
            self.last_pads = {}      # pad index -> latest Fz (N)
            self.connected_t = 0.0
            self.cal_run = None      # home / calibration in progress: {"label", "t0", "seen"}
            self.why_pending = False # home / calibration refused: report the fault from the next poll
            self.live_pct = None     # dragged slider: newest closing percent not sent yet
            self.live_busy = False   # a slider move is in flight
            self.live_t = 0.0        # when the last slider move was sent
            self.live_err = ""       # last slider refusal logged, so a drag logs it once
            self._syncing = False    # slider and spin box updating each other, not the user

            root = QWidget()
            self.setCentralWidget(root)
            outer = QVBoxLayout(root)
            outer.setContentsMargins(14, 10, 14, 10)

            # connection bar
            bar = QHBoxLayout()
            title = QLabel("GripperOne 夹爪控制台")
            title.setObjectName("title")
            bar.addWidget(title)
            bar.addStretch(1)
            self.kind = QComboBox()
            self.kind.addItem("USB", "usb")
            self.kind.addItem("蓝牙", "ble")
            self.kind.addItem("Wi-Fi", "tcp")
            self.kind.currentIndexChanged.connect(self._kind_changed)
            self.target = QComboBox()
            self.target.setEditable(True)
            self.target.setMinimumWidth(260)
            self.refresh = QPushButton("刷新")
            self.refresh.clicked.connect(self._refresh_targets)
            self.connect_btn = QPushButton("连接")
            self.connect_btn.setObjectName("primary")
            self.connect_btn.clicked.connect(self._toggle_connect)
            self.status = QLabel("● 未连接")
            self.status.setObjectName("off")
            for w in (self.kind, self.target, self.refresh, self.connect_btn, self.status):
                bar.addWidget(w)
            outer.addLayout(bar)

            body = QHBoxLayout()
            outer.addLayout(body, 1)

            # operations
            ops = QFrame()
            ops.setObjectName("panel")
            ops.setFixedWidth(360)
            ol = QVBoxLayout(ops)
            ol.addWidget(self._section("准备"))
            row = QHBoxLayout()
            self.btn_home = self._op_button("原点检查 home", lambda: self._op("原点检查", "GRIP_HOME"), motion=True)
            self.btn_cal = self._op_button("完整标定", lambda: self._op("完整标定", "GRIP_CALIBRATE"), motion=True)
            row.addWidget(self.btn_home)
            row.addWidget(self.btn_cal)
            ol.addLayout(row)
            hint = QLabel("home / 标定时两垫之间必须空着")
            hint.setObjectName("caption")
            ol.addWidget(hint)

            ol.addWidget(self._section("开合"))
            g = QGridLayout()
            self.pct_slider = QSlider(Qt.Horizontal)
            self.pct_slider.setRange(5, 95)
            self.pct_slider.setValue(50)
            self.pct = spin(5, 95, 50, 5, 0, " %")
            self.pct_slider.valueChanged.connect(self._slider_changed)
            self.pct_slider.sliderReleased.connect(
                lambda: self._append("> 拖动到 闭合 %d %%" % self.pct_slider.value()))
            self.pct.valueChanged.connect(self._spin_changed)
            self.move_speed = spin(0.05, 15, 2, 0.5, 2, " rad/s")
            g.addWidget(QLabel("闭合"), 0, 0)
            g.addWidget(self.pct_slider, 0, 1)
            g.addWidget(self.pct, 0, 2)
            g.addWidget(QLabel("速度"), 1, 0)
            g.addWidget(self.move_speed, 1, 1, 1, 2)
            ol.addLayout(g)
            row = QHBoxLayout()
            for text, val in (("张开 5 %", 5), ("50 %", 50), ("合拢 95 %", 95)):
                # clicked passes checked=False: without the first parameter it would land in v (0 %)
                row.addWidget(self._op_button(text, lambda _checked=False, v=val: self._move(v), motion=True))
            self.btn_move = self._op_button("移动到设定值", lambda: self._move(self.pct.value()), motion=True)
            ol.addLayout(row)
            ol.addWidget(self.btn_move)

            ol.addWidget(self._section("力控夹取"))
            g = QGridLayout()
            self.force = spin(0.5, 20, 3, 0.5, 1, " N")
            self.grip_speed = spin(0.05, 15, 2, 0.5, 2, " rad/s")
            g.addWidget(QLabel("夹持力"), 0, 0)
            g.addWidget(self.force, 0, 1)
            g.addWidget(QLabel("接近速度"), 1, 0)
            g.addWidget(self.grip_speed, 1, 1)
            ol.addLayout(g)
            self.btn_grip = self._op_button("夹取", self._grip, motion=True)
            self.btn_grip.setObjectName("primary")
            ol.addWidget(self.btn_grip)

            ol.addWidget(self._section("停止与故障"))
            row = QHBoxLayout()
            row.addWidget(self._op_button("停止", lambda: self._op("停止", "GRIP_STOP")))
            row.addWidget(self._op_button("清除故障", lambda: self._op("清除故障", "MOTOR_CLEAR_ERR")))
            ol.addLayout(row)
            self.btn_off = self._op_button("电机断电（Esc）", self._disable)
            self.btn_off.setObjectName("danger")
            ol.addWidget(self.btn_off)
            ol.addStretch(1)
            self.export_btn = QPushButton("导出数据 CSV")
            self.export_btn.clicked.connect(self._export)
            ol.addWidget(self.export_btn)
            body.addWidget(ops)

            # data
            data = QVBoxLayout()
            cards = QGridLayout()
            self.c_state = Card("状态 / 最近故障")
            self.c_pos = Card("位置", CYAN.name())
            self.c_force_t = Card("目标力")
            self.c_ring = Card("ring 垫", ORANGE.name())
            self.c_little = Card("little 垫", GREEN.name())
            self.c_torque = Card("力矩 / 速度")
            self.c_temp = Card("温度 MOS / 转子")
            self.c_motor = Card("电机")
            self.c_cal = Card("标定")
            for i, c in enumerate((self.c_state, self.c_pos, self.c_force_t, self.c_ring, self.c_little,
                                   self.c_torque, self.c_temp, self.c_motor, self.c_cal)):
                cards.addWidget(c, i // 3, i % 3)
            data.addLayout(cards)
            self.force_plot = Plot("夹持力", "N", [("ring", ORANGE), ("little", GREEN), ("目标", RED)])
            self.pos_plot = Plot("闭合程度", "%", [("闭合", CYAN)], fixed=(0.0, 100.0))
            data.addWidget(self.force_plot, 1)
            data.addWidget(self.pos_plot, 1)
            self.log = QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(2000)
            self.log.setMaximumHeight(150)
            self.log.setPlaceholderText("命令结果；USB 连接时还显示固件日志")
            data.addWidget(self.log)
            body.addLayout(data, 1)

            QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._disable)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._drain)
            self.timer.start(REDRAW_MS)
            self._kind_changed()
            self._set_enabled(False)
            self.setStyleSheet("""
                QWidget { background: #ffffff; color: #172033; font-size: 10pt; }
                QFrame#panel, QFrame#card { border: 1px solid #d8e1eb; border-radius: 9px; }
                QLabel#title { font-size: 19px; font-weight: 800; color: #10243e; }
                QLabel#section { font-weight: 800; color: #10243e; padding-top: 8px; }
                QLabel#caption { color: #64748b; font-size: 9pt; }
                QLabel#on { color: #07875c; font-weight: 700; }
                QLabel#off { color: #d83a52; font-weight: 700; }
                QPushButton, QComboBox, QDoubleSpinBox, QLineEdit { background: #ffffff; border: 1px solid #bdcad8;
                    border-radius: 6px; padding: 6px 9px; min-height: 18px; }
                QPushButton:hover { background: #eef7ff; border-color: #0284c7; }
                QPushButton:disabled { color: #a0aec0; border-color: #e2e8f0; }
                QPushButton#primary { background: #e0f2fe; border-color: #0284c7; color: #075985; font-weight: 700; }
                QPushButton#danger { background: #fde8eb; border-color: #d83a52; color: #a61b33; font-weight: 800; }
                QPlainTextEdit { background: #fbfdff; border: 1px solid #d8e1eb; border-radius: 7px;
                                 color: #526579; font-size: 9pt; padding: 5px; }
            """)

        # widgets
        def _section(self, text):
            label = QLabel(text)
            label.setObjectName("section")
            return label

        def _op_button(self, text, fn, motion=False):
            b = QPushButton(text)
            b.clicked.connect(fn)
            if not hasattr(self, "_op_buttons"):
                self._op_buttons = []
                self._motion_buttons = []
            self._op_buttons.append(b)
            if motion:
                self._motion_buttons.append(b)
            return b

        def _set_enabled(self, on):
            # motor off always works; motion buttons wait while home / calibration runs ("停止" aborts it)
            busy = self.cal_run is not None
            for b in self._op_buttons:
                b.setEnabled(b is self.btn_off or (on and not (busy and b in self._motion_buttons)))

        # connection
        def _kind_changed(self):
            self._refresh_targets()

        def _refresh_targets(self):
            kind = self.kind.currentData()
            self.target.clear()
            if kind == "usb":
                # the gripper's USB Serial/JTAG port first, Bluetooth virtual ports last
                ports = sorted(serial.tools.list_ports.comports(),
                               key=lambda q: ("USB" not in (q.description or "").upper(), q.device))
                for port in ports:
                    self.target.addItem(f"{port.device}  ·  {port.description}", port.device)
                self.refresh.setText("刷新")
            elif kind == "ble":
                self.target.addItem(cli.BLE_NAME, cli.BLE_NAME)
                self.refresh.setText("扫描")
                if self.sender() is self.refresh:
                    self._append("扫描蓝牙设备 ...")
                    self.worker.cmd_q.put(("scan",))
            else:
                self.target.addItem("192.168.4.1", "192.168.4.1")
                self.refresh.setText("刷新")

        def _toggle_connect(self):
            if self.connected:
                self.worker.cmd_q.put(("disconnect",))
                return
            target = self.target.currentData() or self.target.currentText().split()[0]
            self.worker.cmd_q.put(("connect", self.kind.currentData(), target))
            self.connect_btn.setEnabled(False)

        # operations
        def _op(self, label, name, args=b"", quiet=False):
            if not quiet:
                self._append(f"> {label}")
            if name in CAL_OPS:   # grey the buttons now, so a second click cannot restart it
                self.cal_run = {"label": label, "t0": time.time(), "seen": False}
                self._set_enabled(self.connected)
            self.worker.cmd_q.put(("op", label, name, args, quiet))

        def _move(self, pct, quiet=False):
            self._op("闭合 %.0f %%，%.2f rad/s" % (pct, self.move_speed.value()), "GRIP_SET_CLOSE",
                     struct.pack("<ff", float(pct), self.move_speed.value()), quiet)

        # slider: the jaw follows while it is dragged
        def _slider_changed(self, v):
            if self._syncing:   # set from the spin box
                return
            self._syncing = True
            self.pct.setValue(v)
            self._syncing = False
            self._live_move(v)

        def _spin_changed(self, v):
            # typed values wait for "移动到设定值"; only the slider moves the jaw at once
            if self._syncing:   # set from the slider
                return
            self._syncing = True
            self.pct_slider.setValue(int(v))
            self._syncing = False

        def _live_move(self, pct):
            if not self.connected or self.cal_run is not None:
                return
            self.live_pct = pct
            self._live_kick()

        def _live_kick(self):
            """Send the newest slider position: one move in flight, at most one per LIVE_MIN_S."""
            if self.live_busy or self.live_pct is None:
                return
            wait = self.live_t + LIVE_MIN_S - time.time()
            if wait > 0:
                QTimer.singleShot(int(wait * 1000) + 1, self._live_kick)
                return
            pct, self.live_pct = self.live_pct, None
            self.live_busy, self.live_t = True, time.time()
            self._move(pct, quiet=True)

        def _grip(self):
            self._op("夹取 %.1f N，接近 %.2f rad/s" % (self.force.value(), self.grip_speed.value()), "GRIP_GRIP",
                     struct.pack("<ff", self.force.value(), self.grip_speed.value()))

        def _disable(self):
            if self.connected:
                self._op("电机断电", "MOTOR_DISABLE")

        # events from the worker
        def _drain(self):
            while True:
                try:
                    ev = self.events.get_nowait()
                except queue.Empty:
                    self._tick()
                    return
                kind = ev[0]
                if kind == "log":
                    self._append("  | " + ev[1])
                elif kind == "status":
                    self._append(ev[1])
                elif kind == "connected":
                    self.connected = True
                    self.connected_t = time.time()
                    self.m_stale = False
                    self._status(True, ev[1])
                elif kind == "disconnected":
                    self.connected = False
                    self.cal_run = None
                    self.live_pct = None
                    self.live_busy = False   # a move lost with the link never gets its result
                    self._status(False, ev[1])
                elif kind == "result" and ev[4]:   # slider move: log refusals once per kind, send the next
                    _, label, st, name, _ = ev
                    self.live_busy = False
                    err = "" if st == "OK" else f"  拖动 {label}: {st}{self._refusal_hint(st, name)}"
                    if err and err.split(": ", 1)[1] != self.live_err.split(": ", 1)[-1]:
                        self._append(err)
                    self.live_err = err
                    self._live_kick()
                elif kind == "result":
                    _, label, st, name, _ = ev
                    hint = self._refusal_hint(st, name)
                    self._append(f"  {label}: {st}{hint}")
                    if name in CAL_OPS:
                        if st == "OK":
                            self._append("  已开始，%s。完成前开合、夹取和 home 按钮不可用，“停止”可中止" % CAL_OPS[name])
                        else:
                            self.cal_run = None
                            self.why_pending = hint == CAL_REFUSAL_NEXT   # other hints already say why
                            self._set_enabled(self.connected)
                elif kind == "scan":
                    self.target.clear()
                    for line in ev[1] or ["未找到 GripperOne"]:
                        addr = line.split()[0]
                        self.target.addItem(line, addr if ":" in addr else cli.BLE_NAME)
                    self._append("扫描完成：%d 台" % len([x for x in ev[1] if ":" in x.split()[0]]))
                elif kind == "calib":
                    self._on_calib(ev[1], ev[2], ev[3])
                elif kind == "motor":
                    self._on_motor(ev[1], ev[2])
                elif kind == "pad":
                    self._on_pad(ev[1], ev[2], ev[3])

        def _refusal_hint(self, status, name):
            """ACKs carry only a status code; say why from the latest data."""
            if status == "ERR_LOCKED":
                return "（未解锁：断开后重新连接）"
            if status != "ERR_STATE":
                return ""
            if self.last_m is None:
                return "（电机没有反馈：检查电机 24 V 电源和 CAN 线）"
            if self.last_m.get("err"):
                return "（电机故障码 0x%X：先点“清除故障”）" % self.last_m["err"]
            c = self.last_c
            if c:
                st = cli.GRIP_STATE[c["state"]] if c["state"] < len(cli.GRIP_STATE) else "?"
                if st == "CALIBRATING":
                    return "（home / 标定正在进行：等它完成，或点“停止”中止）"
                if name in CAL_OPS:
                    return CAL_REFUSAL_NEXT
                if c["fault"]:
                    return "（%s：先点“清除故障”，再 home）" % cli.state_text(c)
                if not c["valid"]:
                    return "（未标定：先 home）"
                if st == "ERROR":
                    return "（夹爪状态 ERROR）"
            return "（当前状态不允许，查看日志）"

        def _cal_refusal(self, c):
            """Why home / calibration did not start: the firmware records it as the last fault."""
            ft = cli.GRIP_FAULT[c["fault"]] if c["fault"] < len(cli.GRIP_FAULT) else str(c["fault"])
            if ft in ("motor not ready", "motor lost"):
                return "电机未就绪（查电机 24 V 电源和 CAN 线）"
            if ft == "tactile unavailable":
                return "两个指腹传感器没有同时在线（查两个垫的接线和供电，ring / little 两张卡片都应有读数）"
            if ft == "none":
                return "安全停止或电机故障被锁存（点“清除故障”后再试）"
            return cli.state_text(c)

        def _track_cal(self, t, c, st):
            """Report the end of home / calibration: its OK only means it started."""
            run = self.cal_run
            if st == "CALIBRATING":
                if run is None:   # started from another link or the CLI
                    run = self.cal_run = {"label": "标定", "t0": t, "seen": False}
                    self._set_enabled(self.connected)
                run["seen"] = True
                return
            if run is None or (not run["seen"] and t - run["t0"] < 3.0):
                return
            self.cal_run = None
            self._set_enabled(self.connected)
            if c["valid"] and not run["seen"] and run["label"] == "原点检查":
                # the firmware skips the check when already calibrated since the motor was probed
                self._append("  原点检查：电机这次上电后已有标定，没有重新检查，只移到闭合 5 %%（接近完全张开）；"
                             "标定保持：开 %+.2f / 合 %+.2f rad" % (c["pos_open"], c["pos_closed"]))
            elif c["valid"]:
                self._append("  %s完成，用时 %.0f s：开 %+.2f / 合 %+.2f rad"
                             % (run["label"], t - run["t0"], c["pos_open"], c["pos_closed"]))
            else:
                self._append("  %s没有完成（%.0f s）：%s" % (run["label"], t - run["t0"], cli.state_text(c)))

        def _status(self, on, text):
            self.status.setText(("● 已连接  " if on else "● 未连接  ") + text)
            self.status.setObjectName("on" if on else "off")
            self.status.style().unpolish(self.status)
            self.status.style().polish(self.status)
            self.connect_btn.setText("断开" if on else "连接")
            self.connect_btn.setEnabled(True)
            self._set_enabled(on)
            self._append(text)

        # replies, each handled the moment it arrives
        def _on_calib(self, t, c, full=True):
            """full: a GRIP_GET_CALIB reply; else a stream frame whose calibration
            values are the last reply's (good for display, not for reporting the
            result of a calibration that just ended)."""
            st = cli.GRIP_STATE[c["state"]] if c["state"] < len(cli.GRIP_STATE) else "?"
            if self.why_pending:
                self.why_pending = False
                self._append("  拒绝原因：" + self._cal_refusal(c))
            if full:
                self._track_cal(t, c, st)
            state = cli.state_text(c).replace(", last fault: ", " / ")
            if self.cal_run and st == "CALIBRATING":
                state += "  ·  已 %.0f s" % (t - self.cal_run["t0"])
            self.c_state.set(state)
            close_pct = None
            if c["opening"] >= 0:
                close_pct = (1.0 - c["opening"]) * 100.0
                self.c_pos.set("%+.3f rad  ·  闭合 %.1f %%" % (c["pos"], close_pct))
            else:
                self.c_pos.set("%+.3f rad  ·  未标定" % c["pos"])
            self.c_force_t.set("%.1f N（默认 %.1f N）" % (c["force_target"], c["default_force"]))
            if c["valid"]:
                self.c_cal.set("开 %+.2f / 合 %+.2f rad" % (c["pos_open"], c["pos_closed"]))
            else:
                self.c_cal.set("进行中" if st == "CALIBRATING" else "未标定，先 home")
            self.pos_plot.empty_text = "等待数据" if c["valid"] else "未标定：先 home，之后才有闭合程度"
            self.force_plot.add(2, t, c["force_target"] if c["force_target"] > 0 else None)
            self.pos_plot.add(0, t, close_pct)
            self.last_c = c
            self.records.append((t, c, self.last_m, self.last_pads.get(0), self.last_pads.get(1)))

        def _on_motor(self, t, m):
            self.c_torque.set("%+.3f N·m / %+.2f rad/s" % (m["torque"], m["vel"]))
            self.c_temp.set("%.0f / %.0f °C" % (m["t_mos"], m["t_rotor"]))
            self.c_motor.set(("使能" if m["enabled"] else "断电") + ("，故障码 0x%X" % m["err"] if m["err"] else ""))
            self.last_m = m
            self.last_m_t = t

        def _on_pad(self, t, index, fz):
            if index not in (0, 1):
                return
            self.last_pads[index] = fz
            (self.c_ring if index == 0 else self.c_little).set("%.2f N" % fz)
            self.force_plot.add(index, t, fz)

        def _tick(self):
            """Every REDRAW_MS: scroll the plots and notice a motor that stopped answering."""
            now = time.time()
            if self.connected and now - max(self.last_m_t, self.connected_t) > STALE_S and not self.m_stale:
                self.last_m = None
                self.m_stale = True
                self.c_motor.set("无反馈（查 24 V / CAN）")
            elif self.last_m is not None and now - self.last_m_t <= STALE_S:
                self.m_stale = False
            self.force_plot.update()
            self.pos_plot.update()

        def _append(self, text):
            self.log.appendPlainText(time.strftime("%H:%M:%S ") + text)

        def _export(self):
            if not self.records:
                self._append("还没有数据可导出")
                return
            path = HERE / "logs" / time.strftime("gui_%Y%m%d_%H%M%S.csv")
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "state", "fault", "position_rad", "closing_pct", "force_target_N",
                            "ring_N", "little_N", "torque_Nm", "velocity_rad_s", "motor_enabled",
                            "motor_err", "t_mos_C", "t_rotor_C"])
                t0 = self.records[0][0]
                for t, c, m, ring, little in self.records:
                    st = cli.GRIP_STATE[c["state"]] if c and c["state"] < len(cli.GRIP_STATE) else ""
                    ft = cli.GRIP_FAULT[c["fault"]] if c and c["fault"] < len(cli.GRIP_FAULT) else ""
                    w.writerow([
                        "%.3f" % (t - t0), st, ft,
                        "%.4f" % c["pos"] if c else "",
                        "%.2f" % ((1.0 - c["opening"]) * 100.0) if c and c["opening"] >= 0 else "",
                        "%.2f" % c["force_target"] if c else "",
                        "" if ring is None else "%.2f" % ring, "" if little is None else "%.2f" % little,
                        "%.4f" % m["torque"] if m else "", "%.3f" % m["vel"] if m else "",
                        m["enabled"] if m else "", m["err"] if m else "",
                        "%.0f" % m["t_mos"] if m else "", "%.0f" % m["t_rotor"] if m else "",
                    ])
            self._append("已导出 %d 行：%s" % (len(self.records), path))

        def closeEvent(self, event):
            self.worker.cmd_q.put(("disconnect",))
            time.sleep(0.2)
            event.accept()

        def load_demo(self):
            """Deterministic data for --render: 20 s at 20 Hz, a close and a 3 N grip."""
            self.connected = True
            self._status(True, "USB COM6 (demo)")
            t0 = time.time() - HISTORY_S
            n = int(HISTORY_S / POLL_S)
            for k in range(n):
                t = t0 + k * POLL_S
                u = k / n
                close = min(95.0, 5 + u * 150) if u < 0.6 else 62.0
                f = 0.0 if u < 0.6 else min(3.0, (u - 0.6) * HISTORY_S * 4) + 0.05 * math.sin(k / 3)
                c = {"valid": 1, "state": 5 if u > 0.7 else 4, "fault": 0, "pos_closed": 5.13, "pos_open": -2.09,
                     "pos_touch": 5.23, "k_force": 1852.8, "i_offset": 0.004, "i_move": 0.018,
                     "default_force": 3.0, "pos": -2.09 + close / 100 * 7.22, "opening": 1 - close / 100,
                     "force_target": 3.0 if u >= 0.6 else 0.0}
                m = {"enabled": 1, "pos": c["pos"], "vel": 0.0, "torque": 0.12, "t_mos": 29.0, "t_rotor": 28.0,
                     "err": 0}
                self._on_motor(t, m)
                self._on_pad(t, 0, f)
                self._on_pad(t, 1, f * 0.85)
                self._on_calib(t, c)
            self._append("> 夹取 3.0 N，接近 2.00 rad/s")
            self._append("  夹取: OK")
            self._append("  | I (336829) gripper: holding 2.90 N (target 3.0 N) at 0.015 pu")

    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    installed = set(QFontDatabase.families())
    UI_FONT = next((f for f in ("Noto Sans SC", "Microsoft YaHei UI", "Microsoft YaHei",
                                "Source Han Sans SC", "SimHei") if f in installed), app.font().family())
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

        QTimer.singleShot(400, save_and_quit)
    else:
        window.show()
    return app.exec()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--render", metavar="PNG", help="render a demo screenshot and exit")
    args = parser.parse_args()
    if args.render:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    return run_gui(args.render)


if __name__ == "__main__":
    sys.exit(main())
