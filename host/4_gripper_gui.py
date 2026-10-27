#!/usr/bin/env python3
"""PySide6 control panel for the gripper: one-click operations and live data.

Connects over USB, Bluetooth LE or Wi-Fi (TCP) with the same links as
host/3_gripper_cli.py. Polls state, position, motor and both pads five times a
second, plots the last 20 s and exports what it recorded to CSV.

    python host/4_gripper_gui.py
    python host/4_gripper_gui.py --render tmp/gripper_gui.png   # offscreen screenshot with demo data

Esc = motor off at once (MOTOR_DISABLE), on any link.
"""

import argparse
import csv
import importlib.util
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

POLL_S = 0.2          # one round of state + motor + pads requests
HISTORY_S = 20.0      # plotted window
RECORD_MAX = 30000    # rows kept for CSV export (100 min at 5 Hz)


# link worker -----------------------------------------------------------------

class GuiLink(cli.Link):
    """cli.Link whose firmware log lines go to the GUI instead of stdout."""

    def __init__(self, io, events):
        super().__init__(io)
        self.events = events

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
        self.next_poll = 0.0

    def run(self):
        while True:
            try:
                item = self.cmd_q.get(timeout=0.02)
            except queue.Empty:
                item = None
            try:
                if item:
                    self._handle(item)
                if self.link and time.time() >= self.next_poll:
                    self.next_poll = time.time() + POLL_S
                    self._poll()
            except (OSError, ConnectionError) as e:
                self._drop("connection lost: %s" % e)

    def _drop(self, why):
        if self.link:
            try:
                self.link.ser.close()
            except Exception:
                pass
        self.link = None
        self.events.put(("disconnected", why))

    def _handle(self, item):
        kind = item[0]
        if kind == "connect":
            _, link_kind, target = item
            if self.link:
                self._drop("reconnecting")
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
            self.events.put(("connected", "%s %s (unlock %s)" % (link_kind.upper(), target, st)))
        elif kind == "disconnect":
            self._drop("disconnected")
        elif kind == "op" and self.link:
            _, label, name, args = item
            st = self.link.command(name, args, timeout=3.0 if name == "MOTOR_SAVE_CONFIG" else 1.0)
            if st == "ERR_LOCKED":   # session relocked (e.g. after a reconnect): unlock and retry once
                self.link.command("SESSION_UNLOCK", struct.pack("<I", cli.SESSION_KEY))
                st = self.link.command(name, args)
            self.events.put(("result", label, st))
        elif kind == "scan":
            self.events.put(("scan", ble_scan()))

    def _poll(self):
        link = self.link
        link.calib = None
        link.motor = None
        link.pads = {}
        link.send(cli.OP["GRIP_GET_CALIB"])
        link.send(cli.OP["MOTOR_GET_STATUS"])
        link.send(cli.OP["HAPTIC_GET"])
        end = time.time() + 0.15
        while time.time() < end and (link.calib is None or link.motor is None or len(link.pads) < 2):
            link.poll(0.02)
        self.events.put(("data", time.time(), link.calib, link.motor, dict(link.pads)))


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
        """Time series of the last HISTORY_S seconds; series = [(label, color, deque)]."""

        def __init__(self, title, unit, series, fixed=None):
            super().__init__()
            self.title, self.unit, self.series, self.fixed = title, unit, series, fixed
            self.times = deque(maxlen=int(HISTORY_S / POLL_S * 3))
            self.setMinimumHeight(170)

        def add(self, t, values):
            self.times.append(t)
            for (_, _, dq), v in zip(self.series, values):
                dq.append(v)
            self.update()

        def paintEvent(self, _e):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor("#fbfdff"))
            plot = QRectF(44, 24, self.width() - 56, self.height() - 40)
            p.setPen(QPen(QColor("#e1e8f0"), 1))
            for i in range(5):
                y = plot.top() + plot.height() * i / 4
                p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(MUTED)
            p.setFont(QFont(UI_FONT, 8))
            fm = p.fontMetrics()
            head = f"{self.title} ({self.unit})"
            p.drawText(QRectF(8, 3, 400, 18), Qt.AlignLeft, head)
            x = 8 + fm.horizontalAdvance(head) + 16
            for label, color, _ in self.series:
                p.fillRect(QRectF(x, 8, 8, 8), color)
                x += 12
                p.drawText(QRectF(x, 3, 200, 18), Qt.AlignLeft, label)
                x += fm.horizontalAdvance(label) + 14
            pts = [(i, t) for i, t in enumerate(self.times) if t >= (self.times[-1] - HISTORY_S if self.times else 0)]
            vals = [dq[i] for _, _, dq in self.series for i, _ in pts if i < len(dq) and dq[i] is not None]
            if len(pts) < 2 or not vals:
                p.drawText(plot, Qt.AlignCenter, "等待数据")
                return
            lo, hi = self.fixed if self.fixed else (min(min(vals), 0.0), max(max(vals), 1.0))
            if not self.fixed:
                hi += (hi - lo) * 0.1
            t1 = self.times[-1]
            t0 = t1 - HISTORY_S
            for _, color, dq in self.series:
                path = QPainterPath()
                started = False
                for i, t in pts:
                    v = dq[i] if i < len(dq) else None
                    if v is None:
                        started = False
                        continue
                    x = plot.left() + (t - t0) / HISTORY_S * plot.width()
                    y = plot.bottom() - (min(max(v, lo), hi) - lo) / (hi - lo) * plot.height()
                    if started:
                        path.lineTo(x, y)
                    else:
                        path.moveTo(x, y)
                        started = True
                p.setPen(QPen(color, 1.8))
                p.drawPath(path)
            p.setPen(MUTED)
            for v, y in ((hi, plot.top()), (lo, plot.bottom())):
                p.drawText(QRectF(2, y - 8, 38, 16), Qt.AlignRight, f"{0.0 if abs(v) < 0.05 else v:.1f}")
            p.drawText(QRectF(plot.left(), plot.bottom() + 2, plot.width(), 14), Qt.AlignRight, "现在")
            p.drawText(QRectF(plot.left(), plot.bottom() + 2, plot.width(), 14), Qt.AlignLeft, f"-{HISTORY_S:.0f} s")

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
            self.btn_home = self._op_button("原点检查 home", lambda: self._op("原点检查", "GRIP_HOME"))
            self.btn_cal = self._op_button("完整标定", lambda: self._op("完整标定", "GRIP_CALIBRATE"))
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
            self.pct_slider.valueChanged.connect(lambda v: self.pct.setValue(v))
            self.pct.valueChanged.connect(lambda v: self.pct_slider.setValue(int(v)))
            self.move_speed = spin(0.05, 15, 2, 0.5, 2, " rad/s")
            g.addWidget(QLabel("闭合"), 0, 0)
            g.addWidget(self.pct_slider, 0, 1)
            g.addWidget(self.pct, 0, 2)
            g.addWidget(QLabel("速度"), 1, 0)
            g.addWidget(self.move_speed, 1, 1, 1, 2)
            ol.addLayout(g)
            row = QHBoxLayout()
            for text, val in (("张开 5 %", 5), ("50 %", 50), ("合拢 95 %", 95)):
                row.addWidget(self._op_button(text, lambda v=val: self._move(v)))
            self.btn_move = self._op_button("移动到设定值", lambda: self._move(self.pct.value()))
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
            self.btn_grip = self._op_button("夹取", self._grip)
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
            self.h_ring, self.h_little, self.h_target = deque(maxlen=400), deque(maxlen=400), deque(maxlen=400)
            self.h_close = deque(maxlen=400)
            self.force_plot = Plot("夹持力", "N", [("ring", ORANGE, self.h_ring), ("little", GREEN, self.h_little),
                                                   ("目标", RED, self.h_target)])
            self.pos_plot = Plot("闭合程度", "%", [("闭合", CYAN, self.h_close)], fixed=(0.0, 100.0))
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
            self.timer.start(50)
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

        def _op_button(self, text, fn):
            b = QPushButton(text)
            b.clicked.connect(fn)
            if not hasattr(self, "_op_buttons"):
                self._op_buttons = []
            self._op_buttons.append(b)
            return b

        def _set_enabled(self, on):
            for b in self._op_buttons:
                b.setEnabled(on or b is self.btn_off)

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
        def _op(self, label, name, args=b""):
            self._append(f"> {label}")
            self.worker.cmd_q.put(("op", label, name, args))

        def _move(self, pct):
            self._op("闭合 %.0f %%，%.2f rad/s" % (pct, self.move_speed.value()), "GRIP_SET_CLOSE",
                     struct.pack("<ff", float(pct), self.move_speed.value()))

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
                    return
                kind = ev[0]
                if kind == "log":
                    self._append("  | " + ev[1])
                elif kind == "status":
                    self._append(ev[1])
                elif kind == "connected":
                    self.connected = True
                    self._status(True, ev[1])
                elif kind == "disconnected":
                    self.connected = False
                    self._status(False, ev[1])
                elif kind == "result":
                    self._append(f"  {ev[1]}: {ev[2]}{self._refusal_hint(ev[2])}")
                elif kind == "scan":
                    self.target.clear()
                    for line in ev[1] or ["未找到 GripperOne"]:
                        addr = line.split()[0]
                        self.target.addItem(line, addr if ":" in addr else cli.BLE_NAME)
                    self._append("扫描完成：%d 台" % len([x for x in ev[1] if ":" in x.split()[0]]))
                elif kind == "data":
                    self._show(*ev[1:])

        def _refusal_hint(self, status):
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
                if c["fault"]:
                    return "（%s：先点“清除故障”，再 home）" % cli.state_text(c)
                if not c["valid"]:
                    return "（未标定：先 home）"
                if st in ("CALIBRATING", "ERROR"):
                    return "（夹爪状态 %s）" % st
            return "（当前状态不允许，查看日志）"

        def _status(self, on, text):
            self.status.setText(("● 已连接  " if on else "● 未连接  ") + text)
            self.status.setObjectName("on" if on else "off")
            self.status.style().unpolish(self.status)
            self.status.style().polish(self.status)
            self.connect_btn.setText("断开" if on else "连接")
            self.connect_btn.setEnabled(True)
            self._set_enabled(on)
            self._append(text)

        def _show(self, t, c, m, pads):
            ring = pads.get(0)
            little = pads.get(1)
            close_pct = target = None
            if c:
                self.c_state.set(cli.state_text(c).replace(", last fault: ", " / "))
                if c["opening"] >= 0:
                    close_pct = (1.0 - c["opening"]) * 100.0
                    self.c_pos.set("%+.3f rad  ·  闭合 %.1f %%" % (c["pos"], close_pct))
                else:
                    self.c_pos.set("%+.3f rad  ·  未标定" % c["pos"])
                target = c["force_target"] if c["force_target"] > 0 else None
                self.c_force_t.set("%.1f N（默认 %.1f N）" % (c["force_target"], c["default_force"]))
                if c["valid"]:
                    self.c_cal.set("开 %+.2f / 合 %+.2f rad" % (c["pos_open"], c["pos_closed"]))
                else:
                    self.c_cal.set("未标定，先 home")
            if ring is not None:
                self.c_ring.set("%.2f N" % ring)
            if little is not None:
                self.c_little.set("%.2f N" % little)
            if m:
                self.c_torque.set("%+.3f N·m / %+.2f rad/s" % (m["torque"], m["vel"]))
                self.c_temp.set("%.0f / %.0f °C" % (m["t_mos"], m["t_rotor"]))
                self.c_motor.set(("使能" if m["enabled"] else "断电") + ("，故障码 0x%X" % m["err"] if m["err"] else ""))
            self.last_c = c
            self.last_m = m
            if m is None:
                self.c_motor.set("无反馈（查 24 V / CAN）")
            self.force_plot.add(t, (ring, little, target))
            self.pos_plot.add(t, (close_pct,))
            self.records.append((t, c, m, ring, little))

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
            """Deterministic data for --render."""
            import math
            self.connected = True
            self._status(True, "USB COM6 (demo)")
            t0 = time.time() - HISTORY_S
            for k in range(int(HISTORY_S / POLL_S)):
                t = t0 + k * POLL_S
                close = min(95.0, 5 + k * 1.2) if k < 60 else 62.0
                f = 0.0 if k < 60 else min(3.0, (k - 60) * 0.4) + 0.05 * math.sin(k)
                c = {"valid": 1, "state": 5 if k > 70 else 4, "fault": 0, "pos_closed": 5.13, "pos_open": -2.09,
                     "pos_touch": 5.23, "k_force": 1852.8, "i_offset": 0.004, "i_move": 0.018,
                     "default_force": 3.0, "pos": -2.09 + close / 100 * 7.22, "opening": 1 - close / 100,
                     "force_target": 3.0 if k >= 60 else 0.0}
                m = {"enabled": 1, "pos": c["pos"], "vel": 0.0, "torque": 0.12, "t_mos": 29.0, "t_rotor": 28.0,
                     "err": 0}
                self._show(t, c, m, {0: f, 1: f * 0.85})
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
