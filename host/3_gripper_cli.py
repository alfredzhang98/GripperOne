#!/usr/bin/env python3
"""Command-line bench tool for the gripper (motor + tactile) over app_proto.

The firmware log and the app_proto frames share the USB serial port; this tool
prints the log lines and decodes the frames. Close idf.py monitor first (only
one program can open the port).

    python host/3_gripper_cli.py log                  # firmware log only
    python host/3_gripper_cli.py watch                # motor position + pad forces, motor stays off
    python host/3_gripper_cli.py calibrate            # full calibration (0x40), saved to flash
    python host/3_gripper_cli.py home                 # pad-touch check, then the stored calibration (0x41)
    python host/3_gripper_cli.py calib                # print the active calibration (0x47)
    python host/3_gripper_cli.py grip 3 [--speed 1]   # grip at 3 N (0x43)
    python host/3_gripper_cli.py close 50 [--speed 1] # to 50 % closing, 5 = most open, 95 = most closed (0x42)
    python host/3_gripper_cli.py cycle [--count 100] [--speed 5]  # clear, home, then 95 % <-> 5 % (log in host/logs/)
    python host/3_gripper_cli.py stop | disable | clear
    --speed: rad/s at the output, 0.05 .. 15; not given = 2
    python host/3_gripper_cli.py save-motor-config    # store mode 4 + TIMEOUT in the motor (0x17)

Every command takes --port (default COM6). Ctrl+C while a command runs sends
MOTOR_DISABLE (0x11): the motor goes limp at once.
"""

import argparse
import os
import re
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

SOF = b"\xA5\x5A"
VERSION = 0x01
HDR_LEN = 7
MAX_PAYLOAD = 400
T_COMMAND, T_RESPONSE, T_TELEMETRY, T_ACK, T_ERROR = range(5)
SESSION_KEY = 0x47521000

OP = {
    "SAFE_STOP": 0x06, "SESSION_UNLOCK": 0x07,
    "MOTOR_DISABLE": 0x11, "MOTOR_CLEAR_ERR": 0x15, "MOTOR_GET_STATUS": 0x16, "MOTOR_SAVE_CONFIG": 0x17,
    "HAPTIC_GET": 0x32,
    "GRIP_CALIBRATE": 0x40, "GRIP_HOME": 0x41, "GRIP_SET_CLOSE": 0x42, "GRIP_GRIP": 0x43,
    "GRIP_RELEASE": 0x44, "GRIP_STOP": 0x46, "GRIP_GET_CALIB": 0x47,
}
OP_NAME = {v: k for k, v in OP.items()}
STATUS = {0: "OK", 1: "ERR_CRC", 2: "ERR_VERSION", 3: "ERR_OPCODE", 4: "ERR_LOCKED", 5: "ERR_STATE", 6: "ERR_ARG"}
# gripper_state_t / gripper_fault_t, mirrored from components/gripper/include/gripper.h
GRIP_STATE = ["UNCAL", "IDLE", "CALIBRATING", "MOVING", "GRIPPING", "HOLDING", "ERROR"]
GRIP_FAULT = [
    "none", "motor not ready", "motor lost", "motor fault", "motor not enabled", "tactile unavailable",
    "tactile stale while holding", "force rose while opening", "jaw does not move", "travel limit", "timeout",
    "tactile tare failed", "tactile not zero after tare", "one-sided contact", "stalled without tactile contact",
    "contact force too high", "force/current gain invalid", "pads did not separate", "stroke invalid",
    "over-force", "contact during move", "empty grasp", "grip stalled without pad force", "position jump",
    "speed mismatch", "pad force far below current", "tactile re-zeroed while driving", "stroke changed",
    "position frame changed",
]
ANSI = re.compile(rb"\x1b\[[0-9;]*m")


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


class Link:
    """Serial port that splits the byte stream into log lines and frames."""

    def __init__(self, port: str, quiet_log: bool = False):
        # DTR and RTS stay low from open to close: on Windows, opening or
        # closing with them high pulses the board's auto-reset circuit, and a
        # restart with the motor holding makes the drive report "CAN commands
        # lost" (fault 0xD), which then blocks every motion until `clear`.
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.02
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.buf = bytearray()
        self.text = bytearray()  # log text cut off by a frame, completed later
        self.seq = 0
        self.quiet_log = quiet_log
        self.motor = None        # latest motor status dict
        self.pads = {}           # sensor index -> fz resultant (N)
        self.calib = None
        self.acks = {}           # seq -> (op, status)

    def send(self, op: int, args: bytes = b"") -> int:
        self.seq = (self.seq + 1) & 0xFF
        self.ser.write(build(T_COMMAND, self.seq, bytes((op,)) + args))
        return self.seq

    def poll(self, duration: float = 0.0):
        end = time.time() + duration
        while True:
            data = self.ser.read(4096)
            if data:
                self.buf += data
                self._parse()
            if time.time() >= end:
                return

    def _parse(self):
        while True:
            i = self.buf.find(SOF)
            text = self.buf if i < 0 else self.buf[:i]
            # print complete text lines; keep a partial line for later
            nl = text.rfind(b"\n")
            if nl >= 0:
                self._log(bytes(self.text + text[:nl + 1]))
                self.text.clear()
                del self.buf[:nl + 1]
                continue
            if i < 0:
                if len(self.buf) > 2048:
                    self._log(bytes(self.text + self.buf))
                    self.text.clear()
                    self.buf.clear()
                return
            if i > 0:
                # text cut off by a frame: keep it until its line ends
                self.text += self.buf[:i]
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                return
            plen = struct.unpack_from("<H", self.buf, 5)[0]
            if plen > MAX_PAYLOAD:
                del self.buf[:1]
                continue
            n = HDR_LEN + plen + 2
            if len(self.buf) < n:
                return
            body = bytes(self.buf[2:HDR_LEN + plen])
            crc = struct.unpack_from("<H", self.buf, HDR_LEN + plen)[0]
            if crc != crc16(body):
                del self.buf[:1]
                continue
            ftype, seq = self.buf[3], self.buf[4]
            payload = bytes(self.buf[HDR_LEN:HDR_LEN + plen])
            del self.buf[:n]
            self._frame(ftype, seq, payload)

    def _log(self, raw: bytes):
        if self.quiet_log:
            return
        for line in ANSI.sub(b"", raw).decode("utf-8", "replace").splitlines():
            line = line.rstrip()
            if line:
                print("  | " + line)

    def _frame(self, ftype: int, seq: int, pl: bytes):
        if not pl:
            return
        op = pl[0]
        if ftype in (T_ACK, T_ERROR) and len(pl) >= 2:
            self.acks[seq] = (op, pl[1])
        elif op == OP["MOTOR_GET_STATUS"] and len(pl) >= 23:
            en = pl[1]
            pos, vel, tq, tmos, trot = struct.unpack_from("<5f", pl, 2)
            self.motor = {"enabled": en, "pos": pos, "vel": vel, "torque": tq, "t_mos": tmos,
                          "t_rotor": trot, "err": pl[22], "t": time.time()}
        elif op == OP["HAPTIC_GET"] and len(pl) >= 25:
            idx = pl[1]
            fz = struct.unpack_from("<3f", pl, 13)[2]
            self.pads[idx] = fz
        elif op == OP["GRIP_GET_CALIB"] and len(pl) >= 44:
            v = struct.unpack_from("<10f", pl, 4)
            self.calib = {"valid": pl[1], "state": pl[2], "fault": pl[3], "pos_closed": v[0], "pos_open": v[1],
                          "pos_touch": v[2], "k_force": v[3], "i_offset": v[4], "i_move": v[5],
                          "default_force": v[6], "pos": v[7], "opening": v[8], "force_target": v[9]}

    def command(self, name: str, args: bytes = b"", timeout: float = 1.0) -> str:
        seq = self.send(OP[name], args)
        end = time.time() + timeout
        while time.time() < end:
            self.poll(0.02)
            if seq in self.acks:
                op, st = self.acks.pop(seq)
                return STATUS.get(st, "0x%02X" % st)
        return "no reply"

    def unlock(self) -> bool:
        st = self.command("SESSION_UNLOCK", struct.pack("<I", SESSION_KEY))
        print("unlock:", st)
        return st == "OK"

    def get_calib(self, timeout: float = 1.0):
        self.calib = None
        self.send(OP["GRIP_GET_CALIB"])
        end = time.time() + timeout
        while time.time() < end and self.calib is None:
            self.poll(0.02)
        return self.calib


def state_text(c) -> str:
    st = GRIP_STATE[c["state"]] if c["state"] < len(GRIP_STATE) else str(c["state"])
    ft = GRIP_FAULT[c["fault"]] if c["fault"] < len(GRIP_FAULT) else str(c["fault"])
    return "%s, last fault: %s" % (st, ft)


def print_calib(c):
    if c is None:
        print("no GRIP_GET_CALIB reply")
        return
    print("state      : " + state_text(c))
    print("calibrated : %s" % ("yes" if c["valid"] else "no"))
    if c["valid"]:
        span = c["pos_open"] - c["pos_closed"]
        print("open       : %+.4f rad" % c["pos_open"])
        print("closed     : %+.4f rad   (stroke %.4f rad = %.1f deg)" % (c["pos_closed"], abs(span),
                                                                         abs(span) * 57.2958))
        print("pad touch  : %+.4f rad" % c["pos_touch"])
        print("k          : %.1f N per unit current, offset %.4f pu, moving current %.4f pu"
              % (c["k_force"], c["i_offset"], c["i_move"]))
    print("position   : %+.4f rad, opening %s, force target %.1f N, default force %.1f N"
          % (c["pos"], "n/a" if c["opening"] < 0 else "%.2f" % c["opening"], c["force_target"],
             c["default_force"]))


def follow(link: Link, until_states, max_s: float):
    """Print the log and the gripper state until one of until_states (or timeout)."""
    t0 = time.time()
    last = None
    while time.time() - t0 < max_s:
        c = link.get_calib(0.5)
        if c:
            s = state_text(c)
            if s != last:
                print(">> %5.1f s  %s" % (time.time() - t0, s))
                last = s
            if GRIP_STATE[c["state"]] in until_states and time.time() - t0 > 1.0:
                return c
        link.poll(0.3)
    print(">> stopped waiting after %.0f s" % max_s)
    return None


def cmd_watch(link: Link, args):
    print("Motor stays disabled. Turn the jaw slowly by hand; Ctrl+C ends.")
    print("Position change from the start: + means the drive reads a larger angle.")
    link.quiet_log = not args.log
    p0 = None
    t_print = 0.0
    while True:
        link.send(OP["MOTOR_GET_STATUS"])
        link.send(OP["HAPTIC_GET"])
        link.poll(0.1)
        m = link.motor
        if m is None or time.time() - t_print < 0.2:
            continue
        t_print = time.time()
        if p0 is None:
            p0 = m["pos"]
        d = m["pos"] - p0
        print("pos %+8.4f rad (%+7.2f deg) | change %+8.4f rad (%+7.2f deg) | vel %+6.2f rad/s | "
              "torque %+6.3f N*m | enabled %d err %d | ring %5.2f N  little %5.2f N"
              % (m["pos"], m["pos"] * 57.2958, d, d * 57.2958, m["vel"], m["torque"], m["enabled"],
                 m["err"], link.pads.get(0, float("nan")), link.pads.get(1, float("nan"))))


class Tee:
    """Print to the console and append to a log file."""

    def __init__(self, console, path: str):
        self.console = console
        self.file = open(path, "a", encoding="utf-8")

    def write(self, s):
        self.console.write(s)
        self.file.write(s)
        self.file.flush()

    def flush(self):
        self.console.flush()
        self.file.flush()


def motor_status(link: Link):
    link.motor = None
    link.send(OP["MOTOR_GET_STATUS"])
    end = time.time() + 0.5
    while time.time() < end and link.motor is None:
        link.poll(0.02)
    return link.motor


def wait_move(link: Link, want_opening: float, max_s: float):
    """Poll until the jaw is idle at want_opening, or a fault, or max_s."""
    t0 = time.time()
    while time.time() - t0 < max_s:
        c = link.get_calib(0.5)
        if c and time.time() - t0 > 0.1:
            state = GRIP_STATE[c["state"]] if c["state"] < len(GRIP_STATE) else "?"
            if c["fault"] != 0 or state == "ERROR":
                return c
            if state == "IDLE" and abs(c["opening"] - want_opening) < 0.02:
                return c
        link.poll(0.05)
    return None


def cmd_cycle(link: Link, args) -> int:
    """home, then args.count round trips high -> low; stops at the first fault."""
    path = args.log_file or time.strftime("host/logs/cycle_%Y%m%d_%H%M%S.log")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    sys.stdout = Tee(sys.stdout, path)
    print("cycle: %d round trips %.0f %% <-> %.0f %% at %s, log %s"
          % (args.count, args.high, args.low, "%.2f rad/s" % args.speed if args.speed > 0 else "the default speed",
             path))
    if not link.unlock():
        return 1
    m = motor_status(link)
    c = link.get_calib()
    # a latched fault blocks every motion; clear only then (the firmware refuses
    # a clear while it drives the motor)
    if (m and m["err"]) or (c and GRIP_STATE[c["state"]] == "ERROR"):
        print("motor error code 0x%X, gripper %s before the start"
              % (m["err"] if m else 0, state_text(c) if c else "?"))
        print("MOTOR_CLEAR_ERR:", link.command("MOTOR_CLEAR_ERR"))
    if args.no_home:
        c = link.get_calib()
    else:
        st = link.command("GRIP_HOME")
        print("GRIP_HOME:", st)
        if st != "OK":
            link.poll(0.5)
            return 1
        c = follow(link, ("IDLE", "UNCAL", "ERROR"), 600.0)
        link.poll(0.5)
    print_calib(c)
    if not c or not c["valid"] or c["fault"] != 0 or GRIP_STATE[c["state"]] != "IDLE":
        print("cycle: home did not finish cleanly; not starting")
        return 1
    m0 = motor_status(link)
    times = {args.high: [], args.low: []}
    for i in range(1, args.count + 1):
        parts = []
        for target in (args.high, args.low):
            t0 = time.time()
            st = link.command("GRIP_SET_CLOSE", struct.pack("<ff", target, args.speed))
            if st != "OK":
                print("cycle %d: GRIP_SET_CLOSE %.0f -> %s; stopping" % (i, target, st))
                link.poll(0.5)
                return 1
            want = 1.0 - target / 100.0
            c = wait_move(link, want, 30.0)
            dt = time.time() - t0
            if c is None or c["fault"] != 0 or abs(c["opening"] - want) >= 0.02:
                print("cycle %d: move to %.0f %% did not finish cleanly (%s); stopping"
                      % (i, target, state_text(c) if c else "no reply"))
                link.poll(1.0)
                return 1
            times[target].append(dt)
            parts.append("to %2.0f %%: %.2f s (opening %.3f)" % (target, dt, c["opening"]))
        line = "cycle %3d/%d  %s" % (i, args.count, "   ".join(parts))
        if i % 10 == 0:
            m = motor_status(link)
            if m:
                line += "   MOS %.0f C, rotor %.0f C" % (m["t_mos"], m["t_rotor"])
        print(line)
    m1 = motor_status(link)
    print("cycle: %d round trips done" % args.count)
    for target, ts in times.items():
        print("  to %2.0f %%: mean %.2f s, max %.2f s (CLI polling adds up to about 0.5 s; the firmware's "
              "'move done' lines give the exact time)" % (target, sum(ts) / len(ts), max(ts)))
    if m0 and m1:
        print("  temperature: MOS %.0f -> %.0f C, rotor %.0f -> %.0f C"
              % (m0["t_mos"], m1["t_mos"], m0["t_rotor"], m1["t_rotor"]))
    return 0


def closing_percent(text: str) -> float:
    v = float(text)
    if not 5.0 <= v <= 95.0:
        raise argparse.ArgumentTypeError("closing percent must be 5 .. 95")
    return v


def main():
    ap = argparse.ArgumentParser(description="Gripper bench CLI (app_proto over USB serial)")
    ap.add_argument("--port", default="COM6")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("log", help="print the firmware log")
    w = sub.add_parser("watch", help="motor position and pad forces, motor stays off")
    w.add_argument("--log", action="store_true", help="also print the firmware log")
    sub.add_parser("calibrate", help="full calibration (0x40), saved to flash")
    sub.add_parser("home", help="pad-touch check, then the stored calibration (0x41)")
    sub.add_parser("calib", help="print the active calibration (0x47)")
    g = sub.add_parser("grip", help="grip at a pad force (0x43)")
    g.add_argument("force", type=float, help="N, 0.5 .. 20; 0 = stored default")
    g.add_argument("--speed", type=float, default=0.0, help="approach speed, rad/s at the output, 0.05 .. 15; not given = 2")
    o = sub.add_parser("close", help="move to a closing percentage (0x42)")
    o.add_argument("percent", type=closing_percent,
                   help="5 = most open .. 95 = most closed (0 / 100 are the calibrated ends, kept as margin)")
    o.add_argument("--speed", type=float, default=0.0,
                   help="rad/s at the output, 0.05 .. 15; not given = 2")
    cy = sub.add_parser("cycle", help="home, then move between two closing percentages N times; "
                                      "everything is also written to host/logs/")
    cy.add_argument("--count", type=int, default=100, help="round trips")
    cy.add_argument("--high", type=closing_percent, default=95.0, help="closing percent, default 95")
    cy.add_argument("--low", type=closing_percent, default=5.0, help="closing percent, default 5")
    cy.add_argument("--speed", type=float, default=0.0, help="rad/s at the output, 0.05 .. 15; not given = 2")
    cy.add_argument("--no-home", action="store_true", help="skip GRIP_HOME (already homed since power-up)")
    cy.add_argument("--log-file", default=None)
    sub.add_parser("stop", help="hold the current position (0x46)")
    sub.add_parser("disable", help="motor off now (0x11)")
    sub.add_parser("clear", help="clear a latched fault (0x15)")
    sub.add_parser("save-motor-config", help="store mode 4 + TIMEOUT in the motor's flash (0x17)")
    args = ap.parse_args()

    link = Link(args.port)
    try:
        if args.cmd == "log":
            print("firmware log; Ctrl+C ends")
            while True:
                link.poll(0.5)
        elif args.cmd == "watch":
            cmd_watch(link, args)
        elif args.cmd == "calib":
            link.poll(0.2)
            print_calib(link.get_calib())
        elif args.cmd in ("calibrate", "home"):
            if not link.unlock():
                return 1
            name = "GRIP_CALIBRATE" if args.cmd == "calibrate" else "GRIP_HOME"
            st = link.command(name)
            print("%s: %s" % (name, st))
            if st != "OK":
                link.poll(0.5)
                return 1
            print("Running. Ctrl+C = motor off at once.")
            # the first calibration crosses the 6.9 rad stroke twice at 0.05 rad/s
            c = follow(link, ("IDLE", "UNCAL", "ERROR"), 600.0)
            link.poll(1.0)
            print_calib(link.get_calib() or c)
        elif args.cmd in ("grip", "close"):
            if not link.unlock():
                return 1
            if args.cmd == "grip":
                st = link.command("GRIP_GRIP", struct.pack("<ff", args.force, args.speed))
            else:
                st = link.command("GRIP_SET_CLOSE", struct.pack("<ff", args.percent, args.speed))
            print("%s: %s" % (args.cmd, st))
            if st != "OK":
                link.poll(0.5)
                return 1
            print("Ctrl+C = motor off at once.")
            follow(link, ("HOLDING", "IDLE", "ERROR"), 300.0)
            link.poll(0.5)
        elif args.cmd == "cycle":
            return cmd_cycle(link, args)
        elif args.cmd == "stop":
            print("GRIP_STOP:", link.command("GRIP_STOP"))
        elif args.cmd == "disable":
            print("MOTOR_DISABLE:", link.command("MOTOR_DISABLE"))
        elif args.cmd == "clear":
            if link.unlock():
                print("MOTOR_CLEAR_ERR:", link.command("MOTOR_CLEAR_ERR"))
        elif args.cmd == "save-motor-config":
            print("Writes CTRL_MODE = 4 and the comms-loss TIMEOUT into the motor's own flash.")
            print("The motor's flash allows about 10000 writes: do this once, not routinely.")
            if input("type yes to continue: ").strip() != "yes":
                return 1
            if link.unlock():
                print("MOTOR_DISABLE:", link.command("MOTOR_DISABLE"))
                print("MOTOR_SAVE_CONFIG:", link.command("MOTOR_SAVE_CONFIG", timeout=3.0))
                link.poll(0.5)
        return 0
    except KeyboardInterrupt:
        if args.cmd not in ("log", "watch", "calib"):
            print("\nCtrl+C: MOTOR_DISABLE ->", link.command("MOTOR_DISABLE"))
        else:
            print()
        return 0
    finally:
        link.ser.close()


if __name__ == "__main__":
    sys.exit(main())
