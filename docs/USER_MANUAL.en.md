[中文](USER_MANUAL.md) | **English**

# GripperOne Gripper User Manual

This manual explains how to connect the gripper, do the first-time setup, and control opening, closing and gripping with commands. For the control principles and parameters, see [GRIPPER_CONTROL.en.md](GRIPPER_CONTROL.en.md).

## 1. Connection options

The gripper supports three connection types. All three use the same command set and can be connected at the same time; each connection is unlocked separately and receives its own replies.

| Type | Current firmware | Best for | Notes |
| --- | --- | --- | --- |
| USB | Enabled | Debugging, flashing, viewing logs | The only way to see the firmware log |
| Bluetooth (BLE) | Enabled | Wireless control | Device name `GripperOne`; only one computer can connect at a time |
| Wi-Fi | Implemented, off by default | Longer range or joining a local network | See Section 3.3 to enable it |

## 2. Preparation

**Hardware:**

1. Connect the 24 V supply for the motor.
2. Connect the gripper controller board to the computer with a USB-C cable (if you only use Bluetooth, power alone is enough).
3. Keep the space between the two tactile pads empty.

**Computer software (Windows / macOS / Linux):**

```powershell
pip install -r host/requirements.txt
```

- This installs PySide6 (GUI), pyserial (USB) and bleak (Bluetooth).
- There are two tools with the same functions:
  - the GUI `host/4_gripper_gui.py`, see Section 5.1;
  - the command line `host/3_gripper_cli.py`.
- All commands below are run from the repository root.

## 3. Connecting

### 3.1 USB

1. Find the gripper's serial port in Device Manager, e.g. COM6.
2. Before running a command, close `idf.py monitor` and any other program using that port.
3. Run:

```powershell
python host/3_gripper_cli.py --port COM6 calib
```

`--port` defaults to COM6 and can be omitted if that is your port.

### 3.2 Bluetooth

1. Turn on the computer's Bluetooth. Do **not** "pair" the gripper in the system Bluetooth settings; the command-line tool connects on its own.
2. Find nearby grippers:

```powershell
python host/3_gripper_cli.py ble-scan
```

The output looks like `58:E6:C5:14:05:42  GripperOne`.

3. Add `--ble` before any command:

```powershell
python host/3_gripper_cli.py --ble calib
python host/3_gripper_cli.py --ble close 50
```

4. With several grippers, use `--ble-name` to select one by address:

```powershell
python host/3_gripper_cli.py --ble --ble-name 58:E6:C5:14:05:42 calib
```

- Connecting takes about 2–5 s, and every command reconnects.
- Bluetooth accepts only one computer at a time. While another computer is connected, `ble-scan` will not find the gripper.

### 3.3 Wi-Fi (off by default)

**Enabling:**

1. Set `APP_ENABLE_WIFI` to 1 in `firmware/main/app_config.h`.
2. Rebuild and flash from the `firmware/` directory (`cd firmware`, then `idf.py build flash`), then run `home`.

**Using the gripper's own hotspot:**

| Item | Value |
| --- | --- |
| Hotspot name | `GripperOne-XXXX`, where XXXX is the last 4 digits of the MAC address; check it with `wifi-info` |
| Password | `gripper123`; change it with `WIFI_AP_PASSWORD` in `app_config.h` |
| Gripper address | `192.168.4.1`, TCP port 5000 |

After the computer joins this hotspot, run:

```powershell
python host/3_gripper_cli.py --tcp 192.168.4.1 calib
```

Note: when the computer joins the gripper's hotspot, it disconnects from its previous Wi-Fi network.

**Joining an existing router (optional):**

1. Save the network over USB:

```powershell
python host/3_gripper_cli.py wifi-set NetworkName NetworkPassword
python host/3_gripper_cli.py wifi-info
```

2. After a few seconds, run `wifi-info` to see the address the router gave the gripper, e.g. 192.168.1.57.
3. Computers on the same network connect with `--tcp 192.168.1.57`.
4. The gripper's own hotspot stays available. Use `wifi-clear` to delete the saved network.

## 4. First use and every power-up

| When | Command | Time | Result |
| --- | --- | --- | --- |
| First use, or after mechanical changes | `calibrate` | About 1.5–3 min | Finds the fully open and fully closed positions and stores them in the gripper |
| After every power-up, reset or flash | `home` | About 40 s | Checks the stored positions and ends at 5 % closed (fully open) |

- While either command runs, the space between the two tactile pads must be empty.
- If `home` touches something along the way, it reports an error; remove the object and run it again. The stored calibration is not lost.
- All open/close and grip commands are available only after `calibrate` or `home` has completed.
- If `calibrate` or `home` has already completed since the motor was last powered on, `home` does not check again: it only moves the gripper to 5 % closing, and the existing calibration stays valid.

## 5. Control

### 5.1 GUI

```powershell
python host/4_gripper_gui.py
```

The GUI labels are in Chinese; English glosses are given in parentheses below.

**Connecting:**

1. In the top-right corner, choose the connection type: `USB`, `蓝牙` (Bluetooth) or `Wi-Fi`.
2. Choose the target:
   - USB: pick a serial port; USB serial ports are listed first;
   - Bluetooth: click `扫描` (Scan) and pick from the list, or just use the default name `GripperOne`;
   - Wi-Fi: enter the gripper's address.
3. Click `连接` (Connect). The connection is unlocked automatically. The first Bluetooth connection takes 5–20 s.

**Left-hand buttons:**

- `原点检查 home` (home check) and `完整标定` (full calibration);
- Open/close: closing percentage and speed, plus three shortcut buttons `张开 5 %` (open 5 %), `50 %` and `合拢 95 %` (close 95 %). The gripper follows the slider while it is dragged; a value typed into the box runs only after `移动到设定值` (Move to setpoint). A closing move stops when it touches something; use the grip command to hold an object;
- Force-controlled grip: grip force and approach speed;
- `停止` (Stop) and `清除故障` (Clear fault);
- `电机断电（Esc）` (Motor off); you can also press Esc.

**Right-hand data (pushed by the firmware: 30 times per second over Bluetooth, 50 times per second over USB and Wi-Fi; older firmware without the push is polled instead, at most 20 times per second. The first log line after connecting states which applies):**

- Status and latest fault, position and closing percentage, target force;
- Force on the two pads, ring and little;
- Torque and speed, motor temperature, motor enable and fault code, calibrated positions;
- Grip-force and closing curves for the last 20 s;
- Command results and the firmware log (the log is shown only over USB).

**Exporting data:** click `导出数据 CSV` (Export data CSV) to save the data recorded since connecting to `host/logs/gui_<date>_<time>.csv`.

### 5.2 Command line

The commands below work over every connection type; prefix them with `--port`, `--ble` or `--tcp` depending on the connection.

| Command | Action | Parameters |
| --- | --- | --- |
| `close <5–95>` | Move to the given closing percentage: 5 = fully open, 95 = fully closed | `--speed v`: 0.05–15 rad/s, default 2 |
| `grip <force>` | Force-controlled grip; holds once either pad reaches the set force | 0.5–20 N, 0 = default 3 N; `--speed v`: approach speed, default 2 |
| `stop` | Stop at the current position | — |
| `disable` | Cut motor power immediately | — |
| `clear` | Clear a latched fault | — |
| `calib` | Show status, calibration data and current position | — |
| `calibrate` / `home` | See Section 4 | — |
| `cycle` | Back-and-forth test | `--count N`, `--speed v`, `--no-home` |
| `watch` | Live position and forces on both pads; motor stays unpowered | — |
| `log` | Show the firmware log (USB only) | — |
| `wifi-info` / `wifi-set` / `wifi-clear` | Wi-Fi status and network settings | See Section 3.3 |
| `ble-scan` | Find nearby grippers | — |

**Examples:**

```powershell
python host/3_gripper_cli.py --ble close 5                # open fully
python host/3_gripper_cli.py --ble grip 3                 # grip at 3 N
python host/3_gripper_cli.py --ble grip 5 --speed 1       # 5 N, approach speed 1 rad/s
python host/3_gripper_cli.py --ble close 50 --speed 15    # move to 50 % at top speed
```

**How a grip works:**

1. If the gripper is more closed than 80 % open, it first opens to 80 %.
2. It closes at the approach speed; on contact with the object it switches to force control and holds the set force.
3. If it reaches 95 % closed without touching anything, it reports "no object detected" and returns to 80 % open.
4. If the object is removed after being gripped, it likewise returns to 80 % open.
5. Sending `grip` again while already holding an object only changes the grip force; the gripper does not reopen.

## 6. Safety

- **You can always stop:** over any connection, `disable` (motor off) and `stop` do not require unlocking. Pressing Ctrl+C while a command is running cuts motor power immediately.
- **When a connection drops:**
  - while moving: the gripper stops where it is;
  - while gripping: it keeps holding the current force so the object is not dropped;
  - that connection's unlock is lost and must be repeated after reconnecting. The command-line tool unlocks automatically.
- **Unlocking:**
  - every command that moves the motor requires unlocking first, and each connection is unlocked separately;
  - the unlock code `0x47521000` is in the public source code; it only prevents accidental operation, not deliberate misuse by others;
  - Bluetooth has no pairing or encryption, so any nearby computer with this tool can connect. Do not leave the gripper connectable while unattended.
- **Force protection:**
  - if either pad reads ≥ 23 N twice in a row, the gripper backs off and stops;
  - if it reads ≥ 24 N twice in a row, motor power is cut and `clear` is required.
- **Do not press on** the gripper while it is moving, and do not put your fingers between the two pads.

## 7. Writing your own host software

All three connection types carry the same data frames. For the format and all command codes, see [PROTOCOL.en.md](PROTOCOL.en.md).

| Connection | Sending commands | Receiving replies |
| --- | --- | --- |
| USB | Write to the serial port at 115200 baud (USB virtual serial port; not actually limited by this value) | Read from the serial port; data frames are mixed with log text, so separate them by the `A5 5A` header and CRC |
| Bluetooth | Write to characteristic `67700002-0000-0000-0000-676700001111` (write and write-without-response) | Subscribe to notifications on characteristic `67700003-0000-0000-0000-676700001111`; one frame may span several notifications, so reassemble them as a byte stream |
| Wi-Fi | Open a TCP connection to port 5000 and write directly | Read from the same connection; only one client is accepted at a time, and a new connection replaces the old one |

The Bluetooth service UUID is `67700001-0000-0000-0000-676700001111`. Every connection must first send the unlock command (opcode 0x07, argument `0x47521000`).

## 8. Troubleshooting

| Symptom | Possible cause | Fix |
| --- | --- | --- |
| Command returns `ERR_STATE` and the log shows `motor fault latched` | The motor driver reported a fault | Run `clear`, then `home` |
| Command returns `ERR_LOCKED` | This connection has not been unlocked | The command-line tool unlocks automatically; your own program must send the unlock command first |
| `calib` shows `calibrated : no` | Positions have not been checked since power-up or reset | Run `home` |
| `home` reports `position frame changed` | Something is between the pads, or the mechanics changed | Remove the object and run `home` again; if it still fails, run `calibrate` |
| `position` stays at 0, the log repeatedly shows `probe … no feedback`, or the GUI `电机` (Motor) field shows `无反馈` (no feedback) | The motor driver is stuck in an abnormal state (2026-10-23 bench test: about 2000 CAN format errors per second, recovered after a power cycle); or the motor has no power or the CAN cable is loose | First switch off the motor's 24 V for about 10 s and power it back on; if that does not help, check the CAN cable and the transceiver supply |
| `ble-scan` does not find the gripper | The gripper is not powered, the computer's Bluetooth is off, or another computer is connected | Disconnect the other computer and try again |
| Bluetooth command reports `cannot connect` | Too far away or the signal is blocked | Move within 5 m and try again |
| `grip contact … peak` is large during a grip | Approach speed too high | Lower `--speed`; for hard objects, 2 rad/s or less is recommended |
| Serial port cannot be opened | The port is in use by `idf.py monitor` or another program | Close the program using the port |
