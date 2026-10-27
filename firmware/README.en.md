[中文](README.md) | **English**

# Firmware (ESP-IDF, native)

Native **ESP-IDF** firmware for the Seeed Studio **XIAO ESP32-C6**. This is *not*
an Arduino project. It is a modular scaffold: drivers live in `components/`, the
entry point in `main/`, and a supervisor state machine ties them together over
FreeRTOS queues.

## Project layout

```
firmware/
├─ CMakeLists.txt            # top-level ESP-IDF project (GripperOne)
├─ sdkconfig.defaults        # target esp32c6, NimBLE, USB-JTAG console
├─ main/
│  ├─ app_main.c             # init + task startup (wiring only)
│  ├─ app_pins.h             # ALL GPIO assignments
│  └─ app_config.h           # baud rates, ports, feature flags, task sizing
└─ components/
   ├─ app_protocol/          # shared types + queue "bus" + user protocol codec
   │                         #   (app_proto.c: framing/CRC/opcodes for BLE+WiFi)
   ├─ board_xiao_esp32c6/    # board bring-up (NVS, banner)
   ├─ haptic_uart/           # UART1 921600, integration-board register polling
   ├─ imu_uart/              # UART0 boot 9600 -> run 115200, WIT 11-byte parser
   ├─ motor_twai/            # TWAI (CAN) DM-J4310, force-position mode only, start-up probe
   ├─ transport_uart/        # app_proto over the native USB Serial/JTAG (always on)
   ├─ wireless_ble/          # app_proto over BLE GATT (APP_ENABLE_BLE, on)
   ├─ wireless_wifi/         # app_proto over TCP: own access point + optional station (APP_ENABLE_WIFI, off)
   ├─ gripper/               # boot calibration (current + tactile) and tactile force grip
   └─ app_supervisor/        # INIT/IDLE/ACTIVE/ERROR/SAFE_STOP + gripper tick
```

The host-side command protocol (framing, CRC, opcodes) is described in
[../docs/PROTOCOL.en.md](../docs/PROTOCOL.en.md).

## Pins and wiring

All GPIO assignments are in `main/app_pins.h`. Pin tables, wiring, and the
external CAN transceiver that the TWAI bus needs are documented in
[../hardware/WIRING.en.md](../hardware/WIRING.en.md).

Controller assignment in firmware: haptic link → `UART_NUM_1`, IMU link →
`UART_NUM_0` (the C6 has two HP UART controllers; the console is on USB
Serial/JTAG so both are free). Change in `main/app_config.h`.

If the haptic link ever goes quiet, the driver's start-up discovery falls back
to a sweep of both pin orientations × six baud rates × both candidate dialects
and hex-dumps whatever replies; this is how the current pin mapping was
established.

## Build

```bash
# One-time, with ESP-IDF exported into the shell:
cd firmware
idf.py set-target esp32c6
idf.py build
```

## Flash & monitor

The XIAO ESP32-C6 exposes USB Serial/JTAG on its USB-C port, so no external
programmer is needed.

```bash
cd firmware
idf.py -p <PORT> flash monitor      # e.g. -p COM7 (Windows) or -p /dev/ttyACM0
# Ctrl-] to exit the monitor
```

If the board is not auto-detected, put it in download mode (hold **BOOT**, tap
**RESET**, release **BOOT**) and retry.

## Tactile sensor GUI (host tool)

`host/2_haptic_uart.py` (at the repository root) is the PySide6
minimum-verification GUI for two PaXini CP-M3025 modules through the ESP32: left
is the ring finger, right is the little finger. It displays both 7×11 Fz maps,
per-point Fx/Fy shear arrows, resultant force curves, actual per-module rates
and sequence loss. Install `host/requirements.txt`, flash the firmware, then run
`python host/2_haptic_uart.py` from the repository root.

## Configuration

- **Pins** — edit `main/app_pins.h`.
- **Baud rates / ports / task sizes** — edit `main/app_config.h`.
- **Transports** — USB is always on; `APP_ENABLE_BLE` (1) and `APP_ENABLE_WIFI` (0) in
  `main/app_config.h`. All run side by side; a reply goes to the link the command came from
  and each link unlocks its own session. The partition table is the large single-app one
  (1.5 MB app) because BLE + Wi-Fi need more than 1 MB.
- **Wi-Fi** — when enabled, the board runs its own access point
  (`WIFI_AP_SSID_PREFIX`-XXXX, password `WIFI_AP_PASSWORD`, channel `WIFI_AP_CHANNEL`;
  connect to 192.168.4.1) and serves app_proto on TCP port `WIFI_TCP_PORT` (5000). It
  can also join a network stored in NVS as a station, retrying every 5 s. All of these
  are in `main/app_config.h`.
- Deeper options via `idf.py menuconfig`.

## FreeRTOS tasks

| Task              | Component        | Role                                            |
| ----------------- | ---------------- | ----------------------------------------------- |
| `haptic_uart`     | haptic_uart      | ingest high-rate pressure data → `haptic_q`     |
| `imu_uart`        | imu_uart         | ingest IMU packets → `imu_q`                    |
| `motor_twai`      | motor_twai       | TX motor command / RX status → `motor_status_q` |
| `usj_rx`          | transport_uart   | USB Serial/JTAG RX; parses framed protocol → `ctrl_q` |
| `wifi_srv`        | wireless_wifi    | TCP server; parses framed protocol → `ctrl_q` (only with `APP_ENABLE_WIFI`) |
| `supervisor`      | app_supervisor   | state machine + safety, drains all queues       |

BLE runs on the NimBLE host task (`nimble_port_freertos_init`).

## Protocols implemented

The real wire formats were decoded from the vendor documents in
[../hardware/datasheets/](../hardware/datasheets/) and are now implemented
(not stubs):

- **Haptic (PaXini PX-6AX GEN3)** — the ESP talks to the **高速通信集成板**
  (high-speed communication integration board) at 921600 8N1, not to a sensor
  module's own pins, so it speaks the board's register protocol
  (`通信盒通信协议` §4.4), not the sensor-direct `0x7B`/`0x79` protocol in §5.4
  of the sensor manual:

  | | frame |
  |---|---|
  | request | `55 AA \| 00 \| func \| reg_addr(2,LE) \| num(2,LE) \| data[N] \| LRC` |
  | response | `AA 55 \| 00 \| func \| reg_addr(2,LE) \| num(2,LE) \| data[M] \| LRC` |
  | auto-push | `AA 56 \| 00 \| frame_len(2,LE) \| total_err(1) \| data[M] \| LRC` |

  `func` is `0x03` read (max 512 B), `0x10` write (max 10 B), `0x17` tare; LRC
  is the two's complement of the sum of every preceding byte. A module is a slot
  in the board's fixed 28-module order (§4.6) — thumb, index, middle, ring,
  little, palm, each finger as proximal/middle/fingertip/nail — so a CP-M3025
  is its finger's *proximal* module: ring = 12, little = 16.

  | | Ring proximal (12) | Little proximal (16) |
  |---|---|---|
  | Distributed force 77×3 = 231 B | `0x2800` | `0x3000` |
  | Resultant 3×int16 | `0x0548` | `0x0560` |
  | Point count uint16 | `0x0048` | `0x0050` |
  | Connection bit | `0x0011` bit4 | `0x0012` bit0 |

  At start-up the driver disables auto-push, reads the version string, the
  module map at `0x0010`–`0x0013` and each slot's point count, and logs a
  specific error if a slot is empty or is not a 77-point module. It then polls
  both modules at 80 Hz with two reads each. Distributed points are
  `Fx:i8, Fy:i8, Fz:u8`; the 6-byte resultant is three int16 of which only the
  low byte carries the value (matching the vendor's own `Hand_UI` parser). All
  values are 0.1 N/LSB. `host/2_haptic_uart.py --selftest` checks these frames
  against the vendor's documented example commands.
- **IMU (WitMotion JY901P, WIT standard protocol)** — boot at 9600 8N1, then
  switch the powered session to 115200 after identification. Fixed 11-byte
  frames (`0x55` + type + 4×int16-LE + 8-bit sum checksum). The sensor runs at
  100 Hz. Every 20 ms the supervisor batches all accumulated samples into one
  USB telemetry frame, normally producing 50 packets/s containing 100 samples/s.
- **Motor (Damiao DM-J4310-2EC)** — TWAI CAN 2.0A @ 1 Mbps, force-position
  mode only (`0x300+ID`: p_des float, v_des ×100, i_des ×10000). A start-up probe
  reads PMAX/VMAX/TMAX, sets CTRL_MODE = 4 and a 300 ms comms-loss TIMEOUT, and
  verifies both by reading back. Feedback decoding uses the ranges read from
  the motor. While disabled, a disable frame every 50 ms keeps feedback alive.

## Current limitations

- **Haptic**: assumes the two CP-M3025 are in the integration board's **无名指**
  (ring) and **小拇指** (little finger) connectors as proximal modules (slots 12
  and 16). Plugged into a different finger connector, the board reports them at
  different slots and the start-up discovery log says so. Only request/response
  polling is implemented; the board's auto-push mode (`0x0017`) is disabled at
  boot and would give a higher frame rate. The 77 values are displayed
  row-major; confirm physical rotation/mirroring by pressing one corner before
  using the map for control.
- **IMU**: startup checks only 9600 and 115200, using a read-only register query
  to identify/wake the unit. A unit found at 9600 is moved to 115200 for the
  powered session (not saved), then ACC/GYRO/ANGLE output at 100 Hz is applied
  without saving. A stopped stream is woken once and then re-detected at those
  same two rates. Yaw is intentionally not exposed.
- **Motor/gripper**: verified on the bench on 2026-10-23 (calibration, home check,
  moves up to 15 rad/s, force grip on a sponge). Open items are listed in
  [../docs/GRIPPER_CONTROL.md](../docs/GRIPPER_CONTROL.md) section 7. Uses
  factory IDs ESC_ID `0x001`, MST_ID `0x000`.
- **Wi-Fi** is implemented (access point + optional station, TCP server) but
  off by default (`APP_ENABLE_WIFI 0`). **BLE** has no pairing/bonding/security.

Remaining gaps are marked with `TODO:` comments in the source.

## Next implementation steps

1. Haptic: verify address 1/2 and point ordering on hardware, then lock the
   physical rotation/mirroring constants for the two mounted fingers.
2. IMU: send the WitMotion config sequence at startup so the unit emits the
   desired packets/rate regardless of prior state.
3. Motor: first powered run with the jaw empty; check the probe log, the
   calibration log (measured speed, stall torque, k) and `CLOSE_DIR`.
4. Flesh out the supervisor safety policy (force limits, orientation limits,
   fault recovery out of `ERROR`/`SAFE_STOP`).
5. Add BLE security (bonding).

## Build verification status

> ✅ **Verified.** Built successfully with **ESP-IDF v6.0** for `esp32c6`
> (`idf.py set-target esp32c6 && idf.py build`) — `GripperOne.bin` links with
> ~27% of the app partition free. Reproduce locally with the build commands
> above after exporting ESP-IDF.
