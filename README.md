# gripper_plus

这是一个基于开源夹爪工程进一步改造的项目，结合了压力传感器和完整无线控制电路，目标是在原有夹爪基础上提升感知能力与控制便利性。

## 项目来源

- 原始开源夹爪工程：<https://github.com/Ragtime-LAB/RhinoV2.0_Gripper>
- 电机/执行器型号：达妙科技 DM-J4310-2EC v1.2

## 项目特点

- 基于开源夹爪工程进行二次开发
- 集成压力传感器
- 集成完整无线控制电路

## XIAO ESP32-C6 与各子模块连接说明

本项目以 Seeed Studio XIAO ESP32-C6 作为主控板，负责 USB 调试/下载、触觉信息高速通信、IMU 通信、OLED 显示以及电机控制等功能。下面整理了主控与各功能子模块之间的引脚对应关系，便于后续接线、排查和维护。

| 功能 | XIAO 引脚 | GPIO | 用的外设 |
| --- | --- | --- | --- |
| USB 调试 / 下载 | USB-C | GPIO12/13 内部 USB | USB CDC/JTAG |
| 触觉信息高速通信板子（UART0） | D6 TX / D7 RX | GPIO16 / GPIO17 | 普通 UART0 |
| IMU JY901P / MPU9250+（UART1，内置 MCU 算法校准） | D1 TX / D0 RX | GPIO1 / GPIO0 | 普通 UART1，经 GPIO Matrix |
| SH1106 驱动的 1.3 inch OLED 屏幕（I2C）（非必须） | D4 SDA / D5 SCL | GPIO22 / GPIO23 | I2C |
| 电机控制通信（TWAI） | D2 TX / D3 RX | GPIO2 / GPIO21 | TWAI |

## 说明

本仓库用于记录夹爪项目的基础信息、来源链接和关键采购信息，便于后续开发、维护和复用。

## 致谢

感谢芝士榴莲肥牛 <https://github.com/wEch1ng> 的支持。

---

# Firmware (ESP-IDF, native)

Native **ESP-IDF** firmware for the Seeed Studio **XIAO ESP32-C6**. This is *not*
an Arduino project. It is a modular scaffold: drivers live in `components/`, the
entry point in `main/`, and a supervisor state machine ties them together over
FreeRTOS queues.

## Project layout

```
gripper_plus/
├─ CMakeLists.txt            # top-level ESP-IDF project
├─ sdkconfig.defaults        # target esp32c6, NimBLE, USB-JTAG console
├─ main/
│  ├─ app_main.c             # init + task startup (wiring only)
│  ├─ app_pins.h             # ALL GPIO assignments
│  └─ app_config.h           # baud rates, ports, feature flags, task sizing
└─ components/
   ├─ app_protocol/          # shared types + inter-task queue "bus"
   ├─ board_xiao_esp32c6/    # board bring-up (NVS, banner)
   ├─ haptic_uart/           # UART1 921600, 0xAA56 auto-push + LRC parser
   ├─ imu_uart/              # UART0 9600, WitMotion 11-byte frame parser
   ├─ motor_twai/            # TWAI (CAN) DM-J4310 MIT-mode codec + API
   ├─ oled_sh1106/           # optional I2C OLED (APP_ENABLE_OLED)
   ├─ wireless_ble/          # NimBLE GATT control interface
   ├─ wireless_wifi/         # optional Wi-Fi control path (skeleton)
   └─ app_supervisor/        # INIT/IDLE/ACTIVE/ERROR/SAFE_STOP state machine
```

## Hardware pin table (XIAO ESP32-C6)

Verified against the project schematic (`esp32_c6_xiao` / Sub1). The schematic's
**UART1** is the haptic link and **UART2** is the IMU link — these are logical
names, not ESP32 controller numbers.

| Schematic net | Function                | XIAO pin | GPIO | ESP peripheral        |
| ------------- | ----------------------- | -------- | ---- | --------------------- |
| (USB-C)       | USB debug / download    | USB-C    | int. | USB Serial/JTAG       |
| UART2_rx      | IMU link RX (XIAO←IMU)  | D0       | 0    | UART0 @ 9600          |
| UART2_tx      | IMU link TX (XIAO→IMU)  | D1       | 1    | UART0 @ 9600          |
| TWAI TX       | Motor CAN TX            | D2       | 2    | TWAI (needs xcvr)     |
| TWAI RX       | Motor CAN RX            | D3       | 21   | TWAI (needs xcvr)     |
| SDA           | OLED I2C SDA            | D4       | 22   | I2C0 @ 400 kHz        |
| SCL           | OLED I2C SCL            | D5       | 23   | I2C0 @ 400 kHz        |
| UART1_tx      | Haptic TX (XIAO→board)  | D6       | 16   | UART1 @ 921600        |
| UART1_rx      | Haptic RX (XIAO←board)  | D7       | 17   | UART1 @ 921600        |

TX/RX are named from the XIAO's perspective (XIAO TX → peripheral RX). Power:
3V3_out (pin 11) and GND (pin 12) feed the sensor boards; a 10 µF bulk cap sits
on 3V3. VBUS and D8–D10 are unused.

Controller assignment in firmware: haptic link → `UART_NUM_1`, IMU link →
`UART_NUM_0` (the C6 has two HP UART controllers; the console is on USB
Serial/JTAG so both are free). Change in `main/app_config.h`.

## ⚠️ External CAN transceiver required

The ESP32-C6 integrates a **TWAI/CAN controller but NOT a CAN transceiver**.
GPIO2 (TX) and GPIO21 (RX) are logic-level pins and must go through an external
**3.3 V CAN transceiver** (e.g. **TCAN332**, **SN65HVD230**) to reach the
differential CAN_H / CAN_L bus. Wiring:

```
ESP32-C6 GPIO2 (TWAI TX) ──► transceiver TXD
ESP32-C6 GPIO21 (TWAI RX) ◄── transceiver RXD
                              transceiver ──► CAN_H / CAN_L (120 Ω terminated)
```

## Build

```bash
# One-time, from the project root, with ESP-IDF exported into the shell:
idf.py set-target esp32c6
idf.py build
```

## Flash & monitor

The XIAO ESP32-C6 exposes USB Serial/JTAG on its USB-C port, so no external
programmer is needed.

```bash
idf.py -p <PORT> flash monitor      # e.g. -p COM7 (Windows) or -p /dev/ttyACM0
# Ctrl-] to exit the monitor
```

If the board is not auto-detected, put it in download mode (hold **BOOT**, tap
**RESET**, release **BOOT**) and retry.

## Configuration

- **Pins** — edit `main/app_pins.h`.
- **Baud rates / ports / task sizes** — edit `main/app_config.h`.
- **Feature flags** — in `main/app_config.h`:
  - `APP_ENABLE_OLED` (default 1) — SH1106 display + `oled_task`
  - `APP_ENABLE_BLE`  (default 1) — NimBLE GATT control
  - `APP_ENABLE_WIFI` (default 0) — Wi-Fi control skeleton
- Deeper options via `idf.py menuconfig`.

## FreeRTOS tasks

| Task              | Component        | Role                                            |
| ----------------- | ---------------- | ----------------------------------------------- |
| `haptic_uart`     | haptic_uart      | ingest high-rate pressure data → `haptic_q`     |
| `imu_uart`        | imu_uart         | ingest IMU packets → `imu_q`                    |
| `motor_twai`      | motor_twai       | TX motor command / RX status → `motor_status_q` |
| `ble_control`     | wireless_ble     | NimBLE host; GATT writes → `ctrl_q`             |
| `oled` (optional) | oled_sh1106      | render system/sensor status                     |
| `supervisor`      | app_supervisor   | state machine + safety, drains all queues       |

### BLE control characteristic

Write payload (little-endian) to the control characteristic:

| byte 0 (command id)        | bytes 1..4        |
| -------------------------- | ----------------- |
| 1 = enable motor           | –                 |
| 2 = disable motor          | –                 |
| 3 = set target             | float32 target    |
| 4 = request status         | –                 |

The status characteristic (read / notify) returns the packed `motor_status_t`.

## Protocols implemented

The real wire formats were decoded from `source_files/` and are now implemented
(not stubs):

- **Haptic (PaXini 通信盒, `通信盒通信协议_V1.0.5`)** — 921600 8N1. The driver
  writes register `0x0016 = 0x01` (resultant-force content) and `0x0017 = 0x01`
  (enable auto-push), then runs a streaming parser that syncs on the `0xAA 0x56`
  auto-push header, validates the LRC (`~sum + 1`), reads the little-endian
  frame-length field, and decodes each connected module's resultant force
  (Fx/Fy/Fz as int16-LE × 0.1 N) into `haptic_sample_t`.
- **IMU (WitMotion JY901P, WIT standard protocol)** — 9600 8N1. Fixed 11-byte
  frames (`0x55` + type + 4×int16-LE + 8-bit sum checksum). Decodes `0x53` angle
  (roll/pitch/yaw = int16/32768×180°) and `0x59` quaternion (int16/32768).
- **Motor (Damiao DM-J4310-2EC)** — TWAI CAN 2.0A @ 1 Mbps. Enable/disable
  frames (`…FC` / `…FD`), MIT-mode command packing (position/velocity/Kp/Kd/
  torque via the standard `float_to_uint` map over ±12.5 rad / ±30 rad·s⁻¹ /
  ±10 N·m / Kp 0–500 / Kd 0–5), and feedback decoding (position/velocity/torque
  + MOS/rotor temperature + error nibble).

## Current limitations

- **Haptic**: runs in resultant-only mode; the distributed-force payload
  (register `0x0016` bit1) and strict per-module slicing by the connection
  bitmap (`0x0010`–`0x0013`) are not decoded — modules are treated as a flat
  array of 6-byte resultant-force records.
- **IMU**: assumes the unit already outputs angle/quaternion. The one-time
  configuration (unlock `0xFF 0xAA 0x69 0x88 0xB5`, set `RSW` reg `0x02`, save)
  is documented as a `TODO` but not sent automatically. Accel/gyro/mag frames
  are parsed-and-ignored (only orientation is propagated).
- **Motor**: uses factory default IDs (ESC_ID `0x001`, MST_ID `0x000`) and the
  4310's default value ranges — confirm/adjust against your motor's registers.
  A fixed impedance gain (Kp=5, Kd=1) holds position; no trajectory planning.
- **OLED**: initialises the SH1106 and pushes a framebuffer, but has no font, so
  it shows a heartbeat bar and logs the real values.
- **Wi-Fi** is a logged no-op skeleton; **BLE** has no pairing/bonding/security.

Remaining gaps are marked with `TODO:` comments in the source.

## Next implementation steps

1. Haptic: read the connection bitmaps + point-count registers at init and slice
   the payload per-module; optionally add distributed-force decoding.
2. IMU: send the WitMotion config sequence at startup so the unit emits the
   desired packets/rate regardless of prior state.
3. Motor: confirm ESC_ID/MST_ID and PMAX/VMAX/TMAX, then add closed-loop grip
   control (target force/position from the supervisor).
4. Add a 5×7 font + text renderer to `oled_sh1106`.
5. Flesh out the supervisor safety policy (force limits, orientation limits,
   fault recovery out of `ERROR`/`SAFE_STOP`).
6. Add BLE security (bonding) and, if needed, the Wi-Fi control server.

## Build verification status

> ✅ **Verified.** Built successfully with **ESP-IDF v6.0** for `esp32c6`
> (`idf.py set-target esp32c6 && idf.py build`) — `gripper_plus.bin` links with
> ~27% of the app partition free. Reproduce locally with the build commands
> above after exporting ESP-IDF.
