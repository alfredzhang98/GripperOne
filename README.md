# gripper_plus

这是一个基于开源夹爪工程进一步改造的项目，结合了压力传感器和完整无线控制电路，目标是在原有夹爪基础上提升感知能力与控制便利性。

夹爪电机控制的用法、实测结果、参数范围和电流计算见 [README_gripper.md](README_gripper.md)。

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
| 触觉 高速通信集成板 | D6 TX / D7 RX | GPIO16 / GPIO17 | UART1 @ 921600 |
| IMU JY901P（内置 MCU 算法校准） | D1 TX / D0 RX | GPIO1 / GPIO0 | UART0，经 GPIO Matrix |
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
   ├─ app_protocol/          # shared types + queue "bus" + user protocol codec
   │                         #   (app_proto.c: framing/CRC/opcodes for BLE+WiFi)
   ├─ board_xiao_esp32c6/    # board bring-up (NVS, banner)
   ├─ haptic_uart/           # UART1 921600, integration-board register polling
   ├─ imu_uart/              # UART0 boot 9600 -> run 115200, WIT 11-byte parser
   ├─ motor_twai/            # TWAI (CAN) DM-J4310, force-position mode only, start-up probe
   ├─ transport_uart/        # app_proto over the native USB Serial/JTAG (default transport)
   ├─ wireless_ble/          # NimBLE GATT control interface (not started by default)
   ├─ wireless_wifi/         # optional Wi-Fi control path (skeleton)
   ├─ gripper/               # boot calibration (current + tactile) and tactile force grip
   └─ app_supervisor/        # INIT/IDLE/ACTIVE/ERROR/SAFE_STOP + gripper tick
```

## Hardware pin table (XIAO ESP32-C6)

Verified against the project schematic (`esp32_c6_xiao` / Sub1). The schematic's
**UART1** is the haptic link and **UART2** is the IMU link — these are logical
names, not ESP32 controller numbers.

| Schematic net | Function                | XIAO pin | GPIO | ESP peripheral        |
| ------------- | ----------------------- | -------- | ---- | --------------------- |
| (USB-C)       | USB debug / download    | USB-C    | int. | USB Serial/JTAG       |
| UART2_rx      | IMU link RX (XIAO←IMU)  | D0       | 0    | UART0, 9600→115200    |
| UART2_tx      | IMU link TX (XIAO→IMU)  | D1       | 1    | UART0, 9600→115200    |
| TWAI TX       | Motor CAN TX            | D2       | 2    | TWAI (needs xcvr)     |
| TWAI RX       | Motor CAN RX            | D3       | 21   | TWAI (needs xcvr)     |
| SDA           | OLED I2C SDA            | D4       | 22   | I2C0 @ 400 kHz        |
| SCL           | OLED I2C SCL            | D5       | 23   | I2C0 @ 400 kHz        |
| UART1_rx      | Haptic RX (XIAO←board)  | D6       | 16   | UART1 @ 921600        |
| UART1_tx      | Haptic TX (XIAO→board)  | D7       | 17   | UART1 @ 921600        |

TX/RX are named from the XIAO's perspective (XIAO TX → peripheral RX). Power:
3V3_out (pin 11) and GND (pin 12) feed the IMU; a 10 µF bulk cap sits on 3V3.
**The haptic integration board must not be run from 3V3** — see the wiring note
below. D8–D10 are unused.

Haptic wiring: D6/D7 go to the integration board's 4-pin UART header
(connector AFC12-S04DCA-00; one row of four, `pin1 TX, pin2 RX, pin3 GND,
pin4 VCC` — the silkscreen staggers the labels onto two lines but the pins are
in a single row). **The haptic TX/RX assignment is the opposite way round from
the XIAO silkscreen**: the header names the board's own pin functions and this
harness joins them label to label, so D6 sits on the board's transmitter and
D7 on its receiver. The firmware therefore drives GPIO17 as TX and GPIO16 as
RX (see `main/app_pins.h`). Verified on hardware — with this mapping a version
read returns `ADAPTER_FW02.26`; with the pair swapped the board is silent at
every baud rate. Tie the grounds. The board is full duplex and takes its own
4.5–5 V supply on the WAFER-HY2.0-2PWB connector; it is not powered from the
XIAO's 3V3. The two sensors plug into the board's **无名指** and **小拇指**
connectors, and the board handles the SPI side of each module, so no CS or
pull-up wiring is needed on the ESP.

> ### ⚠️ The integration board needs 5 V, and 3V3 fails in a way that looks like a broken sensor
>
> §4.3 specifies **4.5–5 V** on the WAFER-HY2.0-2PWB connector. Run it from the
> XIAO's 3V3 rail instead and the board still boots, still answers on UART, still
> returns its version string, and still enumerates **one** sensor perfectly — it
> just silently fails to find the second one. Use VBUS/5 V, never 3V3.
>
> What this looks like when you get it wrong, and why none of it points at the
> supply:
>
> - `0x0010`–`0x0013` comes back `00 10 00 00`: only the ring module is mapped.
> - The missing slot's point count (`0x0050`) reads 0 and its per-module error
>   code (`0x0700 + n×2`) reads `0x0001` — the same code every genuinely empty
>   slot reports. The board is not reporting a fault; it simply never saw it.
> - Reads of the missing module's data area still succeed, returning 231 zero
>   bytes, so the link looks perfectly healthy.
> - The **later** module in the board's fixed scan order is the one that drops
>   (ring is slot 12, little is slot 16), which is the tell: the first sensor
>   brings the rail down far enough that the second never enumerates.
> - Over USB the same board finds both, because the PC supplies a proper 5 V.
>   That difference between the two links is the supply, not the protocol.
>
> The map is latched **once, at the board's own power-up** (§4.6), so fixing the
> supply is not enough on its own — the board has to lose power completely and
> come back up on 5 V before it will rescan. Adding a second supply while it is
> still running changes nothing.

If the link ever goes quiet, the driver's start-up discovery falls back to a
sweep of both pin orientations x six baud rates x both candidate dialects and
hex-dumps whatever replies, which is how the mapping above was established.

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

## Tactile sensor GUI (host tool)

`host/2_haptic_uart.py` is the PySide6 minimum-verification GUI for two PaXini
CP-M3025 modules through the ESP32: left is the ring finger, right is the little
finger. It displays both 7×11 Fz maps, per-point Fx/Fy shear arrows, resultant
force curves, actual per-module rates and sequence loss. Install
`host/requirements.txt`, flash the firmware, then run `python host/2_haptic_uart.py`.

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
- **Transport** — `APP_TRANSPORT` in `main/app_config.h` (default `TRANSPORT_UART`:
  app_proto over the USB-C port; BLE and Wi-Fi are not started).
- Deeper options via `idf.py menuconfig`.

## FreeRTOS tasks

| Task              | Component        | Role                                            |
| ----------------- | ---------------- | ----------------------------------------------- |
| `haptic_uart`     | haptic_uart      | ingest high-rate pressure data → `haptic_q`     |
| `imu_uart`        | imu_uart         | ingest IMU packets → `imu_q`                    |
| `motor_twai`      | motor_twai       | TX motor command / RX status → `motor_status_q` |
| `usj_rx`          | transport_uart   | USB Serial/JTAG RX; parses framed protocol → `ctrl_q` |
| `supervisor`      | app_supervisor   | state machine + safety, drains all queues       |

控制不再是"明文单字节"，而是走下面统一的 **用户通信协议**（BLE 与 Wi-Fi 共用）。

---

# 用户通信协议（User Protocol）— 中文说明

上位机（手机 / PC）通过 **BLE 或 Wi-Fi** 控制夹爪时，**不直接发裸字节**，而是发送带帧头、版本号、CRC 校验的结构化数据包。BLE 和 Wi-Fi 走**同一套解析与分发**（`components/app_protocol/app_proto.c`），命令进入 `supervisor`（监督任务）统一执行，并**自动启用对应子系统**。这样做的好处：

1. **不怕明文/误触**：随机字节过不了帧头 + CRC 校验，不会误让电机动作。
2. **电机动作需先解锁**：危险指令（电机使能/目标/置零、整机自校准）必须先发会话解锁指令，否则被拒绝。
3. **BLE / Wi-Fi 统一**：两种无线通道用完全相同的指令集和数据格式，换通道不用改上位机逻辑。

## 一、帧格式（所有多字节字段均为小端）

```
+------+------+-----+------+-----+---------+--------------+----------+
| A5   | 5A   | VER | TYPE | SEQ | LEN(2)  | PAYLOAD[LEN] | CRC16(2) |
+------+------+-----+------+-----+---------+--------------+----------+
  帧头0  帧头1  版本  类型   序号  负载长度     负载数据        校验
```

| 字段 | 字节 | 说明 |
| --- | --- | --- |
| 帧头 SOF | `A5 5A` | 固定，用于同步；解析器靠它找帧起点 |
| 版本 VER | `01` | 协议版本，不匹配则拒收 |
| 类型 TYPE | 1 | 见下表（命令/响应/遥测/应答/错误） |
| 序号 SEQ | 1 | 请求序号；设备在响应/应答里原样带回，便于上位机配对 |
| 长度 LEN | 2 | 负载字节数（小端）；`PAYLOAD[0]` 是操作码 |
| 负载 PAYLOAD | LEN | 第 1 字节为操作码，其后为参数 |
| 校验 CRC16 | 2 | 对 `VER..PAYLOAD` 做 CRC-16/CCITT-FALSE（poly `0x1021`，初值 `0xFFFF`），小端存放 |

### 消息类型 TYPE

| 值 | 名称 | 方向 | 含义 |
| --- | --- | --- | --- |
| `0x00` | COMMAND | 上位机 → 设备 | 控制命令 |
| `0x01` | RESPONSE | 设备 → 上位机 | 对查询类命令的回复（版本、状态） |
| `0x02` | TELEMETRY | 设备 → 上位机 | 主动上报的遥测流（IMU / 触觉 / 电机） |
| `0x03` | ACK | 设备 → 上位机 | 命令是否被接受（含状态码） |
| `0x04` | ERROR | 设备 → 上位机 | 帧非法/被拒（CRC 错、版本错、未解锁等） |

## 二、操作码表（PAYLOAD[0]）

> "需解锁" = 必须先成功发送会话解锁（`0x07`）后才会执行，否则回 ACK 错误码 `0x04(LOCKED)`。

### 系统类 `0x0-`

| 操作码 | 名称 | 参数 | 需解锁 | 说明 |
| --- | --- | --- | --- | --- |
| `0x01` | PING | 无 | 否 | 心跳/连通性测试，回 ACK OK |
| `0x02` | GET_VERSION | 无 | 否 | 查询协议版本+固件版本，回 RESPONSE |
| `0x03` | GET_STATE | 无 | 否 | 查询状态机状态，回 RESPONSE |
| `0x04` | SELF_CALIBRATE | 无 | **是** | 等同 `GRIP_CALIBRATE`（夹爪标定，含触觉清零）。IMU 校准需静止水平，单独用 `0x23` |
| `0x05` | SET_TELEM_RATE | u16 Hz | 否 | 设置遥测上报频率（默认 20Hz；受 50Hz 控制循环限制，超出按 50Hz 钳制） |
| `0x06` | SAFE_STOP | 无 | 否（安全指令，永远允许） | 立即断电机并进入 SAFE_STOP |
| `0x07` | SESSION_UNLOCK | u32 密钥 | 否 | 会话解锁；密钥 = `0x47521000`，正确才解锁 |
| `0x08` | SESSION_LOCK | 无 | 否 | 重新上锁 |

### 电机类 `0x1-`

电机只由夹爪层（`components/gripper/`）以力位混控模式驱动。直接驱动电机的旧指令已停用，回 `ERR_OPCODE`：`0x10` 使能会绕过夹爪的电流和力限制；`0x12`/`0x13` 是 MIT 模式指令，与力位混控模式的帧格式不同；`0x14` 保存零点会移动电机的位置坐标系，使标定结果失效。

| 操作码 | 名称 | 参数 | 需解锁 | 说明 |
| --- | --- | --- | --- | --- |
| `0x10` | MOTOR_ENABLE | — | — | 已停用 |
| `0x11` | MOTOR_DISABLE | 无 | 否 | 立即失能电机，夹爪停止动作 |
| `0x12` | MOTOR_SET_TARGET | — | — | 已停用 |
| `0x13` | MOTOR_SET_GAINS | — | — | 已停用 |
| `0x14` | MOTOR_SAVE_ZERO | — | — | 已停用 |
| `0x15` | MOTOR_CLEAR_ERR | 无 | 是 | 清除电机故障，并解除 SAFE_STOP 锁定 |
| `0x16` | MOTOR_GET_STATUS | 无 | 否 | 立刻回一帧电机遥测 |
| `0x17` | MOTOR_SAVE_CONFIG | 无 | 是 | 把控制模式 4 和通讯超时写入电机自身 flash（发送 0xAA 存储命令，电机须失能）。电机 flash 约可擦写 1 万次，只在新电机或电机参数被改过时执行一次 |

### IMU 类 `0x2-`

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x20` | IMU_STREAM_EN | 无 | 开启 IMU → USB/上位机遥测流（上电默认关闭，必须显式开启） |
| `0x21` | IMU_STREAM_DIS | 无 | 关闭 USB/上位机遥测流；IMU → ESP32 仍以配置频率采样 |
| `0x22` | IMU_GET | 无 | 立刻返回最近一帧完整 IMU 遥测 |
| `0x23` | IMU_CALIBRATE | 无 | 加速度计校准，模块需水平静止 |
| `0x25` | IMU_SET_REF | 无 | 将当前 Roll/Pitch 保存为参考零点 |
| `0x27` | IMU_POWON_SET | `u0=0/1` | 设置 IMU 上电自动输出；首次启动默认 `0`，用户设置保存在 ESP NVS |
| `0x28` | IMU_SET_RATE | `u0=Hz` | 设置本次会话的 IMU 输出率：10/20/50/100/200 Hz |
| `0x29` | IMU_ACQ_EN | 无 | 恢复 IMU → ESP32 数据帧，使用当前设置的输出率 |
| `0x2A` | IMU_ACQ_DIS | 无 | 停止 IMU → ESP32 数据帧；仅当前会话，不写 IMU Flash |

### 触觉类 `0x3-`

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x30` | HAPTIC_STREAM_EN | 无 | 开启 ESP32→USB 触觉遥测；要求两指都有新鲜数据 |
| `0x31` | HAPTIC_STREAM_DIS | 无 | 关闭 USB 触觉遥测；不改变传感器轮询 |
| `0x32` | HAPTIC_GET | 无 | 返回两指各自最近的一帧完整点阵 |
| `0x34` | HAPTIC_CALIBRATE | 无 | 两个触觉模块空载清零/标定 |
| `0x35` | HAPTIC_ACQ_EN | 无 | 恢复传感器→ESP32 请求/应答轮询 |
| `0x36` | HAPTIC_ACQ_DIS | 无 | 停止轮询；不改变 USB 流开关 |

### 夹爪高级动作 `0x4-`（除 `0x46`、`0x47` 外均需解锁）

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x40` | GRIP_CALIBRATE | 无 | 完整标定：张开到机械限位（电流判断）后回退，触觉清零，合拢到两块触觉垫接触后回退；结果写入 ESP32 的 NVS，覆盖旧记录 |
| `0x41` | GRIP_HOME | 无 | 原点检查：触觉清零后慢速合拢到两垫接触，与 NVS 中保存的接触位置比较，相差 ≤ 0.03 rad（或整圈 2π 加 ≤ 0.03 rad）则平移并恢复保存的标定，停在闭合 5 %；相差更大则报错，保存的标定保留（多半是两垫之间有东西）。没有保存的标定时执行完整标定 |
| `0x42` | GRIP_SET_CLOSE | f32 [, f32] | 闭合百分比 5–95（0 = 标定的最大位置，100 = 最小位置，两端各留 5 % 余量；超出时移到较近的一端）；可选第二个数为速度（输出轴 rad/s，0.05–15，省略或 0 为 2）。张开用 5 |
| `0x43` | GRIP_GRIP | f32 [, f32] | 定力夹取，参数为目标夹持力（N），≤ 0 表示用默认力；范围 0.5–20 N。可选第二个数为接近速度（rad/s，0.05–15，省略或 0 为 2）。夹爪比 80 % 开度更合拢时先张开到 80 %，否则从当前位置直接合拢；合拢到 95 % 仍没有触觉力则报“empty grasp”（未检测到物体）并回到 80 % 开度；检测到物体后按力闭环夹持，位置随物体自适应；物体被取走同样回到 80 % 开度 |
| `0x45` | GRIP_SET_FORCE | f32 | 设置并保存默认夹持力（N），范围 0.5–20 N |
| `0x46` | GRIP_STOP | 无 | 保持当前位置（不需解锁） |
| `0x47` | GRIP_GET_CALIB | 无 | 回 RESPONSE：有效位、夹爪状态、最近故障码 + 10 个 f32（最小/最大/接触位置 rad、k N/pu、偏置电流 pu、移动电流 pu、默认力 N、当前位置 rad、开合度、目标力 N） |
| `0x48` | GRIP_MOVE_DEG | f32 角度, f32 速度 | 相对最小位置的输出轴角度（°）与速度（°/s） |

## 三、遥测显式启用

固件上电后 IMU 与触觉 USB 遥测都默认关闭，GUI 建立连接也不会自动开流。只有显式发送对应的 `STREAM_EN` 才会向 USB/上位机连续上报；读取、设置和校准命令不会隐式开启连续遥测。

USB 上报与 IMU → ESP32 采集是两个独立开关：关闭其中一个不会自动修改另一个。开启 USB 上报时会检查最近 500 ms 内是否收到过有效 IMU 样本；没有新鲜数据时返回 `ERR_STATE`。关闭连续上报后仍可用 `IMU_GET` 获取一帧近期有效样本。

触觉同样把“CP-M3025→ESP32 轮询”和“ESP32→USB 上报”分开。上电默认轮询两指、USB 触觉流关闭；开启流时两指必须都在最近 500 ms 内成功响应，否则返回 `ERR_STATE`。`HAPTIC_GET` 一次返回两个独立遥测帧。

## 四、遥测数据格式（TYPE=`0x02`，PAYLOAD 全小端 float32）

| 遥测 | PAYLOAD 布局 |
| --- | --- |
| IMU（op `0x22`） | `op` + `yaw` + `pitch` + `roll` + `q0 q1 q2 q3`（共 7 个 float，单位：角度°/单位四元数） |
| 触觉（op `0x32`） | 每帧一个模组：`op,index,address,status,point_count` + `seq(u32),timestamp_ms(u32)` + 合力 `Fx,Fy,Fz`(3×f32) + 77×(`Fx:i8,Fy:i8,Fz:u8`)；点值 0.1 N/LSB |
| 电机（op `0x16`） | `op` + `使能(u8)` + `位置` + `速度` + `力矩`（3 个 float）+ `错误码(u8)` |

## 五、应答 / 错误码（ACK 与 ERROR 的 PAYLOAD = `回显操作码` + `状态码`）

| 状态码 | 名称 | 含义 |
| --- | --- | --- |
| `0x00` | OK | 成功 |
| `0x01` | ERR_CRC | CRC 校验失败 |
| `0x02` | ERR_VERSION | 版本不匹配 |
| `0x03` | ERR_OPCODE | 未知操作码 |
| `0x04` | ERR_LOCKED | 会话未解锁却发了电机/自校准命令 |
| `0x05` | ERR_STATE | 当前状态机状态下不允许（如未 ACTIVE 就设目标） |
| `0x06` | ERR_ARG | 参数错误（如解锁密钥不对） |

## 六、典型使用流程

```
1) SESSION_UNLOCK(密钥)      -> ACK OK        # 先解锁
2) 首次：GRIP_CALIBRATE（约 50 s）；之后每次上电：GRIP_HOME（约 10 s），GRIP_GET_CALIB 查看结果
3) GRIP_GRIP(5.0)            -> ACK OK        # 以 5 N 夹取
4) GRIP_SET_CLOSE 5          -> ACK OK        # 张开
5) SAFE_STOP / MOTOR_DISABLE                  # 任何时候都可立即停止
```

## 七、真实示例帧（含正确 CRC，可直接发）

上位机 → 设备（COMMAND）：

| 命令 | 帧（hex） |
| --- | --- |
| PING | `A5 5A 01 00 01 01 00 01 15 1A` |
| GET_STATE | `A5 5A 01 00 02 01 00 03 8B A1` |
| 解锁 UNLOCK(0x47521000) | `A5 5A 01 00 03 05 00 07 00 10 52 47 E7 4B` |
| 开 IMU 流 | `A5 5A 01 00 07 01 00 20 CF 09` |
| 开触觉流 | `A5 5A 01 00 08 01 00 30 10 CF` |
| 整机自校准 | `A5 5A 01 00 09 01 00 04 73 CF` |
| 设遥测 50Hz | `A5 5A 01 00 0A 03 00 05 32 00 3A 96` |
| 安全停 | `A5 5A 01 00 0B 01 00 06 59 02` |

设备 → 上位机（ACK 示例）：

| 应答 | 帧（hex） |
| --- | --- |
| ACK 成功（使能） | `A5 5A 01 03 04 02 00 10 00 52 E0` |
| ACK 未解锁被拒 | `A5 5A 01 03 04 02 00 10 04 D6 A0` |

## 八、BLE 承载方式

服务/特征值（128-bit UUID，后 4 位区分）：

- 服务：`...0001`
- **控制特征** `...0002`（Write / Write-No-Response）：上位机把上面的**命令帧**写进来
- **遥测特征** `...0003`（Read / Notify）：设备把**响应/遥测帧**通过 Notify 推出来；Read 可取最近一帧

注意：遥测帧可能超过 BLE 默认 23 字节 MTU（满配 28 模块的触觉帧达 348 字节，而单条 Notify 只能携带 ATT_MTU−3 字节）。固件做了两层处理:

1. 请求更大的 MTU:`ble_att_set_preferred_mtu(APP_PROTO_MAX_FRAME + 3)`（约 412 字节),中心端协商接受后单条 Notify 即可放下最大帧;
2. **应用层分片兜底**:即使中心端只协商到较小 MTU,固件也会按当前 `ble_att_mtu()` 把一帧拆成多条 Notify 连续发出。

因此中心端(手机/PC)务必把**遥测特征的 Notify 数据当作字节流**顺序拼接后再喂给流式解析器(app_proto),而不是把每条 Notify 当成一个完整帧——解析器靠 SOF+长度+CRC 自动重组,跨 Notify 边界无感。

## 九、安全边界（重要）

会话密钥是写死在固件里的**共享常量**，只能挡"随手误控/明文控制"，**不是真正的加密或身份认证**。要用于正式产品，请再叠加 **BLE 配对绑定（bonding/加密）** 或一次性下发的**会话令牌**。这一层协议负责"结构化 + 校验 + 解锁闸门"，加密由链路层补齐。

---

# 夹爪高级功能（应用层 `components/gripper/`）

电机工作在达妙**力位混控模式**（寄存器 0x0A = 4）。每一帧命令同时给出目标位置、速度上限和电流上限（相对驱动器最大相电流的标幺值），驱动器自己把电流限制在上限以内，因此任何时刻夹持力都有上限。`gripper_tick()` 由 supervisor 每 10 ms 调用一次，输入是电机反馈和两块触觉垫的合力。

## 一、上电自检（`motor_twai`）

电机失能状态下读取 PMAX/VMAX/TMAX（0x15–0x17），反馈解码改用读回的数值；PMAX 只接受 9–25 rad。写入控制模式 4 和通讯超时 300 ms（0x09），并读回确认。以上写入都不存入电机 flash。自检不通过就不允许任何运动，每秒重试一次。

运行中的保护：
- 每次使能前重新读 0x0A 和 0x09，不符就重新自检（电机重启后会回到 flash 里的模式）。
- 使能后反馈却显示“失能且无故障码”，判定电机重启，重新自检。
- 驱动器报故障时，电机断电并锁定，直到收到 `MOTOR_CLEAR_ERR`（命令行 `clear`）。例外：开机后第一帧反馈就是 0xD（CAN 通信丢失）时，说明本板在电机使能时重启过（烧录、复位），驱动器已自行停机；固件自动清除一次，电机保持断电。开机 1 s 之后再出现 0xD 仍按故障锁定。
- 自检时把驱动器的梯形加减速度 ACC/DEC 写为 ±80 rad/s²、位置环增益 KP_APR 写为 108（只写内存，每次自检重写，不存入驱动器 flash）。驱动器出厂值 ACC/DEC ±2 rad/s² 把所有移动限制在约 0.9 rad/s；KP_APR 54 时，临近目标按约 11 × 剩余距离 rad/s 逼近，每次移动最后约 0.5 s 花在这一段。台架 2026-10-23，闭合 5 %↔95 %（6.4 rad）、15 rad/s：KP_APR 54 时单程 1.29–1.40 s，108 时 0.81–0.89 s，两者都没有越过目标。
- 设定值 150 ms 未刷新，电机任务自行失能电机。
- 反馈中断超过 300 ms，失能并重新自检。
- 每次重新自检后自检计数加一，夹爪据此作废当前标定，需要 `GRIP_HOME`（原点检查）或 `GRIP_CALIBRATE`。
- 反馈中断时删除并重建 TWAI 节点。原因：没有节点应答的帧会被控制器一直重发，若不清除，电机恢复供电时会收到断电前的旧命令帧。
- 自检失败时的诊断（第 1–3 次和之后每 10 次打印）：CAN 控制器状态、发送/接收错误计数、按类型统计的总线错误（ACK、位、格式、填充），以及收到的其它 ID 的帧。硬件滤波接收所有标准帧，由接收回调只解码 MST_ID，其它帧只做记录。判读：只有 ACK 错误 = 帧已发到总线但电机未应答（查电机供电、CANH/CANL 是否接到电机、是否接反、终端电阻）；位错误 = 本机收发器或 TX/RX 接线；格式/填充错误 = 波特率不一致；收到其它 ID = 电机的 CAN_ID 或 Master ID 与配置不同，日志会给出应改的宏。
- `MOTOR_SAVE_CONFIG`（`0x17`）可把模式 4 和通讯超时存入电机 flash，使电机自己重启后仍处于力位混控模式。

## 二、标定（`0x40` 触发；`main/app_config.h` 中 `GRIPPER_AUTOCAL_ON_BOOT` 为 1 时上电 60 s 内自动执行一次 `GRIP_HOME`，当前为 0）

1. 电机失能，记录两块触觉垫的基线。
2. **找最大位置**：以 0.05 rad/s 张开。电流上限从 0.005 pu 起，夹爪不动时每 150 ms 乘 1.15；夹爪动起来后设为 0.45 N·m（0.36 × 1.25，按自检读到的 Kt × Imax 换算，本电机 19.27 N·m/pu）。本夹爪张开端没有硬限位，越往外张开所需力矩越大（台架：行程中段 0.28 N·m，72 mm 处 0.32，约 77 mm 处 0.40，80 mm 处 0.51 N·m），所以张开力矩的 0.5 s 平均值达到 0.36 N·m（`CAL_OPEN_TQ_NM`）即为张开端；开始运动 400 ms 后 0.1 s 平均值达到 0.414 N·m 也立即判定。在 0.45 N·m 上限下停住时，力矩已超过阈值，同样判为张开端。开始张开 0.3 rad 内就达到阈值（起始位置已在张开端之外）时，先合拢 2.5 rad 再张开一次。从张开端回退 0.10 rad，记为最大位置。张开过程中任一垫比基线高 0.5 N 即中止并反向后退：`CLOSE_DIR` 方向反了。若持续以不到指令 40 % 的速度移动，以“speed mismatch”中止。台架 2026-10-23：起动电流的 1.5 倍（0.29 N·m）曾作为上限，低于阈值，夹爪在行程中途停住并被误判为张开端，因此改为上述固定上限。
3. 在最大位置做触觉清零。
4. **找最小位置**：合拢。离上次记录的接触位置 0.25 rad 以外用 0.3 rad/s，之内（首次标定为全程）用 0.05 rad/s。开始滑动 300 ms 后，按力矩反馈算出维持滑动所需电流，把上限改为它的 1.5 倍再加 0.024 N·m，且高出滑动所需的部分不超过 10 N × 名义杠杆（触觉读数全为 0 时，垫上的力由这部分决定）。之后每 300 ms 按当地摩擦重新计算一次，可降可升，但不超过第一次的值。任一垫第一次读到 0.3 N 就停在该位置。两块垫都连续两次读到 0.3 N，或其中一块连续两次读到 0.3 N 且在原位保持 0.5 s，记为接触位置。两个夹指不完全平行，轻接触时可能只压到一块垫的测点（台架 2026-10-22：两垫互压，ring 1.03 N 时 little 0.00 N；手压时 ring 4.10 N、little 1.90 N），所以只有一块垫读到接触不算故障；标定和夹取都取两块垫中较大的读数作为夹持力。然后在接触位置轻压两块垫，测“电流→夹持力”系数 k（N/pu）和垫刚度（N/rad）：
   - 先退到接触位置后 0.01 rad 卸载；
   - 电流上限按台阶升高，每级 = 0.002 N·m ÷ 限位处实测的 N·m/pu，取整到电流指令的分辨率 0.0001 pu（本电机每级 1 个分辨率，垫上约 0.19 N）；
   - 每级保持 150 ms，夹爪停稳后取最后 50 ms 的触觉平均值记一个点，日志逐点打印 “squeeze point”；
   - 在 0.5–2.0 N 区间内至少取 8 个点（不够时继续到 3.5 N）；
   - 去掉开头“力几乎不随电流增加”的点（夹爪带速度碰上垫后被摩擦保持的力），再用两两斜率的中位数（Theil–Sen）拟合；
   - 接受条件：拟合残差 ≤ 0.2 N、电流跨度 ≥ 3 级、(限位处 N·m/pu) ÷ k 得到的杠杆在名义杠杆的 0.6–1.4 倍之内。名义杠杆 `GRIP_LEVER_NOMINAL_M` 现为 10.4 mm（72 mm 开口 ÷ 6.9 rad 行程的平均值），应改为在接触附近实测的值。两块垫按同一比例读小时，算出的杠杆偏长，会被拒绝；
   - 速度上限保持 0.05 rad/s，以限制卡滞后突然滑动时的冲击力；电流上限不超过起始电流加 0.2 N·m。再回退 0.05 rad，确认两块垫都低于 0.2 N，记为最小位置。
5. 校验行程（与上次记录相差超过 15 % 时拒绝一次并清除记录，再次标定即接受）。限位、最大、最小、接触位置以及 k、垫刚度等全部写入 NVS。最后张开到最大位置。

轻压测 k 时，若静摩擦明显大于滑动摩擦，夹爪会先卡住再突然滑动，测点拟合不成直线。此时标定照常完成，k 改用电机参数估算（19.27 N·m/pu ÷ 名义杠杆 10.4 mm = 1852.8 N/pu），日志打印 “k not measured”。夹持力由触觉闭环决定，k 只用于计算电流上限。

**原点检查（`0x41`）**：输出轴编码器是单圈绝对值，行程约 7.2 rad 超过一圈，上电后读数可能整体差 2π。原点检查在电机断电时把触觉清零，然后合拢到两垫接触（离保存的接触位置及其 ±2π 位置 0.25 rad 以内用 0.05 rad/s），与保存的接触位置比较：相差 ≤ 0.03 rad（或整圈 2π 加 ≤ 0.03 rad）则把全部位置按实测差值平移后恢复；相差更大则报错并保留保存的标定。约 40 s。

任一垫超过 4 N、合拢停住却没有触觉信号、行程超过 8.0 rad、超时（每阶段 240 s），都中止标定；与触觉有关的故障先朝张开方向后退 0.3 rad（不越过机械限位 0.02 rad 以内，张开阶段不越过本次寻位的起点），后退中触觉力再上升 0.5 N 就立即失能。

本夹爪的行程为 6.9 rad（输出轴 395°，台架实测：张开 −2.00 rad，两垫接触 +4.89 rad），超过一整圈。首次标定张开和合拢都用 0.05 rad/s，约 3–4 分钟；有保存记录后合拢只在接触位置前 0.25 rad 内慢速。输出轴编码器是单圈绝对值，行程超过一圈，因此上电时夹爪若在合拢一端，读数会整体差 2π；原点检查会识别并平移。

## 三、开合控制

`GRIP_SET_CLOSE`、`GRIP_MOVE_DEG` 只在标定完成后、在闭合 5 %–95 % 之间移动（两端各留 5 % 余量；标定和 `GRIP_HOME` 结束时也停在 5 %）。位置目标按 40 rad/s² 加速和减速，到达目标时速度为 0；速度可由命令给出，范围 0.05–15 rad/s（15 rad/s 时 5 %–95 % 的 6.6 rad 约 0.82 s，10 rad/s 时约 0.91 s，受 40 rad/s² 加速度限制），不给时为 2 rad/s；张开、合拢相同。电流上限不是固定值：起始值张开 1.0 N·m、合拢为起动电流 + 3 N / k；驱动器已达上限、夹爪落后于位置目标但仍在移动时，说明运动本身需要更大力矩（速度、加速度、局部摩擦），每 100 ms 把上限乘 1.25，张开最多到 3.0 N·m（电机额定 3.5 N·m），合拢最多到 1.0 N·m；已达上限且 300 ms 不动，判为被挡住，停在原地并报 “move blocked”。合拢途中触觉力上升超过起始值 0.5 N 且不低于 1 N 即停止。合拢时碰到物体的冲击力估算为每 1 rad/s 约 7.6 N：2 rad/s 约 15 N，5 rad/s 约 38 N，10 rad/s 约 76 N，15 rad/s 约 114 N（后两者超过触觉垫 50 N 安全负载）（假设转动惯量 0.002 kg·m²、垫刚度 300 N/rad，均未实测）。

## 四、定力夹取（`GRIP_GRIP`）

1. 夹爪比 80 % 开度更合拢时，先张开到 80 %；否则从当前位置直接合拢。合拢速度由命令给出（0.05–15 rad/s，不给时 2 rad/s），按 40 rad/s² 加速和减速，目标为 95 % 闭合位置。接近时电流上限 = 摩擦电流 + 2 N / k，只在运动本身需要时提高；已达上限且不动 300 ms 而触觉无力，判为物体在垫外，后退并报错。位置目标最多越过接触位置 0.10 rad。每次接触后 0.5 s 内的垫上最大力和接触时的速度写入日志（“grip contact: … peak …”），用于核对冲击力。
2. 任一垫读到 0.3 N 进入力控。若此时位置距接触位置不到 0.03 rad，判定为空抓，退回最小位置。
3. 力控取两块垫中较大的合力，分三区：低于目标带时推进并按积分增大电流；在目标带内保持当前位置；高于目标带时减小电流，并让位置目标以 0.02 rad/s 后退，克服摩擦主动卸载。目标带宽 = max(0.25 N, 4 % × 目标力)。电流上升速度限制为 8 N/s ÷ k；电流上限 = (2 × 目标力 + 3 N) / k，且不超过 25 N / k：k 用电机参数估算时可能偏大，这样即使 k 偏大一倍也能达到目标力；实际夹持力由触觉闭环决定。
4. 推进且停住时，若触觉力低于 0.5 × k ×（电流 − 偏置）− 1 N 持续 300 ms，判定触觉读数偏低或物体压在垫外，后退并停止。
5. 触觉数据中断时不再增加电流，只保持前馈电流；恢复后继续调节。中断超过 10 s 则结束力控，在当前位置以当时的电流保持物体，状态变为 IDLE、故障为“tactile stale while holding”，由上位机决定 `GRIP_SET_CLOSE 5`（张开）或重新 `GRIP_GRIP`。
6. 接触丢失 100 ms 后回到慢速合拢。夹持中触觉被清零（计数变化）则后退。
7. 物体被取走：150 ms 内力下降超过最近峰值的 20 %（且 ≥ 0.5 N）时开始 1 s 的观察。力高于目标带期间、离开目标带后 150 ms 内、以及改变目标力时，不计入下降判断（这些下降由控制器自己造成）。观察期内夹爪比开始观察时又合拢至少 0.01 rad，且到达接触位置前 0.03 rad 以内，才判定两垫之间已无物体，报“empty grasp”并退回最小位置。
8. 发出 `GRIP_GRIP` 时若两垫已经受力（例如触觉中断 10 s 后仍夹着物体），直接从当前电流进入力控，不做首次接触的空抓判断。
9. 过力后退后 2 s 内拒绝新的 `GRIP_GRIP`，以及朝合拢方向的 `GRIP_SET_OPEN` / `GRIP_MOVE_DEG`。
10. 首次接触时，物体表面若在接触位置前 0.03 rad 以内（杠杆 20 mm 时约 0.6 mm），无法与两垫直接接触区分，按空抓处理。

## 五、力和电流的保护层级

| 层级 | 触发条件 | 反应 |
| --- | --- | --- |
| 驱动器 | 每帧命令的电流上限 | 驱动器硬件限流 |
| 夹爪 | 合力超过目标 + max(1 N, 30 %) | 电流立即降到前馈的一半并卸载 |
| 夹爪 | 触觉力低于电流对应值的 30 % | 只打印警告，继续调节 |
| 夹爪 | 任一垫 ≥ 23 N，或任一测点 ≥ 6 N（连续 2 个读数） | 后退 0.3 rad 后保持（标定中则失能） |
| supervisor | 电机使能时任一垫 ≥ 24 N | SAFE_STOP，失能电机，需 `0x15` 解除 |
| motor_twai | 设定值 150 ms 未刷新 / 反馈 300 ms 中断 / 电机寄存器被改 | 失能电机 / 重新自检 |
| 电机 | 300 ms 收不到命令 | 驱动器自行失能 |

触觉传感器量程 0–25 N，安全负载 50 N（手册）。目标力上限 20 N。单个测点 6 N 的阈值是估计值，手册没有给出单点额定值。上位机的触觉清零（`0x34`）只在两块垫数据新鲜（200 ms 内）且都低于 0.5 N、夹爪未驱动电机时执行。

## 五、夹爪命令示例帧（含正确 CRC；均需先解锁）

| 命令 | 帧（hex） |
| --- | --- |
| 完整标定 | `A5 5A 01 00 20 01 00 40 0A 03` |
| 原点检查（恢复保存的标定） | `A5 5A 01 00 21 01 00 41 9F 65` |
| 闭合到 50 %（参数 50.0） | 用 `host/3_gripper_cli.py close 50` 发送 |
| 5N 定力夹持 | `A5 5A 01 00 23 05 00 43 00 00 A0 40 C6 EE` |
| 张开释放 | `A5 5A 01 00 24 01 00 44 7F 89` |
| 设默认力 8N | `A5 5A 01 00 25 05 00 45 00 00 00 41 D7 AE` |
| 保持/停 | `A5 5A 01 00 26 01 00 46 55 44` |
| 读标定 | `A5 5A 01 00 27 01 00 47 C0 22` |

典型开机流程：`解锁 → GRIP_HOME（首次用 GRIP_CALIBRATE）→ GRIP_GRIP / GRIP_SET_CLOSE → …`

电机配置写入电机 flash（只需一次）：`A5 5A 01 00 28 01 00 17 DB AC`（`MOTOR_SAVE_CONFIG`，需先解锁）。

台架命令行工具 `host/3_gripper_cli.py`（需 pyserial，先关闭 idf.py monitor）：`watch` 在电机失能时显示位置和两块垫的力，用来手动确认 `CLOSE_DIR`；`calibrate`、`home`、`calib`、`close 50 [--speed v]`、`grip 3 [--speed v]`、`stop`、`disable`、`clear`、`save-motor-config`。命令运行中按 Ctrl+C 会立即发送 `MOTOR_DISABLE`。

## Protocols implemented

The real wire formats were decoded from `source_files/` and are now implemented
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

  | | 无名指近节 (12) | 小拇指近节 (16) |
  |---|---|---|
  | 分布力 77×3 = 231 B | `0x2800` | `0x3000` |
  | 合力 3×int16 | `0x0548` | `0x0560` |
  | 点数 uint16 | `0x0048` | `0x0050` |
  | 连接位 | `0x0011` bit4 | `0x0012` bit0 |

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
  and **小拇指** connectors as proximal modules (slots 12 and 16). Plugged into a
  different finger connector, the board reports them at different slots and the
  start-up discovery log says so. Only request/response polling is implemented;
  the board's auto-push mode (`0x0017`) is disabled at boot and would give a
  higher frame rate. The 77 values are displayed row-major; confirm physical
  rotation/mirroring by pressing one corner before using the map for control.
- **IMU**: startup checks only 9600 and 115200, using a read-only register query
  to identify/wake the unit. A unit found at 9600 is moved to 115200 for the
  powered session (not saved), then ACC/GYRO/ANGLE output at 100 Hz is applied
  without saving. A stopped stream is woken once and then re-detected at those
  same two rates. Yaw is intentionally not exposed.
- **Motor/gripper**: verified on the bench on 2026-10-23 (calibration, home check,
  moves up to 15 rad/s, force grip on a sponge). Open items are listed in
  [README_gripper.md](README_gripper.md) section 7. Uses factory IDs ESC_ID
  `0x001`, MST_ID `0x000`.
- **OLED**: initialises the SH1106 and pushes a framebuffer, but has no font, so
  it shows a heartbeat bar and logs the real values.
- **Wi-Fi** is a logged no-op skeleton; **BLE** has no pairing/bonding/security.

Remaining gaps are marked with `TODO:` comments in the source.

## Next implementation steps

1. Haptic: verify address 1/2 and point ordering on hardware, then lock the
   physical rotation/mirroring constants for the two mounted fingers.
2. IMU: send the WitMotion config sequence at startup so the unit emits the
   desired packets/rate regardless of prior state.
3. Motor: first powered run with the jaw empty; check the probe log, the
   calibration log (measured speed, stall torque, k) and `CLOSE_DIR`.
4. Add a 5×7 font + text renderer to `oled_sh1106`.
5. Flesh out the supervisor safety policy (force limits, orientation limits,
   fault recovery out of `ERROR`/`SAFE_STOP`).
6. Add BLE security (bonding) and, if needed, the Wi-Fi control server.

## Build verification status

> ✅ **Verified.** Built successfully with **ESP-IDF v6.0** for `esp32c6`
> (`idf.py set-target esp32c6 && idf.py build`) — `gripper_plus.bin` links with
> ~27% of the app partition free. Reproduce locally with the build commands
> above after exporting ESP-IDF.
