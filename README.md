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
   ├─ app_protocol/          # shared types + queue "bus" + user protocol codec
   │                         #   (app_proto.c: framing/CRC/opcodes for BLE+WiFi)
   ├─ board_xiao_esp32c6/    # board bring-up (NVS, banner)
   ├─ haptic_uart/           # UART1 921600, 0xAA56 auto-push + LRC parser
   ├─ imu_uart/              # UART0 9600, WitMotion 11-byte frame parser
   ├─ motor_twai/            # TWAI (CAN) DM-J4310 MIT-mode codec + API
   ├─ oled_sh1106/           # optional I2C OLED (APP_ENABLE_OLED)
   ├─ wireless_ble/          # NimBLE GATT control interface
   ├─ wireless_wifi/         # optional Wi-Fi control path (skeleton)
   ├─ gripper/               # high-level app: auto-calibrate, force-grip, NVS
   └─ app_supervisor/        # INIT/IDLE/ACTIVE/ERROR/SAFE_STOP + gripper tick
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

## Tactile sensor GUI (host tool)

`tools/gui/paxini_gui.html` is a standalone Web Serial console for the PaXini
CP-M3025 (77-point) sensor via the 通信盒: live force map, resultant-force
scope, calibration, and CSV capture. Open it in Chrome/Edge and connect to the
通信盒's USB port. See `tools/gui/README.md`. It talks to the sensor board
directly (not through the ESP32).

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
| `ble_control`     | wireless_ble     | NimBLE host; parses framed protocol → `ctrl_q`  |
| `oled` (optional) | oled_sh1106      | render system/sensor status                     |
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
| `0x04` | SELF_CALIBRATE | 无 | **是** | **整机自校准**：依次做 IMU 自校准 + 触觉清零 + 电机位置置零 |
| `0x05` | SET_TELEM_RATE | u16 Hz | 否 | 设置遥测上报频率（默认 20Hz；受 50Hz 控制循环限制，超出按 50Hz 钳制） |
| `0x06` | SAFE_STOP | 无 | 否（安全指令，永远允许） | 立即断电机并进入 SAFE_STOP |
| `0x07` | SESSION_UNLOCK | u32 密钥 | 否 | 会话解锁；密钥 = `0x47521000`，正确才解锁 |
| `0x08` | SESSION_LOCK | 无 | 否 | 重新上锁 |

### 电机类 `0x1-`（全部需解锁）

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x10` | MOTOR_ENABLE | 无 | 使能电机（发达妙 `…FC` 帧），进入 ACTIVE |
| `0x11` | MOTOR_DISABLE | 无 | 失能电机（`…FD`），进入 IDLE |
| `0x12` | MOTOR_SET_TARGET | f32 | 设置目标位置（rad）/夹持量，仅 ACTIVE 下有效 |
| `0x13` | MOTOR_SET_GAINS | f32 Kp, f32 Kd | 设置 MIT 阻抗增益（Kd 建议非 0） |
| `0x14` | MOTOR_SAVE_ZERO | 无 | 把当前位置设为零点（`…FE`） |
| `0x15` | MOTOR_CLEAR_ERR | 无 | 清除电机错误（`…FB`），从 ERROR 回到 IDLE |
| `0x16` | MOTOR_GET_STATUS | 无 | 立刻回一帧电机遥测 |

### IMU 类 `0x2-`

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x20` | IMU_STREAM_EN | 无 | 开启 IMU 遥测流 |
| `0x21` | IMU_STREAM_DIS | 无 | 关闭 IMU 遥测流 |
| `0x22` | IMU_GET | 无 | 立刻回一帧 IMU 遥测 |
| `0x23` | IMU_CALIBRATE | 无 | IMU 自校准（加计/陀螺，保持静止） |
| `0x24` | IMU_SET_ZERO | 无 | 偏航角 / Z 轴角度归零 |

### 触觉类 `0x3-`

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x30` | HAPTIC_STREAM_EN | 无 | 开启触觉遥测流（并使能传感器自动回传） |
| `0x31` | HAPTIC_STREAM_DIS | 无 | 关闭触觉遥测流 |
| `0x32` | HAPTIC_GET | 无 | 立刻回一帧触觉遥测 |
| `0x33` | HAPTIC_SET_CONTENT | u16 | 选择回传内容（bit0=合力，bit1=分布力） |
| `0x34` | HAPTIC_CALIBRATE | 无 | 触觉模块清零/校准 |

### 夹爪高级动作 `0x4-`（`0x40~0x46` 需解锁，`0x47` 只读）

| 操作码 | 名称 | 参数 | 说明 |
| --- | --- | --- | --- |
| `0x40` | GRIP_CALIBRATE | 无 | **自动标定**：合拢撞限位→回退定零点，张开撞限位→回退定最大开合，写入 flash |
| `0x41` | GRIP_HOME | 无 | 回到完全闭合零点 |
| `0x42` | GRIP_SET_OPEN | f32 | 开合度 0.0（闭合）~ 1.0（最大），在标定范围内线性映射 |
| `0x43` | GRIP_GRIP | f32 | 定力夹持；参数为目标夹持力(N)，0 表示用 flash 里的默认力 |
| `0x44` | GRIP_RELEASE | 无 | 张开到最大 |
| `0x45` | GRIP_SET_FORCE | f32 | 设置并**持久化**默认夹持力(N) |
| `0x46` | GRIP_STOP | 无 | 保持当前位置 |
| `0x47` | GRIP_GET_CALIB | 无 | 读取标定结果，回 RESPONSE（有效位+闭合位+最大位+默认力+夹爪状态） |

## 三、自动启用（auto-enable）

只要上位机对某个子系统发了命令，`supervisor` 会**自动打开该子系统的遥测流**，无需单独再发"开流"指令：

- 发任意 **电机** 命令（`0x10~0x16`）→ 自动开电机遥测流
- 发任意 **IMU** 命令（`0x20~0x24`，`STREAM_DIS` 除外）→ 自动开 IMU 遥测流
- 发任意 **触觉** 命令（`0x30~0x34`，`STREAM_DIS` 除外）→ 自动开触觉遥测流

## 四、遥测数据格式（TYPE=`0x02`，PAYLOAD 全小端 float32）

| 遥测 | PAYLOAD 布局 |
| --- | --- |
| IMU（op `0x22`） | `op` + `yaw` + `pitch` + `roll` + `q0 q1 q2 q3`（共 7 个 float，单位：角度°/单位四元数） |
| 触觉（op `0x32`） | `op` + `错误码(u8)` + `模块数 N(u8)` + N×(`fx fy fz` float，单位 N) |
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
2) MOTOR_ENABLE              -> ACK OK        # 使能（自动开电机遥测）
3) IMU_STREAM_EN             -> ACK OK        # 开 IMU 流
4) HAPTIC_STREAM_EN          -> ACK OK        # 开触觉流
5) MOTOR_SET_TARGET(0.5)     -> ACK OK        # 控制夹爪
   ...  设备持续上报 TELEMETRY(IMU/触觉/电机)
6) SELF_CALIBRATE            -> ACK OK        # 需要时整机自校准
7) MOTOR_DISABLE / SAFE_STOP                  # 收工
```

## 七、真实示例帧（含正确 CRC，可直接发）

上位机 → 设备（COMMAND）：

| 命令 | 帧（hex） |
| --- | --- |
| PING | `A5 5A 01 00 01 01 00 01 15 1A` |
| GET_STATE | `A5 5A 01 00 02 01 00 03 8B A1` |
| 解锁 UNLOCK(0x47521000) | `A5 5A 01 00 03 05 00 07 00 10 52 47 E7 4B` |
| 电机使能 | `A5 5A 01 00 04 01 00 10 40 A4` |
| 设目标 0.5 rad | `A5 5A 01 00 05 05 00 12 00 00 00 3F 00 38` |
| 设增益 Kp5 Kd1 | `A5 5A 01 00 06 09 00 13 00 00 A0 40 00 00 80 3F 2C F1` |
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

把底层"发 CAN 帧转电机"封装成"夹爪该有的动作"。核心是一个状态机 `gripper_tick()`，由 `supervisor` 每个控制周期（~50Hz）喂入最新的**电机反馈 + 触觉数据**来推进。

## 一、自动标定（找零点与最大开合，写 flash）

`GRIP_CALIBRATE (0x40)` 触发，全自动、无需人工量：

1. **找闭合限位**：以"命令位置永远比当前位置领先一点点"的方式慢速合拢——这样施加的力矩被限制住，不会硬怼。撞到机械限位后电机走不动，**速度≈0 持续一段时间**即判定到限位。
2. 从限位**回退一个余量**（`CAL_BACKOFF_RAD`）作为**闭合零点 `pos_closed`**（对应开合度 0）。
3. **找张开限位**：反方向重复，撞到张开限位→回退→**最大开合 `pos_open`**（对应开合度 1）。
4. 把 `{pos_closed, pos_open, 默认夹持力}` 存进 **NVS flash**（`nvs namespace "gripper"`），断电不丢；开机 `gripper_init()` 自动读回。

> 判定用的是电机**速度**（撞死后速度归零）而非硬阈值电流，更好调；力矩/电流可作二次确认。方向 `GRIPPER_CLOSE_DIR`、余量、停滞时长等都在 `gripper.c` 顶部集中，接上真机后按需微调（已标 `TODO`）。

## 二、开合控制

- `GRIP_HOME (0x41)`：回到 `pos_closed`。
- `GRIP_SET_OPEN (0x42, f)`：`目标位 = pos_closed + f×(pos_open − pos_closed)`，`f∈[0,1]`。位置环（MIT 阻抗）跟踪，到位且静止即进入 IDLE 保持。
- `GRIP_RELEASE (0x44)`：张到 `pos_open`。
- `gripper_get_opening()` 反算当前开合度 0~1。

## 三、定力夹持（触觉力闭环）

`GRIP_GRIP (0x43, force_N)`：

1. **合拢找接触**：慢速合，同时读触觉板**合力（∑Fz）**；力超过接触阈值（或撞到刚性物体速度归零）→ 判定夹到东西。
2. **切定力保持**：改用**力矩前馈**做恒力，并用触觉力做 **P 闭环**微调：
   `t_ff += Kp_force × (目标力 − 实测触觉力)`（限幅 `[0, 3N·m]`），Kp=0、留小 Kd 阻尼。
   → 实际夹持力被稳定在设定值，物体软硬不同也能自适应。
3. `GRIP_SET_FORCE (0x45, N)` 设定并持久化默认夹持力；`GRIP_GRIP` 传 0 即用它。

## 四、状态机

```
UNCAL ──calibrate──▶ CAL_CLOSE ─撞限位─▶ CAL_OPEN ─撞限位─▶ IDLE(已标定, 存flash)
IDLE ──set_open/home/release──▶ MOVING ─到位─▶ IDLE
IDLE ──grip──▶ GRIPPING ─接触/卡住─▶ HOLDING(恒力) ──stop/release──▶ IDLE
任意 ──stop──▶ IDLE(保持当前位)      SAFE_STOP 由 supervisor 兜底
```

## 五、夹爪命令示例帧（含正确 CRC；均需先解锁）

| 命令 | 帧（hex） |
| --- | --- |
| 自动标定 | `A5 5A 01 00 20 01 00 40 0A 03` |
| 回零 | `A5 5A 01 00 21 01 00 41 9F 65` |
| 开合到 50% | `A5 5A 01 00 22 05 00 42 00 00 00 3F 42 91` |
| 5N 定力夹持 | `A5 5A 01 00 23 05 00 43 00 00 A0 40 C6 EE` |
| 张开释放 | `A5 5A 01 00 24 01 00 44 7F 89` |
| 设默认力 8N | `A5 5A 01 00 25 05 00 45 00 00 00 41 D7 AE` |
| 保持/停 | `A5 5A 01 00 26 01 00 46 55 44` |
| 读标定 | `A5 5A 01 00 27 01 00 47 C0 22` |

典型开机流程：`解锁 → GRIP_CALIBRATE（首次，之后开机自动读 flash）→ GRIP_SET_OPEN/GRIP → …`

## 六、其它模块可复用的高级抽象

已实现或已预留接口，便于继续往上叠：

- **IMU**：`imu_uart_calibrate()`（加计/陀螺自校准）、`imu_uart_set_zero()`（偏航归零）已接入 `IMU_CALIBRATE/SET_ZERO`；可再加"水平检测/静止检测/敲击检测"（基于角度与角速度阈值）。
- **触觉**：`haptic_total_force()` 合力求和、接触检测已用于夹持；可再加"打滑检测"（合力对时间的变化率）、"抓取质量评估"（多指力分布）。
- **电机**：`motor_twai_set_mit()` 暴露完整 MIT（位置/速度/力矩），夹爪的位置环与力环都基于它；力矩即电流，用于撞限位与恒力。
- **整机自校准** `SELF_CALIBRATE`：一键串联 IMU 自校准 + 触觉清零 + 电机位置置零。

## Protocols implemented

The real wire formats were decoded from `source_files/` and are now implemented
(not stubs):

- **Haptic (PaXini PX-6AX GEN3, sensor `CP-M3025-Core / PXSR-STDCP03B`, 77
  points, via the 高速通信集成板 / 通信盒, `通信盒通信协议_V1.0.5`)** — 921600 8N1. The driver
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
