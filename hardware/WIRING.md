**中文** | [English](WIRING.en.md)

# 接线说明

本项目以 Seeed Studio XIAO ESP32-C6 作为主控板，负责 USB 调试/下载、触觉集成板通信、IMU 通信和电机控制。本文整理主控与各子模块之间的接线，便于接线、排查和维护。

固件中所有引脚只在 [`../firmware/main/app_pins.h`](../firmware/main/app_pins.h) 中定义，波特率、UART 控制器分配和 CAN 参数在 [`../firmware/main/app_config.h`](../firmware/main/app_config.h) 中定义。两者与本文不一致时，以固件为准。

## 1. 引脚表（XIAO ESP32-C6）

已与项目原理图（`esp32_c6_xiao` / Sub1）核对。原理图中的 **UART1** 是触觉链路，**UART2** 是 IMU 链路，这是逻辑名称，不是 ESP32 的 UART 控制器编号。

| 原理图网络 | 功能 | XIAO 引脚 | GPIO | ESP 外设 |
| --- | --- | --- | --- | --- |
| (USB-C) | USB 调试 / 下载 | USB-C | 内部（GPIO12/13） | USB Serial/JTAG |
| UART2_rx | IMU RX（XIAO ← IMU） | D0 | 0 | UART0，9600 → 115200 |
| UART2_tx | IMU TX（XIAO → IMU） | D1 | 1 | UART0，9600 → 115200 |
| TWAI TX | 电机 CAN TX | D2 | 2 | TWAI（需外接收发器） |
| TWAI RX | 电机 CAN RX | D3 | 21 | TWAI（需外接收发器） |
| SDA | OLED I2C SDA（可选，当前固件未用） | D4 | 22 | I2C0 @ 400 kHz |
| SCL | OLED I2C SCL（可选，当前固件未用） | D5 | 23 | I2C0 @ 400 kHz |
| UART1（见注） | 触觉 RX（XIAO ← 集成板） | D6 | 16 | UART1 @ 921600 |
| UART1（见注） | 触觉 TX（XIAO → 集成板） | D7 | 17 | UART1 @ 921600 |

- TX/RX 均按 XIAO 一侧命名（XIAO TX → 外设 RX）。
- IMU 为 JY901P（内置 MCU 姿态解算与校准），经 GPIO Matrix 接 UART0。上电先用 9600 通信，确认链路后切到 115200。
- OLED 为 SH1106 驱动的 1.3 inch 屏，非必需。当前固件已移除 OLED 驱动，D4/D5 在 `app_pins.h` 中标为空闲。
- D8–D10 未使用。
- 注：D6/D7 的方向与 XIAO 丝印相反，见第 3 节。原理图网络 `UART1_tx` / `UART1_rx` 具体落在哪一脚，旧 README 与 `app_pins.h` 注释说法不一致，接线以上表的功能列和固件定义（`PIN_HAPTIC_UART_TX = 17`，`PIN_HAPTIC_UART_RX = 16`）为准。

固件中的 UART 控制器分配：触觉链路 → `UART_NUM_1`，IMU 链路 → `UART_NUM_0`。C6 有两个 HP UART 控制器，控制台走 USB Serial/JTAG，所以两个都可用。修改位置：`../firmware/main/app_config.h`。

## 2. 电源

> ⚠️ **XIAO 的 USB 供电和 PCB 上 24 V→5 V DC-DC 模块（P1）的供电不能同时接入。** 开发阶段建议不焊 DC-DC 模块：这样同时接电机 24 V 电源线和 USB 线，所有供电都正常且没有冲突。脱离电脑独立使用时再焊上 DC-DC 模块，此后不要再同时插 USB。（原理图注释）

- XIAO 的 3V3_out（pin 11）和 GND（pin 12）给 IMU 供电，3V3 上有一颗 10 µF 储能电容。
- **触觉集成板不能用 3V3 供电**，必须用 VBUS / 5 V，原因见第 3.1 节。
- 电机由独立的 24 V 电源供电，见第 4 节。
- 所有模块共地。

## 3. 触觉集成板

D6/D7 接集成板的 4 针 UART 座：

- 连接器 AFC12-S04DCA-00，单排 4 针：`pin1 TX, pin2 RX, pin3 GND, pin4 VCC`。丝印把标签错开印成两行，但针脚是单排。
- **触觉 TX/RX 与 XIAO 丝印方向相反。** 座子上标的是集成板自身的引脚功能，线束按标签同名对接，所以 D6 接在集成板的发送端，D7 接在集成板的接收端：

  ```
  XIAO D7 (GPIO17) → 集成板 pin2 RX    所以 GPIO17 是 ESP 的 TX
  XIAO D6 (GPIO16) ← 集成板 pin1 TX    所以 GPIO16 是 ESP 的 RX
  ```

  固件因此把 GPIO17 作 TX、GPIO16 作 RX（见 `../firmware/main/app_pins.h`）。已在硬件上验证：按此映射读版本号返回 `ADAPTER_FW02.26`；两线对调后，任何波特率下集成板都不响应。如果以后线束改成交叉接法，要把这两个定义换回来。
- 两边必须共地。
- 集成板全双工，自带电源输入：4.5–5 V，接 WAFER-HY2.0-2PWB 连接器，不从 XIAO 的 3V3 取电。
- 两个传感器分别插在集成板的 **无名指** 和 **小拇指** 接口。各模块的 SPI 由集成板处理，ESP 一侧不需要接 CS 或上拉。

如果链路不通，驱动启动时的发现流程会依次尝试两种引脚方向 × 六种波特率 × 两种候选协议格式，并把收到的任何回复以十六进制打印出来。上面的映射就是这样确定的。

### 3.1 集成板必须用 5 V：用 3V3 时的故障看起来像传感器坏了

> 手册 §4.3 规定 WAFER-HY2.0-2PWB 连接器输入 **4.5–5 V**。如果改用 XIAO 的 3V3，集成板仍会启动、仍会响应 UART、仍会返回版本号，也能正常识别 **一个** 传感器，只是找不到第二个，而且不报错。必须用 VBUS / 5 V，不要用 3V3。
>
> 接错时的现象，以及为什么这些现象都不指向电源：
>
> - `0x0010`–`0x0013` 读回 `00 10 00 00`：只映射了无名指模块。
> - 缺失槽位的点数（`0x0050`）读为 0，模块错误码（`0x0700 + n×2`）读为 `0x0001`，与真正空槽位的返回值相同。集成板并没有报告故障，它只是没发现这个模块。
> - 读缺失模块的数据区仍然成功，返回 231 个零字节，所以链路看起来完全正常。
> - 掉线的是集成板固定扫描顺序中 **靠后** 的模块（无名指是槽位 12，小拇指是槽位 16）。这是判断依据：第一个传感器把电源拉低，第二个就无法被识别。
> - 同一块集成板通过 USB 连电脑时能找到两个模块，因为电脑提供了正常的 5 V。两种连接的差别在电源，不在协议。
>
> 模块映射只在 **集成板自身上电时锁存一次**（§4.6）。所以只改好电源还不够，集成板必须完全断电，再以 5 V 重新上电才会重新扫描。运行中再并上一路电源没有作用。

## 4. 电机（DM-J4310-2EC）

以下只列出能从固件和手册文件名确认的内容。电机侧连接器的针脚定义请查电机手册 [`datasheets/motor_TWAI/DM-J4310-2EC V1.2减速电机说明书V1.2定稿.pdf`](datasheets/motor_TWAI/)。

| 项目 | 值 | 来源 |
| --- | --- | --- |
| 电机型号 | DM-J4310-2EC | 手册文件名 |
| 电源 | 24 V（独立电源） | `docs/USER_MANUAL.md`，`gripper.c` 中的额定参数注释 |
| 总线 | CAN 2.0，ESP32-C6 TWAI | `app_pins.h` |
| 波特率 | 1 Mbit/s（`MOTOR_TWAI_BITRATE_KBPS 1000`，固件注释标为待确认） | `app_config.h` |
| ESC_ID（主机 → 电机命令） | `0x001`（对应电机寄存器 0x08） | `app_config.h` |
| MST_ID（电机反馈） | `0x000`（对应电机寄存器 0x07） | `app_config.h` |

- 固件接收时严格按 MST_ID 过滤，总线上其他报文不会被误认为反馈。
- 两个 ID 必须与电机内配置的寄存器一致。

### 4.1 必须外接 CAN 收发器

ESP32-C6 集成了 **TWAI/CAN 控制器，但没有 CAN 收发器**。GPIO2（TX）和 GPIO21（RX）是逻辑电平引脚，必须经过外接的 **3.3 V CAN 收发器**（例如 **TCAN332**、**SN65HVD230**）才能接到差分总线 CAN_H / CAN_L。

```
ESP32-C6 GPIO2  (TWAI TX) ──► 收发器 TXD
ESP32-C6 GPIO21 (TWAI RX) ◄── 收发器 RXD
                              收发器 ──► CAN_H / CAN_L（120 Ω 终端电阻）
```

## 5. 主控 PCB 接口

PCB 工程在 [`pcb/`](pcb/)（Altium：`main.SchDoc`、`main2.PcbDoc`），生产文件（Gerber、钻孔、坐标、元件表）在 [`pcb/production.zip`](pcb/production.zip)。采购链接见 [BOM](bom/BOM.md)。

| 位号 | 封装 / 连接器 | 接到 | 说明 |
| --- | --- | --- | --- |
| Con1 | 24 V 输入焊盘 | 24 V 供电线 → 24 V 直流电源适配器 | 整板与电机的 24 V 输入 |
| Con2 | XT30(2+2) | DM-J4310 电机 | 2 针 24 V 电源 + 2 针 CAN_H / CAN_L，用配套 XT30(2+2) 线 |
| Sub3 | 6 针 2.54 mm | CAN 收发器模块 | TWAI_TX / TWAI_RX ↔ CAN_H / CAN_L |
| Sub1 | XIAO ESP32-C6 | 主控 | 见第 1 节 |
| Sub2 | JY901x | JY901P IMU | UART2_tx / UART2_rx，3V3 供电 |
| P1 | 24 V→5 V DC-DC 模块 | — | 板上 5 V 来源，见第 2 节警告 |
| IC1 | ME6211A33M3G-N（SOT-23） | — | 3.3 V LDO |
| connect in1 | MX1.25 4P 卧贴（MX 1.25-WS-4P） | 触觉集成板 UART 座 | 用 MX1.25 4P 连接线；**线有同向 / 反向两种，按第 3 节的引脚对应仔细核对后再买、再插** |
| JP1 | PH2.0 2P（PH2.0-WI-2P） | 触觉集成板电源（推测） | 2 针供电接口，用途以原理图为准 |
| OLED1 | PH2.0 4P 卧贴（PH2.0-WS-4P） | 外接 1.3 inch OLED（可选） | SDA / SCL + 3V3 / GND，R1、R2 为 4.7 kΩ 上拉；当前固件未驱动 |

## 6. 参考资料

- 触觉：[`datasheets/hand_sensor_uart/`](datasheets/hand_sensor_uart/)（PX-6AX GEN3 用户手册、通信盒通信协议）
- IMU：[`datasheets/imu_JY901P_uart/`](datasheets/imu_JY901P_uart/)
- 电机：[`datasheets/motor_TWAI/`](datasheets/motor_TWAI/)
