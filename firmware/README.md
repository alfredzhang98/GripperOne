**中文** | [English](README.en.md)

# 固件（原生 ESP-IDF）

运行在 Seeed Studio **XIAO ESP32-C6** 上的原生 **ESP-IDF** 固件，不是 Arduino 工程。采用模块化结构：驱动放在 `components/`，入口在 `main/`，由一个监控状态机（supervisor）通过 FreeRTOS 队列把各模块串起来。

## 目录结构

```
firmware/
├─ CMakeLists.txt            # ESP-IDF 顶层工程（GripperOne）
├─ sdkconfig.defaults        # 目标 esp32c6、NimBLE、USB-JTAG 控制台
├─ main/
│  ├─ app_main.c             # 初始化与任务启动（只做组装）
│  ├─ app_pins.h             # 全部 GPIO 分配
│  └─ app_config.h           # 波特率、端口、功能开关、任务栈与优先级
└─ components/
   ├─ app_protocol/          # 公共类型 + 队列“总线” + 用户协议编解码
   │                         #   （app_proto.c：BLE/Wi-Fi 共用的分帧、CRC、操作码）
   ├─ board_xiao_esp32c6/    # 板级初始化（NVS、启动信息）
   ├─ haptic_uart/           # UART1 921600，轮询集成板寄存器
   ├─ imu_uart/              # UART0 先 9600 再切 115200，WIT 11 字节帧解析
   ├─ motor_twai/            # TWAI（CAN）驱动 DM-J4310，仅力位混合模式，含启动探测
   ├─ transport_uart/        # 通过原生 USB Serial/JTAG 传输 app_proto（始终开启）
   ├─ wireless_ble/          # 通过 BLE GATT 传输 app_proto（APP_ENABLE_BLE，开启）
   ├─ wireless_wifi/         # 通过 TCP 传输 app_proto：自建热点 + 可选连入路由器（APP_ENABLE_WIFI，关闭）
   ├─ gripper/               # 上电标定（电流 + 触觉）和触觉力控夹取
   └─ app_supervisor/        # INIT/IDLE/ACTIVE/ERROR/SAFE_STOP 状态机 + 夹爪周期调度
```

上位机命令协议（分帧、CRC、操作码）见 [../docs/PROTOCOL.md](../docs/PROTOCOL.md)。

## 引脚与接线

所有 GPIO 分配都在 `main/app_pins.h`。引脚表、接线，以及 TWAI 总线必需的外置 CAN 收发器，见 [../hardware/WIRING.md](../hardware/WIRING.md)。

固件中的 UART 控制器分配：触觉链路 → `UART_NUM_1`，IMU 链路 → `UART_NUM_0`。C6 有两个 HP UART 控制器，控制台走 USB Serial/JTAG，所以两个都可用。修改位置在 `main/app_config.h`。

如果触觉链路没有响应，驱动的启动探测会退回到全面扫描：两种引脚方向 × 六种波特率 × 两种候选协议，并把收到的回复以十六进制打印出来。当前的引脚映射就是这样确定的。

## 编译

```bash
# 只需执行一次；先在当前 shell 中导出 ESP-IDF 环境：
cd firmware
idf.py set-target esp32c6
idf.py build
```

## 烧录与监视

XIAO ESP32-C6 的 USB-C 口自带 USB Serial/JTAG，不需要外接烧录器。

```bash
cd firmware
idf.py -p <PORT> flash monitor      # 例如 -p COM7（Windows）或 -p /dev/ttyACM0
# 按 Ctrl-] 退出监视
```

如果识别不到开发板，先进入下载模式（按住 **BOOT**，点一下 **RESET**，松开 **BOOT**）再重试。

## 触觉传感器界面（上位机工具）

仓库根目录下的 `host/2_haptic_uart.py` 是 PySide6 编写的最小验证界面，通过 ESP32 读取两块 PaXini CP-M3025 模块：左侧为无名指，右侧为小拇指。界面显示两张 7×11 的 Fz 分布图、每个点的 Fx/Fy 剪切力箭头、合力曲线、各模块实际帧率和序号丢失情况。安装 `host/requirements.txt` 中的依赖并烧录固件后，在仓库根目录运行 `python host/2_haptic_uart.py`。

## 配置

- **引脚**：修改 `main/app_pins.h`。
- **波特率 / 端口 / 任务大小**：修改 `main/app_config.h`。
- **连接方式**：USB 始终开启；`main/app_config.h` 中的 `APP_ENABLE_BLE`（1）和 `APP_ENABLE_WIFI`（0）控制蓝牙和 Wi-Fi。几种连接可以同时工作，回复发回命令来源的那条连接，每条连接各自解锁会话。由于 BLE + Wi-Fi 超过 1 MB，分区表使用大容量单应用分区（应用区 1.5 MB）。
- **Wi-Fi**：开启后，板子自建热点（名称 `WIFI_AP_SSID_PREFIX`-XXXX，密码 `WIFI_AP_PASSWORD`，信道 `WIFI_AP_CHANNEL`，连接 192.168.4.1），在 TCP 端口 `WIFI_TCP_PORT`（5000）上提供 app_proto。也可以作为 station 连入保存在 NVS 中的网络，断开后每 5 s 重试。以上参数都在 `main/app_config.h`。
- 更多选项用 `idf.py menuconfig` 配置。

## FreeRTOS 任务

| 任务              | 组件             | 作用                                            |
| ----------------- | ---------------- | ----------------------------------------------- |
| `haptic_uart`     | haptic_uart      | 接收高速压力数据 → `haptic_q`                   |
| `imu_uart`        | imu_uart         | 接收 IMU 数据包 → `imu_q`                       |
| `motor_twai`      | motor_twai       | 发送电机命令 / 接收状态 → `motor_status_q`      |
| `usj_rx`          | transport_uart   | USB Serial/JTAG 接收，解析协议帧 → `ctrl_q`     |
| `wifi_srv`        | wireless_wifi    | TCP 服务端，解析协议帧 → `ctrl_q`（仅 `APP_ENABLE_WIFI` 开启时） |
| `supervisor`      | app_supervisor   | 状态机与安全逻辑，处理所有队列                  |

BLE 运行在 NimBLE 主机任务中（`nimble_port_freertos_init`）。

## 已实现的设备协议

实际的通信格式是从 [../hardware/datasheets/](../hardware/datasheets/) 中的厂商资料解析出来的，现已完整实现（不是桩代码）：

- **触觉（PaXini PX-6AX GEN3）**：ESP 连接的是**高速通信集成板**（921600 8N1），而不是传感器模块本身的引脚，因此使用集成板的寄存器协议（`通信盒通信协议` §4.4），而不是传感器手册 §5.4 中直连传感器的 `0x7B`/`0x79` 协议：

  | | 帧格式 |
  |---|---|
  | 请求 | `55 AA \| 00 \| func \| reg_addr(2,LE) \| num(2,LE) \| data[N] \| LRC` |
  | 应答 | `AA 55 \| 00 \| func \| reg_addr(2,LE) \| num(2,LE) \| data[M] \| LRC` |
  | 主动上报 | `AA 56 \| 00 \| frame_len(2,LE) \| total_err(1) \| data[M] \| LRC` |

  `func` 为 `0x03` 读（最多 512 B）、`0x10` 写（最多 10 B）、`0x17` 清零；LRC 是前面所有字节之和的二进制补码。每个模块对应集成板固定 28 模块顺序（§4.6）中的一个槽位：拇指、食指、中指、无名指、小拇指、手掌，每根手指依次为近节/中节/指尖/指甲。因此 CP-M3025 是所在手指的*近节*模块：无名指 = 12，小拇指 = 16。

  | | 无名指近节 (12) | 小拇指近节 (16) |
  |---|---|---|
  | 分布力 77×3 = 231 B | `0x2800` | `0x3000` |
  | 合力 3×int16 | `0x0548` | `0x0560` |
  | 点数 uint16 | `0x0048` | `0x0050` |
  | 连接位 | `0x0011` bit4 | `0x0012` bit0 |

  启动时驱动先关闭主动上报，读取版本字符串、`0x0010`–`0x0013` 处的模块映射和各槽位的点数；如果槽位为空或不是 77 点模块，会打印具体错误。之后以 80 Hz 轮询两个模块，每个模块读两次。分布力每点为 `Fx:i8, Fy:i8, Fz:u8`；6 字节合力为三个 int16，但只有低字节有值（与厂商 `Hand_UI` 的解析一致）。所有数值单位为 0.1 N/LSB。`host/2_haptic_uart.py --selftest` 会用厂商文档中的示例命令校验这些帧。
- **IMU（维特智能 JY901P，WIT 标准协议）**：上电以 9600 8N1 通信，识别后把本次上电会话切换到 115200。固定 11 字节帧（`0x55` + 类型 + 4×int16-LE + 8 位累加和校验）。传感器输出 100 Hz。supervisor 每 20 ms 把累积的样本打包成一帧 USB 遥测，正常为每秒 50 包、共 100 个样本。
- **电机（达妙 DM-J4310-2EC）**：TWAI CAN 2.0A @ 1 Mbps，仅使用力位混合模式（`0x300+ID`：p_des 为 float，v_des ×100，i_des ×10000）。启动探测读取 PMAX/VMAX/TMAX，设置 CTRL_MODE = 4 和 300 ms 通信丢失 TIMEOUT，并回读确认。反馈解码使用从电机读到的量程。失能状态下每 50 ms 发送一次失能帧，以保持反馈。

## 当前限制

- **触觉**：假定两块 CP-M3025 插在集成板的**无名指**和**小拇指**接口，作为近节模块（槽位 12 和 16）。插到其他手指接口时，集成板会在不同槽位报告它们，启动探测日志会给出提示。目前只实现了请求/应答轮询；集成板的主动上报模式（`0x0017`）在启动时被关闭，开启后帧率会更高。77 个点按行优先显示；用于控制前，先按压一个角确认实际的旋转/镜像方向。
- **IMU**：启动时只检测 9600 和 115200 两种波特率，用只读寄存器查询来识别/唤醒模块。在 9600 下找到的模块会在本次上电期间切到 115200（不保存），随后设置 ACC/GYRO/ANGLE 以 100 Hz 输出（同样不保存）。数据流中断时先唤醒一次，再在这两种波特率下重新检测。有意不提供 Yaw。
- **电机/夹爪**：2026-10-23 已在台架上验证（标定、原点检查、最高 15 rad/s 的运动、夹海绵的力控夹取）。未解决问题见 [../docs/GRIPPER_CONTROL.md](../docs/GRIPPER_CONTROL.md) 第 7 节。使用出厂 ID：ESC_ID `0x001`，MST_ID `0x000`。
- **Wi-Fi** 已实现（热点 + 可选 station，TCP 服务端），但默认关闭（`APP_ENABLE_WIFI 0`）。**BLE** 没有配对/绑定/加密。

其余待办在源码中以 `TODO:` 注释标出。

## 后续工作

1. 触觉：在硬件上核对地址 1/2 和点的排列顺序，然后确定两根手指的旋转/镜像常量。
2. IMU：启动时发送维特配置序列，使模块无论之前处于什么状态，都输出所需的数据包和频率。
3. 电机：夹口空载完成首次上电运行；检查探测日志、标定日志（实测速度、堵转力矩、k）和 `CLOSE_DIR`。
4. 完善 supervisor 安全策略（力限制、姿态限制、从 `ERROR`/`SAFE_STOP` 恢复故障）。
5. 增加 BLE 安全（绑定）。

## 编译验证状态

> ✅ **已验证。** 使用 **ESP-IDF v6.0** 针对 `esp32c6` 编译通过（`idf.py set-target esp32c6 && idf.py build`），`GripperOne.bin` 链接后应用分区剩余约 27%。导出 ESP-IDF 后，按上面的编译命令即可在本地复现。
