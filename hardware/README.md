**中文** | [English](README.en.md)

# 硬件

整机物料成本约 **3000 元（约 420 美元）**，其中帕西尼触觉传感器约 1500 元，完整清单和采购链接见 [BOM](bom/BOM.md)。

## 目录

```
hardware/
├─ README.md            本文
├─ WIRING.md            引脚分配、接线、供电、PCB 接口
├─ bom/
│  ├─ BOM.md            物料清单（含样图、规格、数量、采购链接、优化建议）
│  ├─ BOM.xlsx          同一份清单的 Excel 版（含单价 / 小计公式、PCB 元件表）
│  └─ img/              BOM 样图
├─ cad/                 SolidWorks 三维模型（Git LFS）
├─ pcb/
│  ├─ gripper_control.SchDoc 主控 PCB 原理图（Altium Designer）
│  ├─ gripper_control.PcbDoc 主控 PCB 版图（Altium Designer）
│  └─ production.zip    生产文件：Gerber、钻孔、坐标文件、元件表 bom_electrical.xlsx
└─ datasheets/
   ├─ motor_TWAI/       达妙 DM-J4310-2EC 说明书和性能曲线
   ├─ hand_sensor_uart/ 帕西尼 PX-6AX GEN3 中英文手册、通信盒协议、厂家例程
   ├─ tactile_gen4/     帕西尼 GEN4 夹爪传感器规格手册
   └─ imu_JY901P_uart/  维特智能 JY901P 资料链接
```

## 组成

| 部分 | 型号 | 说明 |
| --- | --- | --- |
| 夹爪机构 | 基于 [RhinoV2.0_Gripper](https://github.com/Ragtime-LAB/RhinoV2.0_Gripper) | 丝杆螺母驱动连杆，欠驱动手指 |
| 电机 | 达妙 DM-J4310-2EC V1.2（10:1，24 V） | CAN 1 Mbit/s，力位混合模式 |
| 主控 | Seeed Studio XIAO ESP32-C6 | 插在主控 PCB 上 |
| 主控 PCB | 自制双层板（`pcb/`） | 24 V 输入、24 V→5 V DC-DC、3.3 V LDO、CAN 收发器模块、IMU、各接口 |
| 触觉 | 帕西尼 PX6AX-GEN3-CP-M3025-Core × 2 + PX-6AX GEN3 高速通信集成板 | 两指指腹，经集成板 UART 921600 接主控 |
| IMU | 维特智能 JY901P | 焊在主控 PCB 上（Sub2） |
| 显示屏（可选） | 1.3 inch SH1106 OLED | PCB 预留 PH2.0 4P 接口，当前固件未驱动 |

## 三维模型（`cad/`）

总装配体为 `cad/gripper_full_structure_v1.SLDASM`。其他主要装配体：

| 文件 | 内容 |
| --- | --- |
| `gripper_RhinoV2.0_Assembly_v1.SLDASM` | 夹爪机构 |
| `gripper_DM-J4310-2EC-V1.1_3d_20260228_1_v1.SLDASM` | 达妙 DM-J4310-2EC 电机 |
| `gripper_gripper_arm_v1.SLDASM`、`gripper_bottom_circuits_v1.SLDASM` | 主控 PCB 及其元件模型 |
| `gripper_tip1_v1.SLDASM` | 指尖（含帕西尼 CP-M3025 指腹） |

文件名中的 `gripper_` 前缀和 `_v1` 后缀是导出时加上的。装配体按文件名引用零件，**不要单独重命名文件**。零件名为中文，对应的英文名见 [BOM.en.md](bom/BOM.en.md)。

模型文件用 Git LFS 存放，克隆前先安装 Git LFS：

```bash
git lfs install
git clone https://github.com/alfredzhang98/GripperOne.git
```

已经克隆过、但模型文件只有几百字节（LFS 指针）时，在仓库里运行 `git lfs pull`。

## 仿真模型

由本 CAD 整理出的 Isaac Sim 仿真资产（OpenUSD，按 NVIDIA SimReady 规范打包，含关节和 PhysX / Newton 物理参数）发布在 Hugging Face：[AlfredZhang98/robot-gripper-assets](https://huggingface.co/datasets/AlfredZhang98/robot-gripper-assets)，本夹爪对应其中的 `gripper_custom/`。下载方法见[根目录 README](../README.md#仿真资产isaac-sim)。

## 主控 PCB（`pcb/`）

- 用 Altium Designer 打开 `gripper_control.SchDoc` 和 `gripper_control.PcbDoc`。
- 打板直接用 `production.zip` 里的 Gerber 和钻孔文件；贴片用 `Pick Place for main2.txt` 和 `bom_electrical.xlsx`。元件清单也列在 [BOM 的 PCB 元件一节](bom/BOM.md#pcb-元件)。
- 各接口的用途见 [WIRING.md 第 5 节](WIRING.md#5-主控-pcb-接口)。

> ⚠️ **XIAO 的 USB 供电和 PCB 上 DC-DC 模块的供电不能同时接入。** 开发阶段建议不焊 DC-DC 模块（P1），这样电机 24 V 电源线和 USB 线可以同时插。详见 [WIRING.md 第 2 节](WIRING.md#2-电源)。

## 加工与采购要点

- **结构件**：底盖、电机外壳、外壳、手指连杆、手指、挡盖用 PETG 3D 打印（不要用光固化，热熔螺母在光固化件里熔接效果差）；其余结构件 CNC（铨洲），螺母连杆用 304 不锈钢。
- **丝杆和丝杆螺母**：目前在淘宝 [繁展金属制品有限公司](https://shop213748648.taobao.com) 定制。后续建议改为标准件，方便量产，见 [BOM 优化建议](bom/BOM.md#优化建议)。
- **触觉传感器**：原配连接线太短，必须另买加长线（BOM 中已列）。帕西尼已推出夹爪专用的 GEN4 传感器，可以替换 GEN3（规格见 `datasheets/tactile_gen4/`）；固件目前按 GEN3 协议实现，替换前需验证通信协议。
- **PCB 与集成板之间的 MX1.25 4P 线**：分同向和反向两种，按 [WIRING.md 第 3 节](WIRING.md#3-触觉集成板) 的引脚对应仔细核对后再买、再插。
- **外接显示屏插座**：PH2.0 4P 卧贴，注意不要买成 XH2.54 或 MX1.25。
- **CAN**：ESP32-C6 只有 TWAI 控制器，必须使用 CAN 收发器模块（Sub3）。

## 装配顺序（建议）

1. 按 BOM 打印 / CNC 结构件，在打印件上压入热熔螺母。
2. 装丝杆、丝杆螺母、轴承和转子丝杆传动件，再装电机。
3. 装连杆、手指、销轴和扭簧（扭簧左旋、右旋各一个）。
4. 焊接主控 PCB（开发阶段先不焊 DC-DC 模块），插上 XIAO、CAN 模块和 IMU。
5. 装指尖触觉垫，连接集成板，接线按 [WIRING.md](WIRING.md)。
6. 烧录固件（[firmware/README.md](../firmware/README.md)），按[使用手册](../docs/USER_MANUAL.md)完成首次标定。
