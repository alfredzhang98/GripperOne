[中文](README.md) | **English**

# Hardware

Total material cost is about **CNY 3000 (about USD 420)**, of which the PaXini tactile sensors are about CNY 1500. The full list with purchase links is in the [BOM](bom/BOM.en.md).

## Layout

```
hardware/
├─ README.md            This document (Chinese; README.en.md is English)
├─ WIRING.md            Pin assignment, wiring, power, PCB connectors
├─ bom/
│  ├─ BOM.md            Bill of materials (images, specs, quantities, links, suggestions)
│  ├─ BOM.xlsx          Excel version of the same list (unit price / subtotal formulas, PCB parts sheet)
│  └─ img/              BOM images
├─ cad/                 SolidWorks 3D models (Git LFS)
├─ pcb/
│  ├─ gripper_control.SchDoc Main PCB schematic (Altium Designer)
│  ├─ gripper_control.PcbDoc Main PCB layout (Altium Designer)
│  └─ production.zip    Production files: Gerber, drill, pick-and-place, component list bom_electrical.xlsx
└─ datasheets/
   ├─ motor_TWAI/       DAMIAO DM-J4310-2EC manual and performance curves
   ├─ hand_sensor_uart/ PaXini PX-6AX GEN3 manuals (CN/EN), comms-box protocol, vendor examples
   ├─ tactile_gen4/     PaXini GEN4 gripper sensor specification
   └─ imu_JY901P_uart/  WitMotion JY901P documentation link
```

## Components

| Part | Model | Notes |
| --- | --- | --- |
| Gripper mechanism | Based on [RhinoV2.0_Gripper](https://github.com/Ragtime-LAB/RhinoV2.0_Gripper) | Lead screw and nut drive the linkage; underactuated fingers |
| Motor | DAMIAO DM-J4310-2EC V1.2 (10:1, 24 V) | CAN 1 Mbit/s, force-position hybrid mode |
| Controller | Seeed Studio XIAO ESP32-C6 | Plugs into the main PCB |
| Main PCB | Custom 2-layer board (`pcb/`) | 24 V input, 24 V→5 V DC-DC, 3.3 V LDO, CAN transceiver module, IMU, connectors |
| Tactile | PaXini PX6AX-GEN3-CP-M3025-Core × 2 + PX-6AX GEN3 high-speed communication board | Finger pads on both fingers, connected to the controller through the board's UART at 921600 |
| IMU | WitMotion JY901P | Soldered on the main PCB (Sub2) |
| Display (optional) | 1.3 inch SH1106 OLED | PH2.0 4P header on the PCB; not driven by the current firmware |

## 3D models (`cad/`)

The top-level assembly is `cad/gripper_full_structure_v1.SLDASM`. Other main assemblies:

| File | Contents |
| --- | --- |
| `gripper_RhinoV2.0_Assembly_v1.SLDASM` | Gripper mechanism |
| `gripper_DM-J4310-2EC-V1.1_3d_20260228_1_v1.SLDASM` | DAMIAO DM-J4310-2EC motor |
| `gripper_gripper_arm_v1.SLDASM`, `gripper_bottom_circuits_v1.SLDASM` | Main PCB and its component models |
| `gripper_tip1_v1.SLDASM` | Fingertip (with the PaXini CP-M3025 finger pad) |

The `gripper_` prefix and `_v1` suffix were added on export. Assemblies reference parts by file name, so **do not rename files individually**. Part names are in Chinese; [BOM.en.md](bom/BOM.en.md) gives the English name of each part.

The model files are stored with Git LFS. Install Git LFS before cloning:

```bash
git lfs install
git clone https://github.com/alfredzhang98/GripperOne.git
```

If you already cloned and the model files are only a few hundred bytes (LFS pointers), run `git lfs pull` inside the repository.

## Simulation model

Isaac Sim assets prepared from this CAD (OpenUSD, packaged per NVIDIA SimReady, with joints and PhysX / Newton physics) are published on Hugging Face: [AlfredZhang98/robot-gripper-assets](https://huggingface.co/datasets/AlfredZhang98/robot-gripper-assets); this gripper is `gripper_custom/` there. See the [root README](../README.en.md#simulation-assets-isaac-sim) for download instructions.

## Main PCB (`pcb/`)

- Open `gripper_control.SchDoc` and `gripper_control.PcbDoc` in Altium Designer.
- For fabrication, use the Gerber and drill files in `production.zip`; for assembly, `Pick Place for main2.txt` and `bom_electrical.xlsx`. The component list is also in the [PCB parts section of the BOM](bom/BOM.en.md#pcb-parts).
- What each connector is for: [WIRING.en.md section 5](WIRING.en.md#5-main-pcb-connectors).

> ⚠️ **Never power the XIAO from USB and from the PCB's DC-DC module at the same time.** During development, leave the DC-DC module (P1) unsoldered so the motor 24 V cable and the USB cable can both be plugged in. See [WIRING.en.md section 2](WIRING.en.md#2-power).

## Manufacturing and purchasing notes

- **Structural parts**: the bottom cover, motor housing, main housing, finger links, fingers and end cap are 3D printed in PETG (not resin: heat-set inserts do not bond well in resin). All other structural parts are CNC machined (Quanzhou); the nut links are SS304.
- **Lead screw and nut**: currently custom-made by [Fanzhan Metal Products (繁展金属制品有限公司)](https://shop213748648.taobao.com) on Taobao. Switching to standard parts is recommended for production; see the [BOM suggestions](bom/BOM.en.md#suggestions).
- **Tactile sensors**: the stock cables are too short, so the extension cables (listed in the BOM) are required. PaXini now offers a gripper-specific GEN4 sensor that can replace GEN3 (spec in `datasheets/tactile_gen4/`); the firmware implements the GEN3 protocol, so verify the protocol before switching.
- **MX1.25 4P cable between the PCB and the integration board**: sold in same-direction and reversed versions. Check the pin mapping in [WIRING.en.md section 3](WIRING.en.md#3-haptic-integration-board) carefully before buying and plugging in.
- **External display header**: PH2.0 4P horizontal SMD; do not confuse it with XH2.54 or MX1.25.
- **CAN**: the ESP32-C6 has a TWAI controller only, so the CAN transceiver module (Sub3) is required.

## Suggested assembly order

1. Print / CNC the structural parts from the BOM and press heat-set inserts into the printed parts.
2. Fit the lead screw, nut, bearings and rotor-to-lead-screw coupler, then the motor.
3. Fit the links, fingers, pins and torsion springs (one left-hand and one right-hand spring).
4. Solder the main PCB (leave the DC-DC module off during development), then plug in the XIAO, CAN module and IMU.
5. Mount the fingertip tactile pads, connect the integration board, and wire everything per [WIRING.en.md](WIRING.en.md).
6. Flash the firmware ([firmware/README.en.md](../firmware/README.en.md)) and run the first calibration per the [user manual](../docs/USER_MANUAL.en.md).
