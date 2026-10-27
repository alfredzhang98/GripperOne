[中文](README.md) | **English**

# GripperOne

<p align="center"><img src="docs/media/gripper_demo.gif" width="480" alt="Gripper open/close demo"></p>
<p align="center">Open/close demo (<a href="docs/media/gripper_demo.mp4">MP4 with sound</a>)</p>
<p align="center"><a href="https://huggingface.co/datasets/AlfredZhang98/robot-gripper-assets"><img src="https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-robot--gripper--assets-FFD21E" alt="Hugging Face simulation assets"></a> <img src="https://img.shields.io/badge/Isaac%20Sim-OpenUSD%20%7C%20SimReady-76B900" alt="Isaac Sim"></p>

**GripperOne** builds on an open-source gripper and adds pressure (tactile) sensing and a complete wireless control circuit, to give the original gripper better sensing and easier control.

## Origin

- Original open-source gripper: <https://github.com/Ragtime-LAB/RhinoV2.0_Gripper>
- Motor / actuator: DAMIAO DM-J4310-2EC v1.2

## Features

- Derived from an open-source gripper design
- Two PaXini CP-M3025 tactile finger pads, with closed-loop constant-force grasping on the tactile reading
- On-board WitMotion JY901P IMU: currently reports orientation (Euler angles / quaternion); planned for vibration and shake monitoring of the gripper
- Complete wireless control: USB, Bluetooth and Wi-Fi all carry the same protocol
- Seeed Studio XIAO ESP32-C6 controller with native ESP-IDF firmware
- One can be built for about **CNY 3000 (about USD 420)**, of which the PaXini tactile sensors are about CNY 1500; see the [BOM](hardware/bom/BOM.en.md)
- Isaac Sim simulation assets (OpenUSD); see [Simulation assets](#simulation-assets-isaac-sim)

## Repository layout

```
GripperOne/
├─ hardware/                Hardware
│  ├─ bom/                  Bill of materials (BOM.md / BOM.xlsx / images)
│  ├─ cad/                  SolidWorks 3D models (Git LFS)
│  ├─ pcb/                  Main PCB schematic, layout (Altium) and production files
│  ├─ datasheets/           Vendor documents for the motor, tactile sensors, IMU
│  └─ WIRING.md             Pins, wiring, power, PCB connectors
├─ firmware/                Firmware: ESP-IDF project (XIAO ESP32-C6)
│  ├─ main/                 Entry point, pin map app_pins.h, config app_config.h
│  └─ components/           Motor, tactile, IMU, gripper, transport modules
├─ host/                    Host tools (Python)
│  ├─ 4_gripper_gui.py      Gripper GUI
│  ├─ 3_gripper_cli.py      Gripper CLI
│  ├─ 2_haptic_uart.py      Tactile sensor debugging GUI
│  └─ 1_imu_gui.py          IMU debugging GUI
└─ docs/                    User manual, communication protocol, gripper control notes
```

Every document has a Chinese (`X.md`) and an English (`X.en.md`) version.

## Documentation

| Document | Contents |
| --- | --- |
| [hardware/README.en.md](hardware/README.en.md) | Hardware overview, 3D models, manufacturing notes |
| [hardware/bom/BOM.en.md](hardware/bom/BOM.en.md) | Bill of materials (Excel: [BOM.xlsx](hardware/bom/BOM.xlsx)) |
| [hardware/WIRING.en.md](hardware/WIRING.en.md) | Pin assignment, wiring and power |
| [firmware/README.en.md](firmware/README.en.md) | Firmware structure, build and flash, configuration |
| [docs/USER_MANUAL.en.md](docs/USER_MANUAL.en.md) | User manual: connecting (USB / Bluetooth / Wi-Fi), first use, control |
| [docs/GRIPPER_CONTROL.en.md](docs/GRIPPER_CONTROL.en.md) | Gripper motor control: bench results, parameter ranges, current calculation |
| [docs/PROTOCOL.en.md](docs/PROTOCOL.en.md) | User communication protocol and gripper application-layer behavior |
| [Hugging Face dataset](https://huggingface.co/datasets/AlfredZhang98/robot-gripper-assets) | Isaac Sim simulation assets (OpenUSD / SimReady) |

## Simulation assets (Isaac Sim)

The Isaac Sim assets for this gripper are published on Hugging Face: [AlfredZhang98/robot-gripper-assets](https://huggingface.co/datasets/AlfredZhang98/robot-gripper-assets) (CC BY 4.0).

- This gripper is `gripper_custom/` (a CTAG2F90-D gripper, `gripper_ctag2f90d/`, is included for comparison).
- OpenUSD, packaged per NVIDIA SimReady: entry point `usd/gripper_custom.usda`, with geometry and physics in `usd/payloads/`. The articulation is kept (7 rigid bodies, 6 revolute joints).
- PhysX and Newton physics are stored separately and both are disabled by default; enable exactly one. The official benchmark passes 17 tests with 0 failures on each engine.
- Mass, inertia, drive and contact parameters are simulation estimates, not hardware calibration; see the dataset README.

Download:

```bash
pip install -U "huggingface_hub[cli]"
huggingface-cli download AlfredZhang98/robot-gripper-assets --repo-type dataset --include "gripper_custom/*" --local-dir robot-gripper-assets
```

## Quick start

1. Source and manufacture the parts from the [BOM](hardware/bom/BOM.en.md), assemble per the [hardware guide](hardware/README.en.md), and wire per [WIRING.en.md](hardware/WIRING.en.md).

   > ⚠️ Never power the XIAO from USB and from the PCB's DC-DC module at the same time. Leave the DC-DC module unsoldered during development; see [WIRING.en.md](hardware/WIRING.en.md#2-power).

2. Build and flash the firmware (with ESP-IDF exported in the shell):

   ```bash
   cd firmware
   idf.py set-target esp32c6
   idf.py -p <PORT> build flash
   ```

3. Install the host dependencies and open the GUI:

   ```bash
   pip install -r host/requirements.txt
   python host/4_gripper_gui.py
   ```

   Run a full calibration the first time, then a home check at every power-up; see the [user manual](docs/USER_MANUAL.en.md).

## Acknowledgements

Thanks to 芝士榴莲肥牛 <https://github.com/wEch1ng> for their support.

## Video

Douyin: [Built a robotic gripper that can "feel" for 3,000 RMB 🦾](https://v.douyin.com/5bR7jXSu7sw/)

## License

This project is derived from [RhinoV2.0_Gripper](https://github.com/Ragtime-LAB/RhinoV2.0_Gripper) and is released under the same [GNU General Public License v3.0](LICENSE).
