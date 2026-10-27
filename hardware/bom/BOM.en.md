[中文](BOM.md) | **English**

# Bill of Materials (BOM)

One gripper can be built for about **CNY 1500 (about USD 220)**. Excel version: [BOM.xlsx](BOM.xlsx); it matches this page, so update both together.

 Lead screw and nut supplier: [Fanzhan Metal Products (繁展金属制品有限公司)](https://shop213748648.taobao.com) on Taobao.

| No. | Category | Name | Image | Spec / Model | Qty | Process / Material | Source / Link | CAD file | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Structural parts | Rotor-to-lead-screw coupler | <img src="img/01_rotor_to_lead_screw_coupler.png" width="80"> |  | 1 | CNC | Quanzhou CNC (铨洲) | 转子丝杆传动件 |  |
| 2 | Structural parts | Nut link | <img src="img/02_nut_link.png" width="80"> | SS304 | 4 | CNC | Quanzhou CNC (铨洲) | 螺母连杆 | Highly loaded; use SS304 |
| 3 | Structural parts | Bottom cover | <img src="img/03_bottom_cover.png" width="80"> |  | 1 | 3D print (PETG) |  | urconnector | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 4 | Structural parts | Motor housing | <img src="img/04_motor_housing.png" width="80"> |  | 1 | 3D print (PETG) |  | motor_out | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 5 | Structural parts | Main housing | <img src="img/05_main_housing.png" width="80"> |  | 2 | 3D print (PETG) |  | 外壳 | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 6 | Structural parts | Electronics bay housing | <img src="img/06_electronics_bay_housing.png" width="80"> |  | 1 | CNC |  | upper | New in this design: holds the main PCB, side opening for USB |
| 7 | Structural parts | Driving link | <img src="img/07_driving_link.png" width="80"> |  | 2 | CNC | Quanzhou CNC (铨洲) | 主动杆 |  |
| 8 | Structural parts | Underactuated link | <img src="img/08_underactuated_link.png" width="80"> |  | 2 | CNC | Quanzhou CNC (铨洲) | 镜向欠驱动杆 |  |
| 9 | Structural parts | Underactuated link, mirrored | <img src="img/09_underactuated_link_mirrored.png" width="80"> |  | 2 | CNC | Quanzhou CNC (铨洲) | 镜向欠驱动杆-镜像 |  |
| 10 | Structural parts | Finger link | <img src="img/10_finger_link.png" width="80"> |  | 2 | 3D print (PETG) |  | 手指连杆 | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 11 | Structural parts | Driven link | <img src="img/11_driven_link.png" width="80"> |  | 2 | CNC | Quanzhou CNC (铨洲) | 从动杆 |  |
| 12 | Structural parts | Finger | <img src="img/12_finger.png" width="80"> |  | 2 | 3D print (PETG) |  | 手指v2 | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 13 | Structural parts | End cap | <img src="img/13_end_cap.png" width="80"> |  | 1 | 3D print (PETG) |  | 挡盖 | Avoid resin (SLA) prints: heat-set inserts do not bond well in resin |
| 14 | Drive train | Lead screw | <img src="img/14_lead_screw.png" width="80"> <img src="img/14_lead_screw_shop.png" width="80"> | SS304 T8×2 trapezoidal screw; the order link's variant shows "left/right-hand T8×2×200 mm + 2 nuts", please verify | 1 | Custom length / end machining | [Fanzhan Metal (Taobao)](https://item.taobao.com/item.htm?id=719659221034&skuId=5016268544734) | 丝杆 | Recommend switching to a standard part, see Suggestions |
| 15 | Drive train | Lead screw nut | <img src="img/15_lead_screw_nut.png" width="80"> | T8×2 brass nut | 1 | Brass | [Fanzhan Metal (Taobao)](https://item.taobao.com/item.htm?id=719659221034&skuId=5016268544734) | 丝杆螺母 | Same shop as the screw; recommend a standard brass flange nut |
| 16 | Bearings | Small flanged bearing | <img src="img/16_small_flanged_bearing.png" width="80"> | MF85ZZ (5×8×9.2×0.6×2.5) | 4 |  | [Taobao](https://item.taobao.com/item.htm?id=627537372468) |  |  |
| 17 | Bearings | Flanged bearing | <img src="img/17_flanged_bearing.png" width="80"> | MF128ZZ (8×12×13.6×0.8×3.5) | 4 |  | [Taobao (same shop)](https://item.taobao.com/item.htm?id=627537372468) |  |  |
| 18 | Bearings | Deep-groove ball bearing | <img src="img/18_deep_groove_ball_bearing.png" width="80"> | 6×15×5, sealed | 1 |  | [Tmall](https://detail.tmall.com/item.htm?id=735576032715) | 轴承5_15_5 | CAD file says 5×15×5 but BOM says 6×15×5; check the bore |
| 19 | Fasteners, pins & springs | Grooved pin with circlips | <img src="img/19_grooved_pin_with_circlips.png" width="80"> | M5×30 (Ø5 × 30 mm) | 6 |  | [Tmall](https://detail.tmall.com/item.htm?id=617648776883&skuId=5533509412784) |  |  |
| 20 | Fasteners, pins & springs | Grooved pin with circlips | <img src="img/19_grooved_pin_with_circlips.png" width="80"> | M5×45 (Ø5 × 45 mm) | 2 |  | [Tmall (same item)](https://detail.tmall.com/item.htm?id=617648776883) | 双头卡簧销轴m5-45 |  |
| 21 | Fasteners, pins & springs | Binding screw (Chicago screw) | <img src="img/21_binding_screw_chicago_screw.png" width="80"> | M2×14 | 4 |  | [Tmall](https://detail.tmall.com/item.htm?id=738916254976) |  | Pack of 20 sets |
| 22 | Fasteners, pins & springs | Torsion spring | <img src="img/22_torsion_spring.png" width="80"> | Wire 1.2, OD 7.5, 5 coils, 180° | 2 |  | [Tmall](https://detail.tmall.com/item.htm?id=774983364150) |  | 1 left-hand + 1 right-hand (pack of 10) |
| 23 | Fasteners, pins & springs | Socket head cap screw |  | M5×6, ISO 4762 | TBD |  |  |  |  |
| 24 | Fasteners, pins & springs | Socket head cap screw |  | M3×6, ISO 4762 | TBD |  |  |  |  |
| 25 | Fasteners, pins & springs | Socket head cap screw |  | M3×5, ISO 4762 | TBD |  |  |  |  |
| 26 | Fasteners, pins & springs | Low-profile (wafer) head screw |  | M2.5×3 | TBD |  |  |  |  |
| 27 | Fasteners, pins & springs | Socket countersunk screw |  | M3×6, ISO 10642 | TBD |  |  |  |  |
| 28 | Fasteners, pins & springs | Heat-set insert |  | TBD | TBD |  |  |  | Missing from the original BOM, needed by the printed parts |
| 29 | Actuator | Joint motor | <img src="img/29_joint_motor.png" width="80"> | Damiao DM-J4310-2EC V1.2, 10:1, 24 V | 1 |  | DAMIAO | DM-J4310-2EC-V1.1_3d | Link TBD |
| 30 | Electronics | Controller board | <img src="img/30_controller_board.png" width="80"> | Seeed Studio XIAO ESP32-C6 (PCB Sub1) | 1 |  | [Tmall](https://detail.tmall.com/item.htm?id=779487558268) |  |  |
| 31 | Electronics | CAN transceiver module | <img src="img/31_can_transceiver_module.png" width="80"> | 3.3 V, 6-pin 2.54 mm (Sub3) | 1 |  | [Tmall](https://detail.tmall.com/item.htm?id=1026039679688&skuId=6203293530777) |  | ESP32-C6 has the TWAI controller only; a transceiver is required |
| 32 | Electronics | IMU | <img src="img/32_imu.png" width="80"> | WitMotion JY901P (Sub2) | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=995918759607) |  |  |
| 33 | Electronics | 24 V→5 V DC-DC module | <img src="img/33_24_v_5_v_dc_dc_module.png" width="80"> | PCB P1 | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=908768946162) |  | Never together with XIAO USB power; leave unsoldered during development |
| 34 | Electronics | LDO regulator | <img src="img/34_ldo_regulator.png" width="80"> | ME6211A33M3G-N, 3.3 V, SOT-23 (IC1) | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=823005831436) |  |  |
| 35 | Electronics | Main PCB | <img src="img/35_main_pcb.png" width="80"> | 2-layer, production files in hardware/pcb/production.zip | 1 | PCB fab (e.g. JLCPCB) |  | gripper_arm_BOARD | Passives: see the PCB parts sheet / table |
| 36 | Electronics | XT30(2+2) socket | <img src="img/36_xt30_2_2_socket.png" width="80"> | Motor power + CAN (Con2) | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=892794165229) |  |  |
| 37 | Electronics | XT30(2+2) motor cable | <img src="img/37_xt30_2_2_motor_cable.png" width="80"> | Mates Con2, to the DM-J4310 | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=1043911293012) |  |  |
| 38 | Electronics | 24 V DC power adapter | <img src="img/38_24_v_dc_power_adapter.png" width="80"> | 24 V, DELIPPO GaN adapter (listing photo shows the 12 V model; order the 24 V variant) | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=900740020964) |  | Powers the motor and the PCB |
| 39 | Electronics | 24 V supply lead | <img src="img/39_24_v_supply_lead.png" width="80"> | Soldered to the PCB 24 V input pads (Con1) | 1 |  | [Tmall](https://detail.tmall.com/item.htm?id=927221788917) |  |  |
| 40 | Electronics | MX1.25 4P horizontal SMD header | <img src="img/40_mx1_25_4p_horizontal_smd_header.png" width="80"> | PCB → tactile board (connect in1) | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=1011820309391) |  |  |
| 41 | Electronics | MX1.25 4P cable | <img src="img/41_mx1_25_4p_cable.png" width="80"> | PCB ↔ tactile board UART | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=1037011788971) |  | Same-direction and reversed versions exist; check WIRING section 3 before buying |
| 42 | Electronics | PH2.0 4P horizontal SMD header | <img src="img/42_ph2_0_4p_horizontal_smd_header.png" width="80"> | External display (OLED1); must be PH2.0 | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=849667026616) |  | Optional |
| 43 | Electronics | 1.3 inch OLED display | <img src="img/43_1_3_inch_oled_display.png" width="80"> | SH1106, I2C | 1 |  | [Taobao](https://item.taobao.com/item.htm?id=678589112969) |  | Optional; not driven by the current firmware |
| 44 | Electronics | Tactile finger pad | <img src="img/44_tactile_finger_pad.png" width="80"> | PaXini PX6AX-GEN3-CP-M3025-Core | 2 |  | [Tmall](https://detail.tmall.com/item.htm?id=961726831317&skuId=6078300404563) | Finger pad 指腹 M3025 | PaXini now offers a gripper-specific GEN4 that can replace it (firmware implements the GEN3 protocol; verify before switching) |
| 45 | Electronics | Tactile high-speed communication board | <img src="img/45_tactile_high_speed_communication_board.png" width="80"> | PaXini PX-6AX GEN3 | 1 |  | [Tmall](https://detail.tmall.com/item.htm?id=961726831317&skuId=6243788931561) |  | Needs a 4.5–5 V supply, not 3V3 |
| 46 | Electronics | Tactile sensor extension cable | <img src="img/46_tactile_sensor_extension_cable.png" width="80"> | Sensor ↔ integration board | 2 |  | [Tmall](https://detail.tmall.com/item.htm?id=965246600488&skuId=6243521550639) |  | Stock cable is too short; required, one per sensor |

## PCB parts

From the Altium export `bom_electrical.xlsx` in [`../pcb/production.zip`](../pcb/production.zip). Passives can be sourced through JLCPCB SMT assembly or LCSC.

| Comment | Designator | Footprint | LibRef | Qty |
| --- | --- | --- | --- | --- |
| 0.1uf | C1, C2, C5, C6 | C 0603_L | C | 4 |
| 1uF | C3 | C 0603_L | C | 1 |
| 10uf | C4 | C 0603_L | C | 1 |
| 1uF | C8, C11 | C0402_L | C | 2 |
| 10uF | C9 | C0402_L | C | 1 |
| 100nF | C10 | C0402_L | C | 1 |
| 24v_input_pad | Con1 | high volt input | 24v_input_pad | 1 |
| xt30_2_2 | Con2 | xt30_2_2 | xt30_2_2 | 1 |
| HDR-1X4 | connect in1 | MX 1.25-WS-4P | Header 4 | 1 |
| ME6211A33M3G-N | IC1 | SOT95P280X145-3N | ME6211A33M3G-N | 1 |
| HDR-1X2 | JP1 | PH2.0-WI-2P | Header 2 | 1 |
| HDR-1X4 | OLED1 | PH2.0-WS-4P | Header 4 | 1 |
| DC_module | P1 | xibang_24v_dc | DC_module | 1 |
| 4.7k | R1, R2 | R 0603_L | R | 2 |
| 0(NC) | R3 | R 0603_L | R | 1 |
| esp32_c6_xiao | Sub1 | esp32_c6_xiao | esp32_c6_xiao | 1 |
| JY901x | Sub2 | JY901x | JY901x | 1 |
| can_twai | Sub3 | 6pin_2.5_can | can_twai | 1 |

The CAD file column gives the `<name>` part of `hardware/cad/gripper_<name>_v1.SLDPRT`.

## Suggestions

1. **Switch the lead screw and nut to standard parts**: The thread is already a standard T8×2; only the length and end are custom. Redesign the screw length and end to match a stock variant (standard length, plain shaft end) and use a standard brass flange nut, so both can be ordered off the shelf with no custom machining.
2. **Add fastener quantities and buy by standard number**: The original BOM lists fasteners without quantities. Count them from the assembly and order by standard number (ISO 4762 / GB/T 70.1 cap screws, ISO 10642 / GB/T 70.3 countersunk) so any supplier can fill the order.
3. **Complete the electronics**: The original BOM had mechanical parts only. Controller, CAN transceiver, tactile sensors, integration board, IMU, the PCB and its components, connectors and cables are now listed with purchase links.
4. **Strip tracking parameters from links**: The original links carried personal order-history parameters (mi_id, spm, ...). Only the item id and variant skuId are kept.
