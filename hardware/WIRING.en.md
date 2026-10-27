[中文](WIRING.md) | **English**

# Wiring

A Seeed Studio XIAO ESP32-C6 is the controller. It handles USB debug/download, the tactile integration-board link, the IMU link and motor control. This page lists how the controller connects to each sub-module, for wiring, troubleshooting and maintenance.

Every pin the firmware uses is defined only in [`../firmware/main/app_pins.h`](../firmware/main/app_pins.h); baud rates, UART controller assignment and CAN parameters are in [`../firmware/main/app_config.h`](../firmware/main/app_config.h). If this page and the firmware ever disagree, the firmware wins.

## 1. Pin table (XIAO ESP32-C6)

Verified against the project schematic (`esp32_c6_xiao` / Sub1). The schematic's **UART1** is the haptic link and **UART2** is the IMU link — these are logical names, not ESP32 controller numbers.

| Schematic net | Function | XIAO pin | GPIO | ESP peripheral |
| --- | --- | --- | --- | --- |
| (USB-C) | USB debug / download | USB-C | internal (GPIO12/13) | USB Serial/JTAG |
| UART2_rx | IMU RX (XIAO ← IMU) | D0 | 0 | UART0, 9600 → 115200 |
| UART2_tx | IMU TX (XIAO → IMU) | D1 | 1 | UART0, 9600 → 115200 |
| TWAI TX | Motor CAN TX | D2 | 2 | TWAI (needs transceiver) |
| TWAI RX | Motor CAN RX | D3 | 21 | TWAI (needs transceiver) |
| SDA | OLED I2C SDA (optional, unused by current firmware) | D4 | 22 | I2C0 @ 400 kHz |
| SCL | OLED I2C SCL (optional, unused by current firmware) | D5 | 23 | I2C0 @ 400 kHz |
| UART1 (see note) | Haptic RX (XIAO ← board) | D6 | 16 | UART1 @ 921600 |
| UART1 (see note) | Haptic TX (XIAO → board) | D7 | 17 | UART1 @ 921600 |

- TX/RX are named from the XIAO's perspective (XIAO TX → peripheral RX).
- The IMU is a JY901P (on-board MCU does the fusion and calibration), routed to UART0 through the GPIO matrix. The link starts at 9600 and switches to 115200 once it is confirmed.
- The OLED is an optional 1.3 inch SH1106 display. The current firmware has dropped the OLED driver and `app_pins.h` marks D4/D5 as free.
- D8–D10 are unused.
- Note: D6/D7 are the opposite way round from the XIAO silkscreen — see section 3. The old README and the comment block in `app_pins.h` disagree on which pin carries the `UART1_tx` / `UART1_rx` net name; wire by the Function column above and the firmware defines (`PIN_HAPTIC_UART_TX = 17`, `PIN_HAPTIC_UART_RX = 16`).

Controller assignment in firmware: haptic link → `UART_NUM_1`, IMU link → `UART_NUM_0` (the C6 has two HP UART controllers; the console is on USB Serial/JTAG so both are free). Change it in `../firmware/main/app_config.h`.

## 2. Power

> ⚠️ **Never power the XIAO from USB while the 24 V→5 V DC-DC module (P1) on the PCB is also supplying it.** During development, leave the DC-DC module unsoldered: then the motor 24 V cable and the USB cable can both be connected and every rail is powered without conflict. Solder the DC-DC module only for standalone use, and from then on do not plug in USB at the same time. (Note from the schematic.)

- The XIAO's 3V3_out (pin 11) and GND (pin 12) feed the IMU; a 10 µF bulk cap sits on 3V3.
- **The haptic integration board must not be run from 3V3.** Use VBUS / 5 V — see section 3.1.
- The motor runs from its own 24 V supply — see section 4.
- Tie all grounds together.

## 3. Haptic integration board

D6/D7 go to the integration board's 4-pin UART header:

- Connector AFC12-S04DCA-00, one row of four: `pin1 TX, pin2 RX, pin3 GND, pin4 VCC`. The silkscreen staggers the labels onto two lines, but the pins are in a single row.
- **The haptic TX/RX assignment is the opposite way round from the XIAO silkscreen.** The header names the board's own pin functions and the harness joins them label to label, so D6 sits on the board's transmitter and D7 on its receiver:

  ```
  XIAO D7 (GPIO17) → board pin2 RX    so GPIO17 is the ESP's TX
  XIAO D6 (GPIO16) ← board pin1 TX    so GPIO16 is the ESP's RX
  ```

  The firmware therefore drives GPIO17 as TX and GPIO16 as RX (see `../firmware/main/app_pins.h`). Verified on hardware: with this mapping a version read returns `ADAPTER_FW02.26`; with the pair swapped the board is silent at every baud rate. If the harness is ever rewired to cross the pair, swap the two defines back.
- Tie the grounds.
- The board is full duplex and takes its own 4.5–5 V supply on the WAFER-HY2.0-2PWB connector; it is not powered from the XIAO's 3V3.
- The two sensors plug into the board's **无名指** (ring finger) and **小拇指** (little finger) connectors. The board handles the SPI side of each module, so no CS or pull-up wiring is needed on the ESP.

If the link ever goes quiet, the driver's start-up discovery falls back to a sweep of both pin orientations × six baud rates × both candidate dialects and hex-dumps whatever replies, which is how the mapping above was established.

### 3.1 The board needs 5 V, and 3V3 fails in a way that looks like a broken sensor

> §4.3 of the manual specifies **4.5–5 V** on the WAFER-HY2.0-2PWB connector. Run it from the XIAO's 3V3 rail instead and the board still boots, still answers on UART, still returns its version string, and still enumerates **one** sensor perfectly — it just silently fails to find the second one. Use VBUS / 5 V, never 3V3.
>
> What this looks like when you get it wrong, and why none of it points at the supply:
>
> - `0x0010`–`0x0013` comes back `00 10 00 00`: only the ring module is mapped.
> - The missing slot's point count (`0x0050`) reads 0 and its per-module error code (`0x0700 + n×2`) reads `0x0001` — the same code every genuinely empty slot reports. The board is not reporting a fault; it simply never saw it.
> - Reads of the missing module's data area still succeed, returning 231 zero bytes, so the link looks perfectly healthy.
> - The **later** module in the board's fixed scan order is the one that drops (ring is slot 12, little is slot 16), which is the tell: the first sensor brings the rail down far enough that the second never enumerates.
> - Over USB the same board finds both, because the PC supplies a proper 5 V. That difference between the two links is the supply, not the protocol.
>
> The map is latched **once, at the board's own power-up** (§4.6), so fixing the supply is not enough on its own — the board has to lose power completely and come back up on 5 V before it will rescan. Adding a second supply while it is still running changes nothing.

## 4. Motor (DM-J4310-2EC)

Only facts that can be confirmed from the firmware or the manual's filename are listed here. For the motor-side connector pinout, see the motor manual [`datasheets/motor_TWAI/DM-J4310-2EC V1.2减速电机说明书V1.2定稿.pdf`](datasheets/motor_TWAI/).

| Item | Value | Source |
| --- | --- | --- |
| Motor | DM-J4310-2EC | manual filename |
| Supply | 24 V (separate supply) | `docs/USER_MANUAL.md`, rated-speed comment in `gripper.c` |
| Bus | CAN 2.0 via the ESP32-C6 TWAI | `app_pins.h` |
| Bit rate | 1 Mbit/s (`MOTOR_TWAI_BITRATE_KBPS 1000`, marked "TODO: confirm" in firmware) | `app_config.h` |
| ESC_ID (host → motor commands) | `0x001` (motor register 0x08) | `app_config.h` |
| MST_ID (motor feedback) | `0x000` (motor register 0x07) | `app_config.h` |

- The firmware's RX path filters strictly on MST_ID, so foreign bus traffic can't be mis-read as feedback.
- Both IDs must match the registers configured in the motor.

### 4.1 External CAN transceiver required

The ESP32-C6 integrates a **TWAI/CAN controller but NOT a CAN transceiver**. GPIO2 (TX) and GPIO21 (RX) are logic-level pins and must go through an external **3.3 V CAN transceiver** (e.g. **TCAN332**, **SN65HVD230**) to reach the differential CAN_H / CAN_L bus.

```
ESP32-C6 GPIO2  (TWAI TX) ──► transceiver TXD
ESP32-C6 GPIO21 (TWAI RX) ◄── transceiver RXD
                              transceiver ──► CAN_H / CAN_L (120 Ω terminated)
```

## 5. Main PCB connectors

The PCB project is in [`pcb/`](pcb/) (Altium: `main.SchDoc`, `main2.PcbDoc`); production files (Gerber, drill, pick-and-place, component list) are in [`pcb/production.zip`](pcb/production.zip). Purchase links are in the [BOM](bom/BOM.en.md).

| Designator | Footprint / connector | Connects to | Notes |
| --- | --- | --- | --- |
| Con1 | 24 V input pads | 24 V supply lead → 24 V DC adapter | 24 V input for the board and motor |
| Con2 | XT30(2+2) | DM-J4310 motor | 2 pins 24 V power + 2 pins CAN_H / CAN_L; use the matching XT30(2+2) cable |
| Sub3 | 6-pin 2.54 mm | CAN transceiver module | TWAI_TX / TWAI_RX ↔ CAN_H / CAN_L |
| Sub1 | XIAO ESP32-C6 | Controller | See section 1 |
| Sub2 | JY901x | JY901P IMU | UART2_tx / UART2_rx, 3V3 supply |
| P1 | 24 V→5 V DC-DC module | — | On-board 5 V source; see the warning in section 2 |
| IC1 | ME6211A33M3G-N (SOT-23) | — | 3.3 V LDO |
| connect in1 | MX1.25 4P horizontal SMD (MX 1.25-WS-4P) | Tactile integration board UART header | Use an MX1.25 4P cable. **These cables come in same-direction and reversed versions; check the pin mapping in section 3 carefully before buying and plugging in** |
| JP1 | PH2.0 2P (PH2.0-WI-2P) | Tactile integration board power (presumed) | 2-pin supply header; the schematic is authoritative |
| OLED1 | PH2.0 4P horizontal SMD (PH2.0-WS-4P) | External 1.3 inch OLED (optional) | SDA / SCL + 3V3 / GND, R1/R2 are 4.7 kΩ pull-ups; not driven by the current firmware |

## 6. References

- Tactile: [`datasheets/hand_sensor_uart/`](datasheets/hand_sensor_uart/) (PX-6AX GEN3 user manual, comms-box protocol)
- IMU: [`datasheets/imu_JY901P_uart/`](datasheets/imu_JY901P_uart/)
- Motor: [`datasheets/motor_TWAI/`](datasheets/motor_TWAI/)
