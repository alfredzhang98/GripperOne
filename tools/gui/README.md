# PaXini Tactile Console (Web Serial 上位机)

A single-file browser GUI for the PaXini **PX-6AX GEN3** tactile sensor
(specifically **CP-M3025-Core / PXSR-STDCP03B**, 77 sensing points) connected
through the **高速通信集成板 (HAND / 通信盒)**. It mirrors the vendor `pxsr-gen3`
tool: live 77-point force map, resultant-force scope, calibration, and CSV
capture — implemented against the 通信盒 auto-push protocol.

## Run it

Open `paxini_gui.html` **locally in Chrome or Edge** (double-click, or serve
it). Web Serial needs a secure context — `file://` works.

1. Plug the 通信盒's USB into the PC (LED green).
2. Click **Connect Serial** → pick the COM port → 921600 8N1.
3. Choose **Content** (合力+分布力 recommended) → **Start Stream**.
4. Press the sensor: points grow/heat with Fz, arrows show tangential Fxy, the
   right panel shows resultant Fx/Fy/Fz + a rolling trace.
5. With the sensor **unloaded**, click **标定 / Zero** to null it.
6. **Record CSV** → **Download CSV** to log `t, Fx, Fy, Fz, p0x…pNz`.

No hardware? It runs in **Demo mode** (simulated 77-point touch) so you can see
the UI. The hosted artifact preview is always demo-only (browsers block Web
Serial inside the sandbox).

## Protocol (通信盒 / HAND, verified against `Hand_UI.py` + the V1.0.5 manual)

- UART **921600 8N1**, no parity/flow.
- Request `55 AA | 00 | func | addr(2 LE) | len(2 LE) | data | LRC`
  - `func 0x10` = write, `0x03` = read; `LRC = (~Σ + 1) & 0xFF`.
- Enable auto-push: write reg **0x0017 = 1** → `55 AA 00 10 17 00 01 00 01 D8`
- Content select: reg **0x0016** (bit0 合力, bit1 分布力).
- Calibrate/zero: `55 AA 00 17 02 00 01 00 01 E6` (vendor recalibrate, reg 0x0002).
- Auto-push frame `AA 56 | 00 | valid_len(2 LE) | err | payload | LRC`
  - `payload_len = valid_len − 1`, total frame = `valid_len + 6`.
  - Per module: **合力** = Fx,Fy,Fz as `int16 LE × 0.1 N`; then **分布力** =
    per point `Fx,Fy(int8), Fz(uint8) × 0.1 N` (77 points × 3 B for CP-M3025).

## Point layout

The exact XY of the 77 points is in the datasheet figure (§3.2). This GUI lays
them out on a configurable **Cols × Rows** grid (default **11 × 7 = 77**); set
these to match your sensor's array if the physical arrangement differs.

## Notes

- The console talks **directly to the 通信盒**, independent of the ESP32
  firmware. To instead stream tactile data through the gripper over BLE, the
  firmware would set haptic content to distributed (`0x0016 = 0x03`) and relay
  the per-point payload via `app_proto` (today it runs resultant-only).
- Gripper/motor control lives on the ESP32 (`app_proto` over BLE); it is a
  separate connection and not part of this serial console.
