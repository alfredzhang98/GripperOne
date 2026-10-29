[中文](GRIPPER_CONTROL.md) | **English**

# Gripper Motor Control

This document covers how to use the gripper motor control, the current bench test results, the operating parameter ranges, and how the current is calculated. For hardware wiring see [../hardware/WIRING.en.md](../hardware/WIRING.en.md); for the communication and sensor protocols see [PROTOCOL.en.md](PROTOCOL.en.md).

## 1. Components

| Part | Model / location | Role |
| --- | --- | --- |
| Motor | DAMIAO DM-J4310-2EC (10:1 reduction, 24 V) | Opens and closes the gripper; runs in force-position hybrid mode |
| Fingertip tactile pads | PaXini CP-M3025 × 2 (ring, little) | Measure grip force; range 0–25 N, safe load 50 N, output rate 83.3 Hz |
| Controller | XIAO ESP32-C6 | Controls the motor over CAN (1 Mbit/s); reads the tactile pads over UART |
| Control code | `firmware/components/gripper/`, `firmware/components/motor_twai/` | Calibration, open/close, force-controlled grip, protection |
| Host tool | `host/3_gripper_cli.py` | Command-line control and testing |

**Force-position hybrid mode:** every command frame gives the driver three values: target position, speed limit, and current limit. The driver moves toward the target position without exceeding either limit. Current is expressed in pu, where 1 pu is the driver's maximum current. For this motor, 1 pu corresponds to 19.27 N·m at the output shaft (register Kt × Imax). Command resolution is 0.0001 pu, about 0.0019 N·m.

**Position representation:** positions are given as "percent closed". 0 % is the calibrated fully open position; 100 % is the calibrated minimum (closed) position, where the two pads are 0.1 rad apart and not touching. Commands are limited to 5 %–95 %, leaving a 5 % margin at each end.

## 2. Command-Line Tool

Close `idf.py monitor` first; only one program can have the serial port open at a time. Requires pyserial; Bluetooth also requires bleak. Add `--ble` before the command to control over Bluetooth; see [USER_MANUAL.en.md](USER_MANUAL.en.md) for how to connect.

```powershell
python host/3_gripper_cli.py [--port COM6] <command> [arguments]
```

Pressing Ctrl+C while a motion command is running cuts motor power immediately. When the motion finishes, the tool exits automatically and the motor keeps holding its current position.

| Command | Action | Arguments |
| --- | --- | --- |
| `calibrate` | Full calibration; results are written to ESP32 flash | None; nothing may be between the pads |
| `home` | Verifies the stored calibration after power-up, reset, or flashing | None; nothing may be between the pads |
| `close <5–95>` | Moves to the given percent closed | `--speed v`: 0.05–15 rad/s, default 2 |
| `grip <force>` | Force-controlled grip, in N | 0.5–20 N, 0 = default 3 N; `--speed v`: approach speed |
| `stop` | Stops and holds at the current position | None |
| `disable` | Cuts motor power immediately | None |
| `clear` | Clears a latched fault | None |
| `calib` | Shows calibration data and status | None |
| `cycle` | Round-trip test: runs `home` first, then cycles between 95 % and 5 % | `--count N`, `--speed v`, `--no-home`; logs go to `host/logs/` |
| `watch [--log]` | Live display of position, torque, and both pad forces; motor not powered | None |
| `log` | Shows firmware logs only | None |
| `save-motor-config` | Writes control mode and communication timeout to motor flash | Do this once only |

**Typical sequence:**

```powershell
python host/3_gripper_cli.py calibrate        # first use, or after mechanical changes
python host/3_gripper_cli.py home             # after every power-up, reset, or flash
python host/3_gripper_cli.py close 5          # open fully
python host/3_gripper_cli.py grip 3           # grip at 3 N
python host/3_gripper_cli.py close 50 --speed 15
```

- If the driver reports 0xD (CAN communication lost) at startup, the firmware clears it once automatically. This happens when the board restarts while the motor is enabled.
- Any other fault must be cleared with `clear`.

## 3. Calibration and Home Check

**Calibration procedure (`calibrate`):**

1. Open at 0.05 rad/s; when the 0.5 s average opening torque reaches 0.36 N·m, the open end is detected.
2. Back off 0.1 rad and record this as the maximum position (0 %).
3. Zero the tactile pads.
4. Close until either pad reads 0.3 N and record this as the pad contact position; press the pads lightly to measure the current-to-grip-force coefficient k; then back off and record the minimum position (100 %).
5. Write everything to flash, and finish at 5 %.

Calibration takes about 1.5–3 minutes, depending on the starting position.

**Home check (`home`):**

- The motor's output-shaft encoder only tracks the angle within one turn, but the travel is about 7.2 rad, more than one turn, so after power-up the reading may be off by exactly one full turn.
- `home` closes slowly until the pads touch and compares the result with the stored contact position. If the error is within 0.03 rad (or one full turn plus less than 0.03 rad), all stored positions are restored and the gripper stops at 5 %. Takes about 40 s.
- If it hits an object along the way, it reports an error asking you to remove it and retry; the stored calibration is not invalidated.
- It checks the pad contact position (the closed end) and does not travel to the open end; the open end is the stored open position plus the measured difference.
- If a calibration or home check has already completed since the motor was powered on, `home` makes no check: it only moves to 5 % closed, and the current calibration stays valid (since 2026-10-29).

**Current results (bench, 2026-10-23):**

| Item | Value |
| --- | --- |
| Maximum position (0 %) | −2.0853 rad |
| Minimum position (100 %) | +5.1337 rad |
| Pad contact position | +5.2337 rad |
| Travel (maximum to minimum) | 7.2189 rad (413.6°), opening about 72 mm |
| k (current → grip force) | 1852.8 N/pu. The light-press measurement did not yield a valid fit; this is an estimate from 19.27 N·m/pu ÷ nominal lever arm 10.4 mm |
| Friction current (near contact, 0.05 rad/s) | 0.0042 pu (0.081 N·m) |
| Breakaway current | 0.0176 pu (0.339 N·m) |

## 4. Open/Close Motion

**Driver settings:** written to driver RAM at startup; they revert to factory values on power loss, so they are rewritten at every startup.

- Trapezoidal acceleration/deceleration: ACC/DEC = ±80 rad/s², factory value ±2;
- Position loop gain: KP_APR = 108, factory value 54.

**Speed:** 0.05–15 rad/s, default 2 rad/s, same for opening and closing. At an average of about 10 mm/rad, 15 rad/s is roughly 150 mm/s.

**Current limit:** raised automatically as needed.

- Starting values:
  - Opening: 1.0 N·m;
  - Closing: I = max(I_move, I_off) + 3 N ÷ k (currently about 0.37 N·m).
- When the driver is at the limit and the gripper lags the expected position but is still moving, the limit is multiplied by 1.25 every 100 ms, up to 3.0 N·m for opening and 1.0 N·m for closing.
- At the limit and no movement for 300 ms: treated as blocked; the gripper stops and reports `move blocked`.
- While closing, if the pad force rises 0.5 N above its starting value and is at least 1 N, the gripper stops.
- The starting value is the lowest reading since the current closing started, and it is kept until the next opening move:
  - a new closing command that arrives while the gripper is still closing (for example from dragging the GUI slider, about 10 commands per second) does not reset it;
  - after a stop on contact, a further closing command stops again at once instead of pressing another 0.5 N into the object per command. Use the grip command to hold an object.

**Measured (5 % ↔ 95 %, travel 6.4 rad):**

| Speed | Open to 5 % | Close to 95 % | Overshoot | Temperature |
| --- | --- | --- | --- | --- |
| 2 rad/s | 3.39 s | 3.40 s | 0 rad | — |
| 15 rad/s, firmware timing | 0.81 s | 0.88–0.89 s | 0 rad | MOS 29 °C, rotor 28 °C |
| 15 rad/s, CLI timing, includes ~0.1 s polling delay | 0.91 s | 1.00 s | — | Unchanged over 10 round trips |

50 consecutive round trips each at 10 rad/s and 15 rad/s, with no faults. The spread of the final positions is 0.001–0.002 rad, as read from the motor encoder.

## 5. Force-Controlled Grip

**Procedure:**

1. If the gripper is more closed than 80 % open, it first opens to 80 %; otherwise it closes directly from its current position.
2. Close at the approach speed, targeting the 95 % position.
3. When either pad reads 0.3 N, switch to force control.
4. If it reaches 95 % with no force: reports "no object detected" and returns to the 80 % open position.

**Force measurement:**

- The larger of the two pad readings is used; the target is reached when either side reaches it. The two pads should see equal force; a lower reading on one side is due to uneven contact or misaligned fingers.
- The tactile pads are read every 12.5 ms (80 Hz); the sensor itself outputs at 83.3 Hz.
- A reading that differs from the last trusted reading by more than 8 N is held back until the next reading confirms it. This filters out the occasional bogus 25.5 N frame that appears after tactile zeroing.

**Symbols:**

| Symbol | Meaning | Current value |
| --- | --- | --- |
| F_t | Target grip force | 0.5–20 N |
| F | Larger of the two pad readings | — |
| k | Current → grip force coefficient | 1852.8 N/pu |
| I_off, I_move | Calibrated friction current, breakaway current | 0.0042 / 0.0176 pu |
| I_f | Friction current for this grip | Peak sliding torque measured during approach ÷ 19.27 N·m/pu, not less than I_off |
| db | Half-width of the target band | max(0.25 N, 4 % × F_t) |

**Current calculation:**

| Phase | Formula | Notes |
| --- | --- | --- |
| Approach | I = max(I_move, I_off) + 2 N ÷ k, at most 0.6 N·m | Raised as needed per the open/close motion rules; if at the limit, gripper not moving for 300 ms, and no tactile force, the object is judged to be outside the pads |
| Feedforward | I_ff = I_f + F_t ÷ k | Applied immediately on contact |
| Cap | I_cap = 1.5 × I_f + min(2 F_t + 3 N, 30 N) ÷ k + ΔI | The 1.5 factor overcomes static friction; ΔI see next row; overall cap 0.06 pu (1.16 N·m) |
| Cap increase | When force is below F_t − db, pads read contact, gripper not moving, and current at the cap for 100 ms, ΔI increases by 0.03 N·m | ΔI at most 0.30 N·m |
| Integral | Increases by 2 × (F_t − F) ÷ k per second; when the gripper is not moving, by at least 20 N ÷ k per second | Current rises at most 20 N ÷ k per second |

**Stiffness K of the object between the pads (N/rad):** every correction's distance and speed are computed from K, so soft and hard objects follow the same rules.

| Item | Rule |
| --- | --- |
| Position used | The jaw position 50 ms earlier, aligned with the tactile delay |
| Start value and ceiling | The stored pad stiffness (pads pressed directly together, currently 300 N/rad). An object between the pads can only lower it; a K that is too high only makes corrections smaller and slower |
| Measurement | The larger of the secant from the contact point and the latest local slope; computed only over at least 0.008 rad of travel and 1 N of force change |
| Before a measurement | After travel Δx (≥ 0.01 rad) with a force rise of only ΔF, K ≤ (ΔF + 0.5 N) ÷ Δx, so K drops quickly for a soft object |
| Range | 3 N/rad up to the stored pad stiffness |

**Position target** (time constant T = 0.25 s, 5 × the 50 ms tactile delay):

| Force range | Action |
| --- | --- |
| F below the target band | Target (F_t − F) ÷ K ahead of the jaw, limited to 0.003–0.05 rad; speed limit (F_t − F) ÷ (K × T), limited to 0.02–1.0 rad/s |
| Within the target band | Hold at the current position. If the error still exceeds db for 300 ms, make one correction of (F_t − F) ÷ K at \|F_t − F\| ÷ (K × T); an opening correction uses at least 1.5 × I_f |
| F above the target band | Current decreases via the integral; the position target backs off at (F − F_t) ÷ (K × T) (0.01–1.0 rad/s), lagging the jaw by at most (F − F_t) ÷ K (0.003–0.3 rad). Unloading has its own current floor: it starts at 1.5 × I_f and, while the target lags by the full limit and the jaw does not move, is multiplied by 1.25 every 100 ms, up to 0.06 pu (1.16 N·m). The floor is used only while the target is more open than the jaw; if the object pushes the jaw more open than the target, the target follows the jaw instead of pulling it back. The extra current can therefore only pull the jaw open, never add grip force |
| F > min(F_t + max(1 N, 30 % × F_t), 22 N) | Current drops immediately to I_f + 0.5 F_t ÷ k |

The target band has hysteresis: the gripper stops when the force enters F_t ± db and resumes pressing or unloading only after the force leaves F_t ± 3 db. Errors between the two are removed by the single correction above. After the force stays within the target band for 150 ms, the state changes to `HOLDING`; it returns to `GRIPPING` when the force leaves F_t ± 3 db.

**Idealised simulation** (2026-10-29; linear object, no friction, no current limit, 50 ms tactile delay, starting at contact):

| K (N/rad) | 3 N: peak / within ±db | 8 N: peak / within ±db | 8 → 1 N time |
| --- | --- | --- | --- |
| 6 | 2.82 N / 1.05 s | 7.77 N / 1.50 s | 1.21 s |
| 54 | 2.82 N / 0.83 s | 7.76 N / 0.77 s | 0.58 s |
| 300 | 3.09 N / 0.33 s | 8.08 N / 0.51 s | 0.55 s |

The simulation ignores friction, the current limit and the different loading and unloading stiffness of foam. It only rules out an obviously unstable T; bench measurements decide.

**History of these changes (2026-10-29, none verified on the bench yet):**

1. Finger, 3 N target: below the band, every shortfall produced the same 0.05 rad lead at 1.0 rad/s. The finger's stiffness was about 54 N/rad, so 0.05 rad alone is about 2.7 N, and the force cycled between 2.7 and 4.8 N. Band hysteresis was added.
2. Target lowered step by step from 7 N to 0.5 N: the pads stayed at 4.9 / 4.3 N, with −0.12 N·m and −0.01 rad/s; the torque could not overcome static friction. The unload current floor was added.
3. Sponge, 8 N target: the force stayed at 7.1–7.7 N, inside the ±3 db hysteresis; on 8 → 1 N it fell only to 4.1 N in 6 s, at the fixed 0.1 rad/s walk-back (this sponge is about 6 N/rad). Corrections were made K-based and the in-band correction was added.

**Sponge test (target 3 N, 2026-10-23):**

| Approach speed | I_f / I_ff / I_cap at contact (pu) | Peak within 0.5 s | Settled force | Contact to settled |
| --- | --- | --- | --- | --- |
| 2 rad/s | 0.007 / 0.009 / 0.015 | 2.6 N | 2.90 N | 0.83 s |
| 5 rad/s | 0.011 / 0.012 / 0.021 | 4.9 N; current cut immediately after exceeding 4.0 N | 3.10 N | 1.53 s |

**Contact impact estimate:** about 8–9 N per 1 rad/s. Basis: output-shaft inertia taken as 0.0027 kg·m² (rotor 2.70e-5 kg·m² × 10², read from the driver register), pad stiffness assumed to be 300 N/rad. Hard objects have not been tested yet; an approach speed of no more than 2 rad/s is recommended.

## 6. Operating Range and Protection

| Item | Range or threshold | Basis |
| --- | --- | --- |
| Grip force | 0.5–20 N, default 3 N | 80 % of the 25 N tactile range |
| Speed | 0.05–15 rad/s, default 2 rad/s | Motor rated 12.6 rad/s, no-load 20.9 rad/s (24 V, datasheet p. 6) |
| Position | 5 %–95 % closed | 5 % margin at each end |
| Torque | Closing ≤ 1.0 N·m, grip ≤ 1.16 N·m, opening ≤ 3.0 N·m, any command ≤ 3.1 N·m | Motor rated 3.5 N·m, peak 12.5 N·m |

**Protection layers (innermost to outermost):**

| Layer | Condition | Action |
| --- | --- | --- |
| Driver | Per-frame current limit | Hardware current limiting |
| Gripper | Force exceeds min(F_t + max(1 N, 30 % × F_t), 22 N) | Cut current immediately and shed force |
| Gripper | Either pad ≥ 23 N or any single taxel ≥ 6 N, for 2 consecutive readings | Back off 0.3 rad and hold; during calibration, stop |
| Gripper | Either pad ≥ 4 N during calibration | Abort calibration and back off |
| Gripper | Tactile data lost for 300 ms | During approach or closing, stop in place; while holding, current no longer increases (stays at or below the feedforward current); after more than 10 s, force control ends and position is held |
| Supervisor | Either pad ≥ 24 N while the motor is enabled, for 2 consecutive readings | Motor powered off; requires `clear` |
| Supervisor | Driver reports a fault code | Motor powered off; requires `clear` |
| Motor driver module | No new command for 150 ms, or no feedback for 300 ms | Motor powered off; motor re-detected |
| Driver | No CAN command received for 300 ms | Driver powers off on its own and reports 0xD |

## 7. Known Issues

| Issue | Status | Basis |
| --- | --- | --- |
| Open-end position is unstable | The open end is detected at 0.36 N·m; across several calibrations the maximum position ranged from −1.31 to −2.20 rad, a difference of about 9 mm | Near the open end the torque is only 0.02–0.06 N·m below the threshold, while readings fluctuate by about ±0.05 N·m. Suggest switching to "contact position − fixed travel" |
| k not measured | Estimated from motor parameters: 1852.8 N/pu | The light-press measurement did not fit a straight line; grip force is set by the tactile closed loop, so k only affects the current cap |
| Fingers slightly non-parallel | Under light pressure the two pads read differently, e.g. ring 4.1 N, little 1.9 N | Control uses the larger of the two pad readings |
| Not yet tested | Impact when gripping hard objects, absolute force accuracy (calibration weight test), long-term durability | — |
