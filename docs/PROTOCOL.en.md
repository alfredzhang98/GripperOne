[中文](PROTOCOL.md) | **English**

# User Protocol and Gripper Application Layer

This document has two parts: the **user communication protocol** between the host and the device (frame format, opcodes, telemetry, example frames, BLE transport), and the behavior of the **gripper application layer** in the firmware (self-check, calibration, open/close moves, force-controlled grip, protection layers). For how to use the gripper controls and the bench results, see [GRIPPER_CONTROL.en.md](GRIPPER_CONTROL.en.md); for hands-on operation, see [USER_MANUAL.en.md](USER_MANUAL.en.md).

## User Communication Protocol

When a host (phone / PC) controls the gripper over **BLE or Wi-Fi**, it **does not send raw bytes**. It sends structured packets with a frame header, a version number, and a CRC. BLE and Wi-Fi share **the same parser and dispatcher** ([`firmware/components/app_protocol/app_proto.c`](../firmware/components/app_protocol/app_proto.c)); commands go to the `supervisor` task, which executes them in one place and **automatically enables the relevant subsystem**. Benefits:

1. **Robust against plaintext / stray input**: random bytes cannot pass the frame header + CRC check, so they cannot move the motor by accident.
2. **Motor actions require an unlock**: dangerous commands (motor enable / target / zeroing, whole-device self-calibration) are rejected unless a session unlock command has been sent first.
3. **BLE / Wi-Fi are unified**: both wireless channels use exactly the same command set and data format, so switching channels needs no change in host logic.

### 1. Frame format (all multi-byte fields are little-endian)

```
+------+------+-----+------+-----+---------+--------------+----------+
| A5   | 5A   | VER | TYPE | SEQ | LEN(2)  | PAYLOAD[LEN] | CRC16(2) |
+------+------+-----+------+-----+---------+--------------+----------+
  SOF0   SOF1  ver.  type   seq.  payload len  payload data    checksum
```

| Field | Bytes | Description |
| --- | --- | --- |
| Header SOF | `A5 5A` | Fixed, for synchronization; the parser uses it to find the start of a frame |
| Version VER | `01` | Protocol version; frames with a mismatched version are rejected |
| Type TYPE | 1 | See the table below (command / response / telemetry / ack / error) |
| Sequence SEQ | 1 | Request sequence number; the device echoes it in the response/ACK so the host can match them |
| Length LEN | 2 | Payload byte count (little-endian); `PAYLOAD[0]` is the opcode |
| Payload PAYLOAD | LEN | First byte is the opcode, followed by the arguments |
| Checksum CRC16 | 2 | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over `VER..PAYLOAD`, stored little-endian |

#### Message type TYPE

| Value | Name | Direction | Meaning |
| --- | --- | --- | --- |
| `0x00` | COMMAND | host → device | Control command |
| `0x01` | RESPONSE | device → host | Reply to a query command (version, state) |
| `0x02` | TELEMETRY | device → host | Unsolicited telemetry stream (IMU / tactile / motor) |
| `0x03` | ACK | device → host | Whether the command was accepted (with a status code) |
| `0x04` | ERROR | device → host | Frame invalid / rejected (CRC error, version error, not unlocked, etc.) |

### 2. Opcode table (PAYLOAD[0])

> "Unlock" = executed only after a successful session unlock (`0x07`); otherwise the device replies with ACK status `0x04 (LOCKED)`.

#### System `0x0-`

| Opcode | Name | Arguments | Unlock | Description |
| --- | --- | --- | --- | --- |
| `0x01` | PING | none | no | Heartbeat / connectivity test; replies ACK OK |
| `0x02` | GET_VERSION | none | no | Query protocol version + firmware version; replies RESPONSE |
| `0x03` | GET_STATE | none | no | Query the state-machine state; replies RESPONSE |
| `0x04` | SELF_CALIBRATE | none | **yes** | Same as `GRIP_CALIBRATE` (gripper calibration, including tactile zeroing). IMU calibration needs the unit still and level; use `0x23` separately |
| `0x05` | SET_TELEM_RATE | u16 Hz | no | Set the telemetry report rate (default 20 Hz; limited by the 50 Hz control loop, values above are clamped to 50 Hz) |
| `0x06` | SAFE_STOP | none | no (safety command, always allowed) | Immediately cut motor power and enter SAFE_STOP |
| `0x07` | SESSION_UNLOCK | u32 key | no | Session unlock; key = `0x47521000`, unlocks only if correct. Each connection (USB, Bluetooth, Wi-Fi) is unlocked separately, and the unlock expires when that connection drops |
| `0x08` | SESSION_LOCK | none | no | Lock again (locks only the connection that sent this command) |
| `0x09` | WIFI_SET_STA | u8 length + SSID + u8 length + password | yes | Save the Wi-Fi network to join and connect immediately; an SSID length of 0 deletes it |
| `0x0A` | WIFI_GET_INFO | none | no | Replies RESPONSE: whether running, network status, hotspot/network/client addresses, hotspot name, network name |

#### Motor `0x1-`

The motor is driven only by the gripper layer (`firmware/components/gripper/`) in force-position hybrid mode. The old commands that drove the motor directly are retired and reply `ERR_OPCODE`: `0x10` enable would bypass the gripper's current and force limits; `0x12`/`0x13` are MIT-mode commands whose frame format differs from force-position hybrid mode; `0x14` save-zero would move the motor's position frame and invalidate the calibration.

| Opcode | Name | Arguments | Unlock | Description |
| --- | --- | --- | --- | --- |
| `0x10` | MOTOR_ENABLE | — | — | Retired |
| `0x11` | MOTOR_DISABLE | none | no | Disable the motor immediately; the gripper stops moving |
| `0x12` | MOTOR_SET_TARGET | — | — | Retired |
| `0x13` | MOTOR_SET_GAINS | — | — | Retired |
| `0x14` | MOTOR_SAVE_ZERO | — | — | Retired |
| `0x15` | MOTOR_CLEAR_ERR | none | yes | Clear the motor fault and release the SAFE_STOP latch |
| `0x16` | MOTOR_GET_STATUS | none | no | Reply immediately with one motor telemetry frame |
| `0x17` | MOTOR_SAVE_CONFIG | none | yes | Write control mode 4 and the communication timeout into the motor's own flash (sends the 0xAA store command; the motor must be disabled). The motor flash is rated for about 10,000 erase cycles; run this only once, on a new motor or after its parameters were changed |

#### IMU `0x2-`

| Opcode | Name | Arguments | Description |
| --- | --- | --- | --- |
| `0x20` | IMU_STREAM_EN | none | Start the IMU → USB/host telemetry stream (off by default at power-up; must be enabled explicitly) |
| `0x21` | IMU_STREAM_DIS | none | Stop the USB/host telemetry stream; the IMU → ESP32 link keeps sampling at the configured rate |
| `0x22` | IMU_GET | none | Return the latest complete IMU telemetry frame immediately |
| `0x23` | IMU_CALIBRATE | none | Accelerometer calibration; the module must be level and still |
| `0x25` | IMU_SET_REF | none | Save the current Roll/Pitch as the reference zero |
| `0x27` | IMU_POWON_SET | `u0=0/1` | Set IMU auto-output at power-up; default `0` on first boot, the user setting is stored in ESP NVS |
| `0x28` | IMU_SET_RATE | `u0=Hz` | Set the IMU output rate for this session: 10/20/50/100/200 Hz |
| `0x29` | IMU_ACQ_EN | none | Resume IMU → ESP32 data frames at the currently set output rate |
| `0x2A` | IMU_ACQ_DIS | none | Stop IMU → ESP32 data frames; current session only, not written to IMU flash |

#### Tactile `0x3-`

| Opcode | Name | Arguments | Description |
| --- | --- | --- | --- |
| `0x30` | HAPTIC_STREAM_EN | none | Start ESP32 → USB tactile telemetry; requires fresh data from both fingers |
| `0x31` | HAPTIC_STREAM_DIS | none | Stop USB tactile telemetry; sensor polling is unchanged |
| `0x32` | HAPTIC_GET | none | Return the latest complete point map of each finger |
| `0x34` | HAPTIC_CALIBRATE | none | Zero/tare both tactile modules with no load |
| `0x35` | HAPTIC_ACQ_EN | none | Resume sensor → ESP32 request/response polling |
| `0x36` | HAPTIC_ACQ_DIS | none | Stop polling; the USB stream switch is unchanged |

#### Gripper actions `0x4-` (all require unlock except `0x46` and `0x47`)

| Opcode | Name | Arguments | Description |
| --- | --- | --- | --- |
| `0x40` | GRIP_CALIBRATE | none | Full calibration: open to the mechanical limit (detected by current) and back off, zero the tactile pads, close until the two tactile pads touch and back off; the result is written to the ESP32 NVS, replacing the old record |
| `0x41` | GRIP_HOME | none | Home check: zero the tactile pads, close slowly until the two pads touch, and compare with the contact position saved in NVS. If the difference is ≤ 0.03 rad (or a full turn 2π plus ≤ 0.03 rad), shift and restore the saved calibration and stop at 5 % closed; if the difference is larger, report an error and keep the saved calibration (most likely something is between the pads). Without a saved calibration, run a full calibration |
| `0x42` | GRIP_SET_CLOSE | f32 [, f32] | Closing percent 5–95 (0 = calibrated maximum position, 100 = minimum position, with a 5 % margin at each end; out-of-range values move to the nearer end); optional second number is the speed (output shaft rad/s, 0.05–15; omitted or 0 means 2). Use 5 to open |
| `0x43` | GRIP_GRIP | f32 [, f32] | Constant-force grip; the argument is the target grip force (N), ≤ 0 means use the default force; range 0.5–20 N. Optional second number is the approach speed (rad/s, 0.05–15; omitted or 0 means 2). If the gripper is more closed than 80 % opening it first opens to 80 %, otherwise it closes directly from the current position; if there is still no tactile force at 95 % closed it reports "empty grasp" (no object detected) and returns to 80 % opening; once an object is detected it holds it under closed-loop force control, with the position adapting to the object; if the object is removed it also returns to 80 % opening |
| `0x45` | GRIP_SET_FORCE | f32 | Set and save the default grip force (N), range 0.5–20 N |
| `0x46` | GRIP_STOP | none | Hold the current position (no unlock needed) |
| `0x47` | GRIP_GET_CALIB | none | Replies RESPONSE: valid flag, gripper state, last fault code + 10 f32 (min/max/contact position rad, k N/pu, bias current pu, move current pu, default force N, current position rad, opening, target force N) |
| `0x48` | GRIP_MOVE_DEG | f32 angle, f32 speed | Output-shaft angle (°) relative to the minimum position, and speed (°/s) |

### 3. Telemetry must be enabled explicitly

After power-up, both IMU and tactile USB telemetry are off by default, and connecting the GUI does not start a stream either. Continuous reports to USB/host start only when the corresponding `STREAM_EN` is sent explicitly; read, set and calibrate commands never implicitly start continuous telemetry.

USB reporting and IMU → ESP32 acquisition are two independent switches: turning one off does not change the other. When USB reporting is enabled, the firmware checks whether a valid IMU sample has arrived in the last 500 ms; without fresh data it returns `ERR_STATE`. After continuous reporting is turned off, `IMU_GET` still returns one recent valid sample.

The tactile path likewise separates "CP-M3025 → ESP32 polling" from "ESP32 → USB reporting". At power-up both fingers are polled and the USB tactile stream is off; to enable the stream, both fingers must have responded successfully within the last 500 ms, otherwise `ERR_STATE` is returned. `HAPTIC_GET` returns two separate telemetry frames at once.

### 4. Telemetry data format (TYPE=`0x02`, PAYLOAD all little-endian float32)

| Telemetry | PAYLOAD layout |
| --- | --- |
| IMU (op `0x22`) | `op` + `yaw` + `pitch` + `roll` + `q0 q1 q2 q3` (7 floats total; units: degrees / unit quaternion) |
| Tactile (op `0x32`) | One module per frame: `op,index,address,status,point_count` + `seq(u32),timestamp_ms(u32)` + resultant `Fx,Fy,Fz` (3×f32) + 77×(`Fx:i8,Fy:i8,Fz:u8`); point values 0.1 N/LSB |
| Motor (op `0x16`) | `op` + `enabled(u8)` + `position` + `velocity` + `torque` (3 floats) + `error code(u8)` |

### 5. ACK / error codes (ACK and ERROR PAYLOAD = `echoed opcode` + `status code`)

| Status | Name | Meaning |
| --- | --- | --- |
| `0x00` | OK | Success |
| `0x01` | ERR_CRC | CRC check failed |
| `0x02` | ERR_VERSION | Version mismatch |
| `0x03` | ERR_OPCODE | Unknown opcode |
| `0x04` | ERR_LOCKED | Motor/self-calibration command sent without a session unlock |
| `0x05` | ERR_STATE | Not allowed in the current state-machine state (e.g. setting a target before ACTIVE) |
| `0x06` | ERR_ARG | Bad argument (e.g. wrong unlock key) |

### 6. Typical usage flow

```
1) SESSION_UNLOCK(key)       -> ACK OK        # unlock first
2) First time: GRIP_CALIBRATE (about 50 s); after every later power-up: GRIP_HOME (about 10 s); GRIP_GET_CALIB to check the result
3) GRIP_GRIP(5.0)            -> ACK OK        # grip with 5 N
4) GRIP_SET_CLOSE 5          -> ACK OK        # open
5) SAFE_STOP / MOTOR_DISABLE                  # stop immediately at any time
```

### 7. Real example frames (correct CRC, ready to send)

Host → device (COMMAND):

| Command | Frame (hex) |
| --- | --- |
| PING | `A5 5A 01 00 01 01 00 01 15 1A` |
| GET_STATE | `A5 5A 01 00 02 01 00 03 8B A1` |
| UNLOCK(0x47521000) | `A5 5A 01 00 03 05 00 07 00 10 52 47 E7 4B` |
| Start IMU stream | `A5 5A 01 00 07 01 00 20 CF 09` |
| Start tactile stream | `A5 5A 01 00 08 01 00 30 10 CF` |
| Whole-device self-calibration | `A5 5A 01 00 09 01 00 04 73 CF` |
| Set telemetry to 50 Hz | `A5 5A 01 00 0A 03 00 05 32 00 3A 96` |
| Safe stop | `A5 5A 01 00 0B 01 00 06 59 02` |

Device → host (ACK examples):

| Reply | Frame (hex) |
| --- | --- |
| ACK success (enable) | `A5 5A 01 03 04 02 00 10 00 52 E0` |
| ACK rejected, not unlocked | `A5 5A 01 03 04 02 00 10 04 D6 A0` |

### 8. BLE transport

Service / characteristics (128-bit UUIDs, distinguished by the last 4 digits):

- Service: `...0001`
- **Control characteristic** `...0002` (Write / Write-No-Response): the host writes the **command frames** above here
- **Telemetry characteristic** `...0003` (Read / Notify): the device pushes **response/telemetry frames** via Notify; Read returns the latest frame

Note: telemetry frames can exceed the BLE default 23-byte MTU (a fully populated 28-module tactile frame reaches 348 bytes, while a single Notify carries only ATT_MTU−3 bytes). The firmware handles this in two layers:

1. Request a larger MTU: `ble_att_set_preferred_mtu(APP_PROTO_MAX_FRAME + 3)` (about 412 bytes); once the central accepts it in negotiation, the largest frame fits in a single Notify;
2. **Application-level fragmentation as a fallback**: even if the central negotiates only a smaller MTU, the firmware splits one frame into several consecutive Notifies according to the current `ble_att_mtu()`.

The central (phone/PC) must therefore **treat the telemetry characteristic's Notify data as a byte stream**, concatenate it in order, and feed it to the streaming parser (app_proto), rather than treating each Notify as a complete frame. The parser reassembles frames using SOF + length + CRC, so Notify boundaries are transparent.

### 9. Security boundary (important)

The session key is a **shared constant** hard-coded in the firmware. It only blocks "casual mis-control / plaintext control"; it is **not real encryption or authentication**. For a production product, add **BLE pairing/bonding (encryption)** or a one-time provisioned **session token** on top. This protocol layer provides "structure + validation + unlock gate"; encryption must come from the link layer.

---

## Gripper Advanced Functions (application layer [`firmware/components/gripper/`](../firmware/components/gripper/))

For usage, parameter ranges and bench results, see [GRIPPER_CONTROL.en.md](GRIPPER_CONTROL.en.md).

The motor runs in DAMIAO **force-position hybrid mode** (register 0x0A = 4). Every command frame carries a target position, a speed limit, and a current limit (per-unit of the drive's maximum phase current); the drive itself keeps the current within the limit, so the grip force is bounded at all times. The supervisor calls `gripper_tick()` every 10 ms; its inputs are the motor feedback and the resultant forces of the two tactile pads.

### 1. Power-up self-check (`motor_twai`)

With the motor disabled, read PMAX/VMAX/TMAX (0x15–0x17) and decode feedback using the values read back; PMAX is accepted only within 9–25 rad. Write control mode 4 and a communication timeout of 300 ms (0x09), and read them back to confirm. None of these writes are stored in the motor flash. If the self-check fails, no motion is allowed; it retries once per second.

Protection during operation:
- Before every enable, re-read 0x0A and 0x09; if they do not match, run the self-check again (after a restart the motor returns to the mode stored in its flash).
- If feedback after enabling shows "disabled with no fault code", the motor is judged to have restarted and the self-check runs again.
- When the drive reports a fault, the motor is powered off and latched until `MOTOR_CLEAR_ERR` (CLI `clear`) is received. Exception: if the very first feedback frame after boot is 0xD (CAN communication lost), this board restarted while the motor was enabled (flashing, reset), and the drive has already stopped itself; the firmware clears it once automatically and the motor stays off. A 0xD that appears more than 1 s after boot is still latched as a fault.
- During the self-check, the drive's trapezoidal acceleration/deceleration ACC/DEC is written as ±80 rad/s² and the position-loop gain KP_APR as 108 (RAM only, rewritten at every self-check, not stored in the drive flash). The factory ACC/DEC of ±2 rad/s² limits every move to about 0.9 rad/s; with KP_APR 54 the drive approaches the target at about 11 × remaining distance rad/s, and the last ~0.5 s of every move is spent in this phase. Bench 2026-10-23, closing 5 %↔95 % (6.4 rad) at 15 rad/s: one-way 1.29–1.40 s with KP_APR 54, 0.81–0.89 s with 108; neither overshot the target.
- If the setpoint is not refreshed for 150 ms, the motor task disables the motor on its own.
- If feedback is lost for more than 300 ms, disable and rerun the self-check.
- Each rerun of the self-check increments a self-check counter; the gripper uses it to invalidate the current calibration, which then needs `GRIP_HOME` (home check) or `GRIP_CALIBRATE`.
- When feedback is lost, the TWAI node is deleted and recreated. Reason: a frame that no node acknowledges is retransmitted by the controller indefinitely; if not cleared, the motor would receive stale command frames from before the power loss when it is powered again.
- Diagnostics on self-check failure (printed on attempts 1–3 and every 10th after that): CAN controller state, TX/RX error counters, bus errors counted by type (ACK, bit, form, stuff), and frames received with other IDs. The hardware filter accepts all standard frames; the RX callback decodes only MST_ID and just records other frames. How to read them: only ACK errors = frames reach the bus but the motor does not acknowledge (check motor power, whether CANH/CANL reach the motor, reversed wiring, termination resistor); bit errors = this board's transceiver or TX/RX wiring; form/stuff errors = baud-rate mismatch; frames with other IDs = the motor's CAN_ID or Master ID differs from the configuration, and the log names the macro to change.
- `MOTOR_SAVE_CONFIG` (`0x17`) can store mode 4 and the communication timeout in the motor flash, so the motor stays in force-position hybrid mode after restarting on its own.

### 2. Calibration (triggered by `0x40`; when `GRIPPER_AUTOCAL_ON_BOOT` in `firmware/main/app_config.h` is 1, `GRIP_HOME` runs automatically once within 60 s of power-up; currently 0)

1. Disable the motor and record the baseline of both tactile pads.
2. **Find the maximum position**: open at 0.05 rad/s. The current limit starts at 0.005 pu and is multiplied by 1.15 every 150 ms while the gripper does not move; once the gripper moves, it is set to 0.45 N·m (0.36 × 1.25, converted using Kt × Imax read during the self-check; 19.27 N·m/pu for this motor). This gripper has no hard stop at the open end, and the torque needed grows the further it opens (bench: 0.28 N·m mid-stroke, 0.32 at 72 mm, 0.40 at about 77 mm, 0.51 N·m at 80 mm), so the open end is reached when the 0.5 s average of the opening torque reaches 0.36 N·m (`CAL_OPEN_TQ_NM`); a 0.1 s average reaching 0.414 N·m after 400 ms of motion also triggers immediately. Stalling under the 0.45 N·m limit means the torque is already above the threshold, so it also counts as the open end. If the threshold is reached within 0.3 rad of starting to open (the start was already beyond the open end), first close 2.5 rad and then open again. Back off 0.10 rad from the open end and record it as the maximum position. If either pad rises 0.5 N above its baseline while opening, abort and back off in reverse: `CLOSE_DIR` is reversed. If it keeps moving at less than 40 % of the commanded speed, abort with "speed mismatch". Bench 2026-10-23: 1.5 × the breakaway current (0.29 N·m) was once used as the limit, which is below the threshold; the gripper stalled mid-stroke and was misjudged as the open end, so it was changed to the fixed limit above.
3. Zero the tactile pads at the maximum position.
4. **Find the minimum position**: close. Use 0.3 rad/s farther than 0.25 rad from the last recorded contact position, and 0.05 rad/s within it (the whole way on the first calibration). 300 ms after sliding starts, compute the current needed to keep sliding from the torque feedback and set the limit to 1.5 × that plus 0.024 N·m, with the part above the sliding current not exceeding 10 N × the nominal lever (when all tactile readings are 0, this part determines the force on the pads). After that, recompute every 300 ms from the local friction, allowed to go down or up but never above the first value. Stop at the position where either pad first reads 0.3 N. Record the contact position when both pads read 0.3 N twice in a row, or one pad reads 0.3 N twice in a row and stays in place for 0.5 s. The two fingers are not perfectly parallel, and a light contact may press on the sensing points of only one pad (bench 2026-10-22: pads pressed against each other, ring 1.03 N while little 0.00 N; pressed by hand, ring 4.10 N, little 1.90 N), so contact read by only one pad is not a fault; both calibration and gripping use the larger of the two pad readings as the grip force. Then press both pads lightly at the contact position to measure the "current → grip force" coefficient k (N/pu) and the pad stiffness (N/rad):
   - First back off to 0.01 rad behind the contact position to unload;
   - Raise the current limit in steps, each step = 0.002 N·m ÷ the N·m/pu measured at the stop, rounded to the current-command resolution of 0.0001 pu (for this motor one resolution step per level, about 0.19 N on the pads);
   - Hold each step for 150 ms; once the gripper settles, take the tactile average of the last 50 ms as one point, and the log prints each point as "squeeze point";
   - Take at least 8 points in the 0.5–2.0 N range (if not enough, continue up to 3.5 N);
   - Drop the leading points where "force barely rises with current" (force held by friction after the gripper hit the pads with some speed), then fit using the median of pairwise slopes (Theil–Sen);
   - Acceptance: fit residual ≤ 0.2 N, current span ≥ 3 steps, and the lever obtained from (N·m/pu at the stop) ÷ k within 0.6–1.4 × the nominal lever. The nominal lever `GRIP_LEVER_NOMINAL_M` is currently 10.4 mm (average of 72 mm opening ÷ 6.9 rad stroke) and should be replaced by a value measured near contact. If both pads read low by the same ratio, the computed lever comes out too long and is rejected;
   - Keep the speed limit at 0.05 rad/s to limit the impact force of a sudden slip after sticking; the current limit does not exceed the starting current plus 0.2 N·m. Then back off 0.05 rad, confirm both pads are below 0.2 N, and record it as the minimum position.
5. Validate the stroke (if it differs from the last record by more than 15 %, reject once and clear the record; the next calibration is accepted). The limits, maximum, minimum, contact position, k, pad stiffness, etc. are all written to NVS. Finally open to the maximum position.

When measuring k by light pressing, if static friction is clearly larger than sliding friction, the gripper first sticks and then suddenly slips, and the points do not fit a straight line. In that case calibration still completes, k is estimated from the motor parameters instead (19.27 N·m/pu ÷ nominal lever 10.4 mm = 1852.8 N/pu), and the log prints "k not measured". The grip force is set by the tactile closed loop; k is only used to compute the current limit.

**Home check (`0x41`)**: the output-shaft encoder is single-turn absolute and the stroke of about 7.2 rad exceeds one turn, so after power-up the reading may be off by 2π as a whole. The home check zeroes the tactile pads with the motor off, then closes until the two pads touch (0.05 rad/s within 0.25 rad of the saved contact position and its ±2π positions), and compares with the saved contact position: if the difference is ≤ 0.03 rad (or a full turn 2π plus ≤ 0.03 rad), all positions are shifted by the measured difference and restored; if the difference is larger, it reports an error and keeps the saved calibration. About 40 s.

Calibration aborts if either pad exceeds 4 N, closing stalls without a tactile signal, the stroke exceeds 8.0 rad, or a timeout occurs (240 s per phase). For tactile-related faults it first backs off 0.3 rad in the opening direction (not past 0.02 rad from the mechanical limit, and during the opening phase not past this search's starting point); if the tactile force rises another 0.5 N while backing off, the motor is disabled immediately.

The stroke of this gripper is 6.9 rad (395° at the output shaft; bench: open −2.00 rad, pads touching +4.89 rad), more than one full turn. The first calibration opens and closes at 0.05 rad/s and takes about 3–4 minutes; once a record is saved, closing is slow only within 0.25 rad before the contact position. The output-shaft encoder is single-turn absolute and the stroke exceeds one turn, so if the gripper is at the closed end at power-up the reading is off by 2π as a whole; the home check detects and shifts it.

### 3. Open/close control

`GRIP_SET_CLOSE` and `GRIP_MOVE_DEG` move only after calibration and only between 5 %–95 % closed (a 5 % margin at each end; calibration and `GRIP_HOME` also finish at 5 %). The position target accelerates and decelerates at 40 rad/s² and reaches the target with zero speed; the speed can be given in the command, range 0.05–15 rad/s (the 6.6 rad from 5 % to 95 % takes about 0.82 s at 15 rad/s and about 0.91 s at 10 rad/s, limited by the 40 rad/s² acceleration), default 2 rad/s; opening and closing are the same. The current limit is not fixed: it starts at 1.0 N·m for opening and at breakaway current + 3 N / k for closing; when the drive is at the limit and the gripper lags the position target but is still moving, the motion itself needs more torque (speed, acceleration, local friction), so the limit is multiplied by 1.25 every 100 ms, up to 3.0 N·m for opening (motor rated 3.5 N·m) and up to 1.0 N·m for closing; at the limit and not moving for 300 ms, it is judged blocked, stops in place and reports "move blocked". While closing, it stops if the tactile force rises more than 0.5 N above its starting value and is at least 1 N. The impact force when closing onto an object is estimated at about 7.6 N per 1 rad/s: about 15 N at 2 rad/s, 38 N at 5 rad/s, 76 N at 10 rad/s, 114 N at 15 rad/s (the last two exceed the tactile pads' 50 N safe load) (assuming 0.002 kg·m² inertia and 300 N/rad pad stiffness, neither measured).

### 4. Constant-force grip (`GRIP_GRIP`)

1. If the gripper is more closed than 80 % opening, it first opens to 80 %; otherwise it closes directly from the current position. The closing speed is given in the command (0.05–15 rad/s, default 2 rad/s), with 40 rad/s² acceleration and deceleration, toward the 95 % closed position. During the approach the current limit = friction current + 2 N / k, raised only when the motion itself needs it; at the limit and not moving for 300 ms with no tactile force, the object is judged to be outside the pads, and it backs off and reports an error. The position target goes at most 0.10 rad past the contact position. For each contact, the peak pad force within 0.5 s and the speed at contact are logged ("grip contact: … peak …") to check the impact force.
2. Force control starts when either pad reads 0.3 N. If the position at that moment is less than 0.03 rad from the contact position, it is judged an empty grasp and returns to the minimum position.
3. Force control uses the larger of the two pads' resultant forces, in three zones: below the target band it advances and raises the current by integration; inside the target band it holds the current position; above the target band it lowers the current and backs the position target off at 0.02 rad/s to unload actively against friction. Target band width = max(0.25 N, 4 % × target force). The current ramp rate is limited to 8 N/s ÷ k; the current limit = (2 × target force + 3 N) / k, and no more than 25 N / k: k estimated from motor parameters may be too large, so this still reaches the target force even if k is twice too large; the actual grip force is set by the tactile closed loop.
4. When advancing and stalled, if the tactile force stays below 0.5 × k × (current − bias) − 1 N for 300 ms, the tactile reading is judged too low or the object is pressing outside the pads; it backs off and stops.
5. When tactile data is interrupted, the current is no longer increased and only the feed-forward current is held; regulation resumes when data returns. If the interruption lasts more than 10 s, force control ends and the object is held at the current position with the current at that time; the state becomes IDLE with fault "tactile stale while holding", and the host decides between `GRIP_SET_CLOSE 5` (open) or a new `GRIP_GRIP`.
6. 100 ms after contact is lost, it returns to slow closing. If the tactile pads are zeroed during the grip (counter changes), it backs off.
7. Object removed: a 1 s observation starts when the force drops by more than 20 % of the recent peak (and ≥ 0.5 N) within 150 ms. Drops while the force is above the target band, within 150 ms after leaving the target band, and when the target force is changed are not counted (the controller causes those drops itself). Only if, during observation, the gripper closes at least another 0.01 rad beyond where observation started and reaches within 0.03 rad of the contact position, is the space between the pads judged empty; it reports "empty grasp" and returns to the minimum position.
8. If both pads are already loaded when `GRIP_GRIP` is sent (for example still holding an object after a 10 s tactile interruption), it enters force control directly from the present current, skipping the first-contact empty-grasp check.
9. Within 2 s after an over-force back-off, a new `GRIP_GRIP` is rejected, as are `GRIP_SET_OPEN` / `GRIP_MOVE_DEG` in the closing direction.
10. On first contact, an object surface within 0.03 rad before the contact position (about 0.6 mm with a 20 mm lever) cannot be distinguished from the two pads touching directly, and is treated as an empty grasp.

### 5. Force and current protection layers

| Layer | Trigger | Response |
| --- | --- | --- |
| Drive | Current limit in each command frame | Drive hardware current limiting |
| Gripper | Resultant force above target + max(1 N, 30 %) | Current immediately drops to half the feed-forward and unloads |
| Gripper | Tactile force below 30 % of the value expected from the current | Warning printed only; regulation continues |
| Gripper | Either pad ≥ 23 N, or any sensing point ≥ 6 N (2 consecutive readings) | Back off 0.3 rad and hold (disable if calibrating) |
| supervisor | Either pad ≥ 24 N while the motor is enabled | SAFE_STOP, motor disabled, release with `0x15` |
| motor_twai | Setpoint not refreshed for 150 ms / feedback lost for 300 ms / motor registers changed | Disable motor / rerun self-check |
| Motor | No command for 300 ms | Drive disables itself |

Tactile sensor range is 0–25 N, safe load 50 N (datasheet). Target force is capped at 20 N. The 6 N single-point threshold is an estimate; the datasheet gives no per-point rating. A host tactile zero (`0x34`) is executed only when both pads have fresh data (within 200 ms), both read below 0.5 N, and the gripper is not driving the motor.

### 6. Gripper command example frames (correct CRC; all require unlock first)

| Command | Frame (hex) |
| --- | --- |
| Full calibration | `A5 5A 01 00 20 01 00 40 0A 03` |
| Home check (restore saved calibration) | `A5 5A 01 00 21 01 00 41 9F 65` |
| Close to 50 % (argument 50.0) | Send with `host/3_gripper_cli.py close 50` |
| 5 N constant-force grip | `A5 5A 01 00 23 05 00 43 00 00 A0 40 C6 EE` |
| Open / release | `A5 5A 01 00 24 01 00 44 7F 89` |
| Set default force 8 N | `A5 5A 01 00 25 05 00 45 00 00 00 41 D7 AE` |
| Hold / stop | `A5 5A 01 00 26 01 00 46 55 44` |
| Read calibration | `A5 5A 01 00 27 01 00 47 C0 22` |

Typical boot flow: `unlock → GRIP_HOME (GRIP_CALIBRATE the first time) → GRIP_GRIP / GRIP_SET_CLOSE → …`

Write the motor configuration to the motor flash (needed only once): `A5 5A 01 00 28 01 00 17 DB AC` (`MOTOR_SAVE_CONFIG`, unlock first).

Bench CLI tool `host/3_gripper_cli.py` (needs pyserial; close idf.py monitor first; see also [GRIPPER_CONTROL.en.md](GRIPPER_CONTROL.en.md)): `watch` shows the position and both pads' forces while the motor is disabled, used to confirm `CLOSE_DIR` by hand; `calibrate`, `home`, `calib`, `close 50 [--speed v]`, `grip 3 [--speed v]`, `stop`, `disable`, `clear`, `save-motor-config`. Pressing Ctrl+C while a command runs sends `MOTOR_DISABLE` immediately.
