# R3X Chassis CAN Protocol V1

Status: Frozen for host codec and STM32 implementation
Frozen date: 2026-08-24
Physical-layer bitrate amendment: 2026-09-06
Target: RK3588/Linux host and STM32F407 chassis controller

## 1. Scope and authority

This document is the normative byte-level contract for the R3X chassis CAN V1 interface. The Linux/ROS2 codec and STM32 firmware must implement the same definitions; neither side may introduce private field layouts or reinterpret units.

The protocol carries one equivalent steering command, two rear-wheel velocity commands, command acknowledgement, controller health, rear-wheel velocity, rear-wheel position, fault state, and the optional maintenance calibration service defined in section 14. PWM values, encoder ticks, servo pulse widths, ROS messages, and OTA/DFU traffic are outside this CAN protocol.

Normative terms `MUST`, `MUST NOT`, `SHOULD`, and `MAY` describe requirements. Changes to identifiers, field offsets, units, safety semantics, or timing require a protocol version change or a documented backward-compatible amendment plus matching codec tests on both endpoints.

## 2. Physical and link layer

| Property | Frozen value |
|---|---|
| CAN controller | STM32 CAN1 remapped to PB8/PB9 (PB8 RX, PB9 TX) |
| CAN generation | Classic CAN 2.0A |
| Identifier | 11-bit standard identifier |
| Frame type | Data frame |
| DLC | 8 bytes for every V1 frame |
| Bitrate | 500,000 bit/s |
| Byte order | Little-endian for all multi-byte integers |
| Application CRC | None; V1 relies on the Classic CAN frame CRC |

The RK3588 SocketCAN interface MUST use 500 kbit/s. The bus MUST have 120 ohm termination at both physical ends. A configured socket or CAN controller does not establish device health; health is established using valid feedback and heartbeat data.

The STM32F407 implementation uses `PCLK1/CAN kernel clock = 42 MHz` from
`SYSCLK=168 MHz` and `APB1=HCLK/4`. Its bxCAN timing is `Prescaler=7`,
`SJW=1 TQ`, `BS1=8 TQ`, and `BS2=3 TQ`: `42,000,000 / (7 * 12) =
500,000 bit/s`, with a 75% sample point. These physical-layer
parameters do not change any V1 identifier, payload, sequence, pairing, or
safety timing.

At the current maximum scheduled traffic, the normal 8-frame feedback cycle
plus the 2-frame command group is 10 frames every 20 ms. AutotuneSafe adds 11
telemetry frames, for 21 frames every 20 ms while that profile is active. Using
approximately 130 wire bits per standard 8-byte data frame (including nominal
stuffing/IFS allowance), this is about 65 kbit/s (13%) normal and 136.5 kbit/s
(27.3%) in the AutotuneSafe case at 500 kbit/s, before retries or other nodes.
These are load estimates, not a relaxation of the 10 ms pair window or 100 ms
MCU watchdog; error-passive, bus-off, termination, common-ground, and physical
waveform checks remain required on the real bus.

## 3. Common types and conventions

- `u8`, `u16`, and `u32` are unsigned integers.
- `i16` and `i32` are two's-complement signed integers.
- Angles use milliradians (`mrad`) on CAN.
- Rear-wheel linear velocities use millimetres per second (`mm/s`) on CAN.
- Rear-wheel accumulated angular positions use milliradians (`mrad`) on CAN.
- Vehicle forward motion is positive for both rear wheels.
- Equivalent steering is positive for a left turn and negative for a right turn.
- `command_seq` and `feedback_seq` wrap naturally from `0xFFFF` to `0x0000`.
- `device_time_ms` wraps naturally as a `u32` millisecond counter.
- Receivers MUST compare counters using modular arithmetic and MUST tolerate natural wrap.

Every frame starts with:

| Byte | Type | Field | Value |
|---:|---|---|---|
| 0 | `u8` | `protocol_version` | `0x01` |
| 1..2 | `u16` | sequence | `command_seq` or `feedback_seq` |

## 4. CAN identifiers

| CAN ID | Direction | Name | Purpose |
|---:|---|---|---|
| `0x120` | Host -> MCU | `CMD_STEERING` | Equivalent steering command |
| `0x121` | Host -> MCU | `CMD_REAR_WHEELS` | Left and right rear-wheel velocity commands |
| `0x122` | Host -> MCU | `CMD_CALIBRATION` | Maintenance calibration transaction service |
| `0x123` | Host -> MCU | `CMD_PID_GAINS` | Maintenance PID Kp/Ki transaction first frame |
| `0x124` | Host -> MCU | `CMD_PID_D` | Maintenance PID Kd transaction commit frame |
| `0x180` | MCU -> Host | `FB_STATUS` | Heartbeat, applied command sequence, fault code |
| `0x181` | MCU -> Host | `FB_HEALTH` | Health flags and MCU device time |
| `0x182` | MCU -> Host | `FB_REAR_VELOCITY` | Left and right rear-wheel velocity feedback |
| `0x183` | MCU -> Host | `FB_REAR_LEFT_POSITION` | Left rear-wheel accumulated position |
| `0x184` | MCU -> Host | `FB_REAR_RIGHT_POSITION` | Right rear-wheel accumulated position |
| `0x185` | MCU -> Host | `FB_CALIBRATION` | Calibration transaction response and progress |
| `0x186` | MCU -> Host | `FB_DIAGNOSTICS` | Execution state, command age, and CAN diagnostic status |
| `0x187` | MCU -> Host | `FB_PID_GAINS` | PID transaction acknowledgement |
| `0x188` | MCU -> Host | `FB_CONTROL_OUTPUT` | Actual signed wheel control output |

V1 receivers MUST ignore unrelated CAN identifiers. A frame using a known identifier but the wrong IDE, RTR, DLC, version, reserved bits, or reserved bytes is a protocol error.

## 5. Command group

`0x120` and `0x121` form one atomic command group. Except for the E-stop bypass in section 7, the MCU MUST apply neither frame independently.

### 5.1 Mode and flag byte

Both command frames carry identical `mode_flags` in byte 3.

| Bits | Name | Values |
|---:|---|---|
| 1..0 | `mode` | `0=STOP`, `1=VELOCITY`, `2=MAINTENANCE`, `3=RESERVED` |
| 2 | `estop_active` | `1` requests emergency stop |
| 3 | `safe_stop_active` | `1` requests safe stop |
| 4 | `reset_fault_request` | `1` requests permitted fault recovery |
| 7..5 | reserved | MUST be zero |

V1 normal operation accepts only `STOP` and `VELOCITY`. `MAINTENANCE` and `RESERVED` MUST NOT cause actuator motion.

Safety priority is:

```text
physical E-stop or CAN E-stop
> latched device fault
> safe-stop or MCU command timeout
> STOP
> VELOCITY
```

### 5.2 `0x120 CMD_STEERING`

| Byte | Type | Field | Requirement |
|---:|---|---|---|
| 0 | `u8` | `protocol_version` | `0x01` |
| 1..2 | `u16` | `command_seq` | Command-group sequence |
| 3 | `u8` | `mode_flags` | Section 5.1 |
| 4..5 | `i16` | `equivalent_steering_mrad` | Positive left, negative right |
| 6..7 | `u16` | reserved | MUST be zero |

The host codec converts the Ackermann equivalent steering angle from radians to `mrad`. Servo direction, centre, pulse width, rate limiting, and calibration remain MCU-owned and MUST NOT appear in host commands.

### 5.3 `0x121 CMD_REAR_WHEELS`

| Byte | Type | Field | Requirement |
|---:|---|---|---|
| 0 | `u8` | `protocol_version` | `0x01` |
| 1..2 | `u16` | `command_seq` | Same value as `0x120` |
| 3 | `u8` | `mode_flags` | Same value as `0x120` |
| 4..5 | `i16` | `rear_left_velocity_mmps` | Positive vehicle-forward direction |
| 6..7 | `i16` | `rear_right_velocity_mmps` | Positive vehicle-forward direction |

### 5.4 Atomic validation and freshness

The MCU commits a command group only when all conditions are true:

1. Both frames are standard data frames with DLC 8 and version 1.
2. Both frames have the same `command_seq` and exactly the same `mode_flags`.
3. Reserved bits and reserved bytes are zero.
4. Steering and both rear-wheel targets are inside the calibrated vehicle envelope.
5. The two frames arrive no more than 10 ms apart.
6. The mode is `STOP` or `VELOCITY`.
7. The sequence is fresh.

A duplicate `command_seq` MUST NOT refresh the MCU command watchdog, update `applied_command_seq`, or reapply actuator targets. A different sequence is fresh, including `0xFFFF -> 0x0000`.

The host SHOULD transmit a complete command group every 20 ms. The MCU command watchdog expires after 100 ms without a fresh, complete, accepted command group. Timeout forces both rear-wheel targets to zero, enters `SAFE_STOP`, and reports fault `0x0004`.

The ROS adapter's upstream command timeout is a separate policy. The current vehicle profile uses 500 ms for `/car/cmd_vel_limited` freshness; once it expires, the adapter MUST immediately begin sending fresh STOP/safe-stop groups. It MUST NOT weaken or replace the independent 100 ms MCU watchdog.

## 6. Normal command semantics

- `STOP`: both rear-wheel targets are zero. A valid STOP group may carry a permitted recovery request. Steering behaviour follows the MCU state machine and calibration policy.
- `VELOCITY`: apply all three targets as one atomic group. The MCU performs steering calibration and independent closed-loop rear-wheel control.
- `safe_stop_active=1`: both rear-wheel targets are treated as zero and the MCU enters `SAFE_STOP`, regardless of encoded wheel values.
- Out-of-envelope steering or wheel targets reject the entire group. Receivers MUST NOT clamp individual components into a different trajectory.
- The MCU does not perform Ackermann kinematics. The host computes the equivalent steering target and the two rear-wheel targets from the same motion command.

## 7. Emergency-stop bypass and recovery

Any otherwise well-formed known command frame with `estop_active=1` MUST trigger emergency braking immediately without waiting for the other command frame. This single-frame bypass:

- sets both rear-wheel targets and PWM output to zero;
- applies the TB6612 emergency BRAKE state;
- latches fault `0x0001`;
- clears pending partial command groups;
- MUST NOT update `applied_command_seq`, because no complete command group was committed.

Physical E-stop and CAN E-stop are logically ORed.

Recovery requirements are MCU state-machine requirements:

- `SAFE_STOP` returns only through a fresh, all-zero STOP group and then re-arbitrates to `STANDBY` or `CALIBRATION_REQUIRED`; it MUST NOT resume directly from a VELOCITY group.
- `ESTOP` requires the physical input released continuously for at least 50 ms, CAN E-stop cleared, and a fresh all-zero STOP group with `reset_fault_request=1`; recovery re-arbitrates to `STANDBY` or `CALIBRATION_REQUIRED`, never directly to DRIVE.
- Device faults may be reset only when their physical cause is gone and their fault policy permits reset.
- `calibration_failed` requires its explicit permitted RESET path; after clearing, missing valid calibration selects `CALIBRATION_REQUIRED` rather than DRIVE.

## 8. Feedback group

The MCU freezes one immutable feedback snapshot every 20 ms. Frames `0x180..0x184` and the optional diagnostic frame `0x186` from a snapshot carry the same `feedback_seq`. New measurements or state changes during transmission belong to the next snapshot.

### 8.1 `0x180 FB_STATUS`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `heartbeat_counter` |
| 4..5 | `u16` | `applied_command_seq` |
| 6..7 | `u16` | `fault_code` |

`heartbeat_counter` increments for every newly frozen feedback snapshot and wraps naturally. `applied_command_seq` changes only when a complete command group is actually committed.

### 8.2 `0x181 FB_HEALTH`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `status_flags` |
| 4..7 | `u32` | `device_time_ms` |

`status_flags`:

| Bit | Name | Meaning when set |
|---:|---|---|
| 0 | `command_group_accepted` | Most recent complete group was accepted |
| 1 | `left_encoder_ok` | Left encoder chain is currently valid |
| 2 | `right_encoder_ok` | Right encoder chain is currently valid |
| 3 | `steering_command_accepted` | Steering target in the most recent complete group was accepted |
| 4 | `steering_feedback_available` | Reserved capability flag; MUST remain zero for R3X V1 |
| 5 | `estop_active` | MCU is in E-stop condition |
| 6 | `safe_stop_active` | MCU is in safe-stop condition |
| 7 | `fault_latched` | At least one latched fault exists |

`steering_command_accepted` confirms command acceptance only. It does not prove that the servo or wheels reached the requested angle.

### 8.3 `0x182 FB_REAR_VELOCITY`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `velocity_flags` |
| 4..5 | `i16` | `rear_left_velocity_mmps` |
| 6..7 | `i16` | `rear_right_velocity_mmps` |

`velocity_flags bit0` validates the left value and `bit1` validates the right value. Bits 7..2 MUST be zero. A velocity is valid only when sampled for this feedback cycle; stale values MUST NOT be marked valid.

### 8.4 `0x183 FB_REAR_LEFT_POSITION`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `position_valid` |
| 4..7 | `i32` | `rear_left_position_mrad` |

`position_valid` is `0` or `1`; other values are invalid.

### 8.6 `0x186 FB_DIAGNOSTICS`

This backward-compatible, read-only extension carries execution facts that do not fit in the base health byte. It never accepts commands and does not replace `0x180 FB_STATUS` fault selection.

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `diagnostic_flags` |
| 4 | `u8` | `safety_state` |
| 5 | `u8` | `safety_action` |
| 6 | `u8` | `command_age_10ms` |
| 7 | `u8` | `can_error_class` |

`command_age_10ms` is the accepted command age in 10 ms units, truncated toward zero. `0xFF` means no fresh accepted command or an age that cannot be represented. `safety_state` and `safety_action` use the firmware enum values; unknown values MUST be treated as non-drivable.

`diagnostic_flags`:

| Bit | Name | Meaning when set |
|---:|---|---|
| 0 | `calibration_required` | No valid wheel calibration permits normal drive |
| 1 | `drive_allowed` | Safety manager currently permits wheel DRIVE |
| 2 | `command_fresh` | The accepted command group is inside the MCU watchdog window |
| 3 | `left_encoder_invalid` | Left wheel sample is not trusted in the current control step |
| 4 | `right_encoder_invalid` | Right wheel sample is not trusted in the current control step |
| 5 | `control_overrun` | The selected fault is `control_overrun` |
| 6 | `motor_stall` | The selected fault is `motor_stall` |
| 7 | `can_error` | A CAN warning/passive/bus-off/TX error has been observed since init |

`can_error_class` is `0=none`, `1=warning/protocol`, `2=error-passive`, `3=bus-off`, and `4=TX failure`. The firmware also maintains saturating 16-bit RX-invalid and TX-failure counters through its diagnostics API; they reset on MCU restart and are not packed into this eight-byte compatibility frame.

### 8.5 `0x184 FB_REAR_RIGHT_POSITION`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | `position_valid` |
| 4..7 | `i32` | `rear_right_position_mrad` |

`position_valid` is `0` or `1`; other values are invalid.

### 8.6 Host reconstruction and watchdogs

The host groups feedback by `feedback_seq` and MUST NOT combine fields from different snapshots. Arrival of a valid `0x182` permits best-effort velocity feedback and wheel-odometry input; missing `0x183` or `0x184` marks only the corresponding position invalid and MUST NOT block velocity processing.

The vehicle profile uses these host-side thresholds:

| Check | Frozen threshold | Meaning |
|---|---:|---|
| MCU feedback period | 20 ms | Nominal snapshot period, 50 Hz |
| Host feedback timeout | 100 ms | No fresh valid feedback group |
| Host heartbeat timeout | 200 ms | Heartbeat did not advance |

The host MUST reject duplicate/out-of-order feedback according to modular sequence rules, track Linux receive time separately from `device_time_ms`, and enter its safe-stop policy on timeout. Opening the CAN socket alone MUST NOT set feedback or heartbeat healthy.

## 9. Fault codes

| Code | Name | Latching policy |
|---:|---|---|
| `0x0000` | `none` | No |
| `0x0001` | `estop_active` | Yes |
| `0x0002` | `encoder_fault` | Yes |
| `0x0003` | `steering_command_rejected` | No |
| `0x0004` | `command_timeout` | No |
| `0x0005` | `command_group_incomplete` | No |
| `0x0006` | `command_group_inconsistent` | No |
| `0x0007` | `command_range_invalid` | No |
| `0x0008` | `mcu_watchdog_reset` | Yes |
| `0x0009` | `motor_driver_fault` | Reserved; latched if supported |
| `0x000A` | `calibration_failed` | Yes |
| `0x000B` | `motor_stall` | Yes |
| `0x000C` | `control_overrun` | Yes |
| `0xFFFF` | `unknown_device_fault` | Yes |

Only one fault code is transported in V1: the current highest-priority fault selected by `safety_manager`. Selection priority is:

```text
estop_active
> calibration/watchdog/encoder/motor/unknown device fault
> command_timeout
> command group/range/steering error
> none
```

The MCU MAY retain additional internal diagnostic bits, but they do not change the V1 wire layout.

## 10. Frozen limits and calibration-owned limits

Protocol storage limits are fixed by field types. Vehicle safety limits are not arbitrary codec constants; they come from validated vehicle calibration.

| Limit | Current bring-up ceiling | Freeze status |
|---|---:|---|
| Absolute equivalent steering | 600 mrad | Current MCU vehicle envelope; out-of-envelope groups are rejected |
| Absolute rear-wheel velocity | 3000 mm/s | Codec ceiling; vehicle profile limit must be lower and measured |
| Command pair window | 10 ms | Frozen |
| MCU command timeout | 100 ms | Frozen |
| Feedback period | 20 ms | Frozen |

Wheel radius, encoder PPR and quadrature factor, gear ratio, encoder polarity, rear track, wheelbase, steering centre/direction/limits, servo pulse widths, and safe vehicle speed are calibration data. Missing required calibration MUST inhibit VELOCITY mode and enter `CALIBRATION_REQUIRED`; it is not by itself a fault and MUST NOT report `0x000A`. Code `0x000A` is reserved for a completed calibration transaction that explicitly failed validation.

## 11. Required conformance vectors

Both host and MCU codec suites MUST use matching byte vectors for at least:

1. STOP and VELOCITY groups.
2. Forward/reverse and left/right signed targets.
3. Exact 10 ms pair-window boundary and expired partial groups.
4. Mismatched sequence and flags.
5. Invalid IDE, RTR, DLC, version, reserved bits, and reserved bytes.
6. Duplicate command sequence and `0xFFFF -> 0x0000` wrap.
7. Single-frame E-stop without `applied_command_seq` update.
8. All five base feedback frames and the optional diagnostic frame from one immutable snapshot.
9. Feedback sequence, heartbeat, and device-time wrap.
10. Invalid/stale velocity and position flags.
11. MCU timeout at 100 ms and host feedback/heartbeat timeouts.

## 12. Current STM32 implementation conformance

As of 2026-09-01, `BSP/bsp_bxcan.c`, `chassis_control`, and their host tests implement the V1 identifiers, byte layouts, fresh-sequence pairing, range ceilings, 10 ms pair window, 100 ms timeout, single-frame E-stop detection, five base feedback serializers plus the `0x186` diagnostics serializer, real encoder feedback snapshots, PE1 physical E-stop input, calibration gating, the `0x122`/`0x185` non-blocking calibration transaction with CAN loopback bench coverage, the `0x123`/`0x124` volatile PID maintenance transaction with `0x187` acknowledgement, and the complete minimum safety-manager evidence paths for encoder, stall, control-overrun, watchdog-reset, fault-priority, and E-stop-release handling. CAN1 is physically documented and configured on the PB8/PB9 remap.

The following items remain hardware/acceptance work, not protocol alternatives:

- verify the independently clocked IWDG timeout and reset-cause report on the target;
- validate CAN bus-off/TX-failure electrical behaviour and whether the vehicle requires a driver-disable/power-isolation input in addition to the immediate SAFE_STOP path;
- complete measured steering pulse centre/limits and wheel calibration on the real vehicle (the firmware now ramps PWM and persists the first trusted response, but PPR/gear/radius still require a measured drivetrain procedure);
- complete H4 differential-direction evidence with synchronized target L/R, actual L/R, PWM L/R and encoder delta L/R logs before any H5 ground test.

Until these gaps are closed and bench-tested, passing codec and bxCAN loopback tests proves transport/format behaviour only, not complete vehicle-control conformance.

## 13. OTA separation

USB DFU firmware upgrade is a maintenance-plane protocol and MUST NOT reuse these CAN identifiers or create a second actuator owner. Before the MCU enters DFU, the vehicle must be stationary and in a latched maintenance-safe state. During DFU the motor and steering outputs must remain electrically safe, and normal CAN command acceptance is unavailable until the application has rebooted, passed self-tests, and re-established fresh feedback.

## 14. Calibration service amendment

This is a backward-compatible V1 amendment. Legacy receivers ignore `0x122` and `0x185`; a host that uses calibration MUST implement this entire section. Both frames are standard Classic-CAN data frames with DLC 8 and byte 0 equal to `0x01`.

### 14.1 `0x122 CMD_CALIBRATION`

| Byte | Type | Field | Requirement |
|---:|---|---|---|
| 0 | `u8` | `protocol_version` | `0x01` |
| 1..2 | `u16` | `service_seq` | Calibration transaction sequence |
| 3 | `u8` | `opcode` | `START=0x01`, `CANCEL=0x02`, `QUERY=0x03` |
| 4 | `u8` | `options` | bit0 `maintenance_confirm`, bit1 `wheels_lifted_confirm`; bits7..2 zero |
| 5..6 | `u16` | `service_cookie` | `0xC35A` |
| 7 | `u8` | reserved | zero |

`service_seq` is a natural-wrapping `u16`; every value is legal. A new Start is fresh only if `(int16_t)(new_seq - last_accepted_start_seq) > 0`. Equality is duplicate, a modular distance of `0x8000` is ambiguous and rejected, and `0xFFFF -> 0x0000` is fresh. MCU restart clears the comparison baseline.

Start creates a PRECHECK transaction only when the safety policy permits calibration. Repeated Start for its active sequence is idempotent: it does not restart stages, holds, timers, or output. Query is read-only. Cancel is accepted only for the active transaction's sequence, requires `maintenance_confirm`, immediately discards pending data, and terminates the transaction. A new Start while any transaction is active is rejected as busy and must not alter the active transaction. Cancel does not permit an immediate new Start: the safety state must return to `CALIBRATION_REQUIRED` and all preconditions must be met again.

The MCU retains the most recent terminal result for 10 seconds. A duplicate Start or Query for that sequence returns the cached result without a new action. Cache expiry does not make a sequence fresh again. Calibration service traffic never refreshes the ordinary command watchdog; the host must keep transmitting fresh, complete all-zero `0x120` and `0x121` command groups while calibration is active.

### 14.2 `0x185 FB_CALIBRATION`

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `service_seq` |
| 3 | `u8` | `transaction_state` |
| 4 | `u8` | `calibration_stage` |
| 5 | `u8` | `exit_reason` |
| 6 | `u8` | `result_flags` |
| 7 | `u8` | `progress_percent` (`0..100`) |

It is emitted every 20 ms while active and once no later than the next 20 ms service period for every valid service request. Event responses carry the addressed request's `service_seq`; regular frames carry the active sequence or the cached terminal sequence. This frame is not part of the immutable `feedback_seq` snapshot group in section 8.

`transaction_state`: `NONE=0`, `PRECHECK=1`, `RUNNING=2`, `SUCCEEDED=3`, `CANCELED=4`, `ABORTED=5`, `FAILED_VALIDATION=6`, `REJECTED=7`.

`calibration_stage`: `NONE=0`, `PRECHECK=1`, `LEFT_FORWARD=2`, `LEFT_SETTLE=3`, `LEFT_REVERSE=4`, `RIGHT_FORWARD=5`, `RIGHT_SETTLE=6`, `RIGHT_REVERSE=7`, `VALIDATE=8`.

`result_flags`: bit0 `terminal`, bit1 `success`, bit2 `response_to_request`; bits7..3 zero.

`exit_reason` is frozen as follows:

| Value | Name |
|---:|---|
| `0x00` | `NONE` |
| `0x01` | `SUCCESS` |
| `0x02` | `OPERATOR_CANCEL` |
| `0x03` | `SAFETY_ESTOP` |
| `0x04` | `SAFETY_FAULT` |
| `0x05` | `COMMUNICATION_TIMEOUT` |
| `0x06` | `ENCODER_INVALID` |
| `0x07` | `PRECHECK_TIMEOUT` |
| `0x08` | `FORWARD_INSUFFICIENT_MOTION` |
| `0x09` | `FORWARD_DIRECTION_MISMATCH` |
| `0x0A` | `FORWARD_TIMEOUT` |
| `0x0B` | `SETTLE_TIMEOUT` |
| `0x0C` | `REVERSE_INSUFFICIENT_MOTION` |
| `0x0D` | `REVERSE_DIRECTION_MISMATCH` |
| `0x0E` | `REVERSE_TIMEOUT` |
| `0x0F` | `LEFT_RIGHT_RESULT_INCONSISTENT` |
| `0x10` | `TOTAL_TIMEOUT` |
| `0x11` | `REJECTED_BUSY` |
| `0x12` | `REJECTED_STALE_SEQUENCE` |
| `0x13` | `REJECTED_BAD_FORMAT` |
| `0x14` | `REJECTED_NOT_PERMITTED` |

A transport-invalid frame (wrong IDE, RTR, DLC, or version) is dropped under the normal V1 rule and has no response. A transport-valid frame with a decodable `service_seq` but invalid opcode, options, cookie, or reserved byte emits `REJECTED_BAD_FORMAT`.

### 14.3 Calibration preconditions and failure semantics

A fresh all-zero STOP requires a complete, matching-sequence `0x120 + 0x121` command group, within its pair window and command watchdog, using STOP mode with both rear targets equal to zero. A single zero-valued frame is insufficient. Both that STOP condition and credible stillness of both wheels must hold continuously for the configured hold windows before testing begins.

Only trusted encoder samples may establish stillness, movement, direction, or minimum-start-PWM response. Invalid samples break stillness holds and never count as motion. Reaching the configured invalid-sample threshold raises `encoder_fault`; it is a safety abort, not `calibration_failed`.

Only trusted evidence of insufficient motion, direction mismatch, unsuccessful effective-start detection, failed settle, or inconsistent final results latches `calibration_failed` and enters `FAULT + COAST`. Safety/operator abort, communication timeout, and encoder invalid discard pending data without that latch and eventually return to `CALIBRATION_REQUIRED` under the normal safety policy.

### 14.4 Calibration service conformance vectors

Host and MCU tests must additionally cover same-sequence Start and Query; busy new Start; Cancel and restart gate; service sequence wrap and ambiguous half-range rejection; one-sample stillness; invalid-sample interruption and escalation; mandatory settle before reverse; CAN timeout; safety/ESTOP preemption; one-wheel validation failure; partial pending discard; atomic commit; preservation of prior active calibration; and validation-timeout versus sensor-invalid-timeout classification.

## 15. PID tuning maintenance amendment

This is a backward-compatible V1 maintenance extension. Legacy receivers ignore `0x123`, `0x124`, and `0x187`; a host that uses PID tuning MUST implement this entire section. The extension changes volatile runtime gains only and MUST NOT write Flash.

### 15.1 `0x123 CMD_PID_GAINS` and `0x124 CMD_PID_D`

Both command frames are standard data frames with DLC 8 and byte 0 equal to `0x01`. Bytes 1..2 carry a wrapping `transaction_seq`, byte 3 selects the wheel (`bit0=left`, `bit1=right`, `0x03=both`). Values are unsigned Q8.8:

| ID | Byte 4..5 | Byte 6..7 |
|---:|---|---|
| `0x123` | `Kp` | `Ki` |
| `0x124` | `Kd` | little-endian commit cookie `0xC35A` |

The two frames MUST use the same sequence and axis mask and arrive no more than 10 ms apart. The MCU commits all selected gains atomically only after the `0x124` frame. The accepted range is `0 <= Kp,Ki,Kd <= 16.0`; the MCU retains its fixed `-1000..1000` control-output limit.

The transaction is accepted only when the latest ordinary command is fresh, complete, all-zero `STOP`, no CAN or physical E-stop is active, and no fault is active. A VELOCITY, SAFE_STOP, stale/partial command, E-stop, or fault context rejects the transaction and leaves the previously active gains unchanged. The effective gains are consumed by both `PID_t` instances in the existing 10 ms `chassis_control` task; this is the actual runtime application point.

### 15.2 `0x187 FB_PID_GAINS`

| Byte | Field |
|---:|---|
| 0 | `protocol_version` |
| 1..2 | `transaction_seq` |
| 3 | `status`: `0=none`, `1=accepted`, `2=rejected_format`, `3=rejected_unsafe`, `4=rejected_timeout` |
| 4 | `axis_mask` |
| 5..7 | reserved, zero |

The frame is sent with the regular feedback pump. An accepted response acknowledges volatile runtime application; it does not imply persistence across MCU reset. A tuning host MUST keep sending fresh all-zero STOP groups while changing gains and MUST issue a fresh STOP group after every experiment before the next gain transaction.

The repository command-line tool `python tools/pid/autotune.py` uses this extension when a live CAN feedback adapter is available. The host uses `python-can` and requires a configured backend/channel; this firmware has no current/voltage feedback fields, so live samples record current/voltage as `null`. The host must receive an accepted `0x187` acknowledgement and safe `0x181`/`0x186` feedback before scoring a trial.

## 16. Control-output feedback amendment

`0x188 FB_CONTROL_OUTPUT` is a read-only V1 maintenance/diagnostic extension emitted as part of the regular feedback pump. Its layout is:

| Byte | Type | Field |
|---:|---|---|
| 0 | `u8` | `protocol_version` |
| 1..2 | `u16` | `feedback_seq` |
| 3 | `u8` | validity flags: bit0 left, bit1 right; bits7..2 zero |
| 4..5 | `i16` | signed left `Motor_Drive` output (`-1000..1000`) |
| 6..7 | `i16` | signed right `Motor_Drive` output (`-1000..1000`) |

This is the actual output command applied by the MCU control task, after PID calculation and output clamping. The host logger MUST record it when the validity flags are set. Current and voltage remain unavailable until the motor driver/board exposes those measurements.
