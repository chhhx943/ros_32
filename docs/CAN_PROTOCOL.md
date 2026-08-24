# R3X Chassis CAN Protocol V1

Status: Frozen for host codec and STM32 implementation
Frozen date: 2026-08-24
Target: RK3588/Linux host and STM32F407 chassis controller

## 1. Scope and authority

This document is the normative byte-level contract for the R3X chassis CAN V1 interface. The Linux/ROS2 codec and STM32 firmware must implement the same definitions; neither side may introduce private field layouts or reinterpret units.

The protocol carries one equivalent steering command, two rear-wheel velocity commands, command acknowledgement, controller health, rear-wheel velocity, rear-wheel position, and fault state. PWM values, encoder ticks, servo pulse widths, ROS messages, and OTA/DFU traffic are outside this CAN protocol.

Normative terms `MUST`, `MUST NOT`, `SHOULD`, and `MAY` describe requirements. Changes to identifiers, field offsets, units, safety semantics, or timing require a protocol version change or a documented backward-compatible amendment plus matching codec tests on both endpoints.

## 2. Physical and link layer

| Property | Frozen value |
|---|---|
| CAN controller | STM32 CAN1 on PA11/PA12 |
| CAN generation | Classic CAN 2.0A |
| Identifier | 11-bit standard identifier |
| Frame type | Data frame |
| DLC | 8 bytes for every V1 frame |
| Bitrate | 1,000,000 bit/s |
| Byte order | Little-endian for all multi-byte integers |
| Application CRC | None; V1 relies on the Classic CAN frame CRC |

The RK3588 SocketCAN interface MUST use 1 Mbit/s. The bus MUST have 120 ohm termination at both physical ends. A configured socket or CAN controller does not establish device health; health is established using valid feedback and heartbeat data.

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
| `0x180` | MCU -> Host | `FB_STATUS` | Heartbeat, applied command sequence, fault code |
| `0x181` | MCU -> Host | `FB_HEALTH` | Health flags and MCU device time |
| `0x182` | MCU -> Host | `FB_REAR_VELOCITY` | Left and right rear-wheel velocity feedback |
| `0x183` | MCU -> Host | `FB_REAR_LEFT_POSITION` | Left rear-wheel accumulated position |
| `0x184` | MCU -> Host | `FB_REAR_RIGHT_POSITION` | Right rear-wheel accumulated position |

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

- `SAFE_STOP` returns only through a fresh, all-zero STOP group and then enters `STOPPED`; it MUST NOT resume directly from a VELOCITY group.
- `ESTOP` requires the physical input released continuously for at least 50 ms, CAN E-stop cleared, and a fresh all-zero STOP group with `reset_fault_request=1`; recovery ends in `STOPPED`.
- Device faults may be reset only when their physical cause is gone and their fault policy permits reset.
- `calibration_invalid` cannot be cleared by an ordinary CAN reset request.

## 8. Feedback group

The MCU freezes one immutable feedback snapshot every 20 ms. Frames `0x180..0x184` from a snapshot carry the same `feedback_seq`. New measurements or state changes during transmission belong to the next snapshot.

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
| `0x0002` | `rear_encoder_fault` | Yes |
| `0x0003` | `steering_command_rejected` | No |
| `0x0004` | `command_timeout` | No |
| `0x0005` | `command_group_incomplete` | No |
| `0x0006` | `command_group_inconsistent` | No |
| `0x0007` | `command_range_invalid` | No |
| `0x0008` | `mcu_watchdog_reset` | Yes |
| `0x0009` | `motor_driver_fault` | Reserved; latched if supported |
| `0x000A` | `calibration_invalid` | Yes |
| `0xFFFF` | `unknown_device_fault` | Yes |

Only one fault code is transported in V1. Selection priority is:

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
| Absolute equivalent steering | 1000 mrad | Codec ceiling; vehicle limit must be lower and bench calibrated |
| Absolute rear-wheel velocity | 3000 mm/s | Codec ceiling; vehicle profile limit must be lower and measured |
| Command pair window | 10 ms | Frozen |
| MCU command timeout | 100 ms | Frozen |
| Feedback period | 20 ms | Frozen |

Wheel radius, encoder PPR and quadrature factor, gear ratio, encoder polarity, rear track, wheelbase, steering centre/direction/limits, servo pulse widths, and safe vehicle speed are calibration data. Missing or invalid required calibration MUST inhibit VELOCITY mode and report `0x000A`; implementations MUST NOT silently substitute experience-based defaults.

## 11. Required conformance vectors

Both host and MCU codec suites MUST use matching byte vectors for at least:

1. STOP and VELOCITY groups.
2. Forward/reverse and left/right signed targets.
3. Exact 10 ms pair-window boundary and expired partial groups.
4. Mismatched sequence and flags.
5. Invalid IDE, RTR, DLC, version, reserved bits, and reserved bytes.
6. Duplicate command sequence and `0xFFFF -> 0x0000` wrap.
7. Single-frame E-stop without `applied_command_seq` update.
8. All five feedback frames from one immutable snapshot.
9. Feedback sequence, heartbeat, and device-time wrap.
10. Invalid/stale velocity and position flags.
11. MCU timeout at 100 ms and host feedback/heartbeat timeouts.

## 12. Current STM32 implementation conformance

As of 2026-08-24, `BSP/bsp_bxcan.c` and its host tests implement the V1 identifiers, byte layouts, two-frame pairing, range ceilings, 10 ms pair window, 100 ms timeout, single-frame E-stop detection, and five feedback serializers.

The following gaps are implementation work, not protocol alternatives:

- reject duplicate `command_seq` without refreshing the watchdog;
- do not update `applied_command_seq` for a single-frame E-stop;
- enforce the full STOP-mediated SAFE_STOP/ESTOP recovery state machine;
- source feedback flags and values from real encoder/control snapshots;
- add TIM4 steering calibration and output;
- add physical PE1 E-stop and immediate hardware brake path;
- add CAN error/bus-off diagnostics and IWDG reset reporting.

Until these gaps are closed and bench-tested, passing codec and bxCAN loopback tests proves transport/format behaviour only, not complete vehicle-control conformance.

## 13. OTA separation

USB DFU firmware upgrade is a maintenance-plane protocol and MUST NOT reuse these CAN identifiers or create a second actuator owner. Before the MCU enters DFU, the vehicle must be stationary and in a latched maintenance-safe state. During DFU the motor and steering outputs must remain electrically safe, and normal CAN command acceptance is unavailable until the application has rebooted, passed self-tests, and re-established fresh feedback.
