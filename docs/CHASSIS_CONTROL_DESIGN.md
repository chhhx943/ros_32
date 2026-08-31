# R3X Chassis Closed-Loop Control Design

Status: Frozen design baseline

Frozen date: 2026-08-30

Scope: STM32F407 R3X rear-wheel chassis using TB6612FNG, TIM3 motor PWM,
TIM4 steering-servo PWM, TIM1/TIM2 encoders, TIM6 scheduling, and CAN V1.

This document is the normative architecture and behavior specification for the chassis-control implementation. Byte layouts remain normative in [CAN_PROTOCOL.md](CAN_PROTOCOL.md). Where older implementation notes conflict with this document, this document defines the intended target architecture.

## 1. Fixed hardware and units

| Item | Frozen assignment |
|---|---|
| Left motor PWM | PA6 / TIM3 CH1 |
| Right motor PWM | PA7 / TIM3 CH2 |
| Steering servo PWM | PB6 / TIM4 CH1, 50 Hz, 1 us timer tick |
| Left motor direction | PB12/PB13 |
| Right motor direction | PB14/PB15 |
| Left encoder | PE9/PE11 / TIM1 encoder mode |
| Right encoder | PA0/PA1 / TIM2 encoder mode |
| Control timebase | TIM6, 100 Hz monotonically increasing tick |
| Motor | MG513xP28, 12 V, nominal output approximately 300 rpm |

Encoder constants have one meaning only:

```c
ENCODER_PPR_PER_CHANNEL = 500;  /* motor shaft, one encoder channel */
QUADRATURE_FACTOR      = 4;
GEAR_RATIO             = 28;
COUNTS_PER_WHEEL_REV   = 56000; /* 500 * 4 * 28 */
WHEEL_RADIUS_MM        = 33.25;
WHEEL_CIRCUMFERENCE_MM = 208.915; /* 2 * pi * 33.25 */
```

`500` is not a post-quadrature CPR value. No component may apply another factor of four.

The steering servo uses a 20 ms period and a 1000--2000 us pulse envelope, with
1500 us as the initial neutral command. `equivalent_steering_mrad > 0` is the
left-turn command and is converted to a bounded servo pulse by the MCU-owned
servo BSP. The centre, direction, linkage limits, and Ackermann geometry remain
vehicle calibration data and must be verified mechanically before DRIVE testing.

## 2. Exclusive ownership and data flow

```text
encoder -> wheel_control ------>\
                                 > chassis_control -> bsp_motor -> TIM3/GPIO
wheel_calibration ------------->/
               ^                         ^
               |                         |
          safety_manager -------- CAN command / health snapshot
```

| Module | Sole responsibility | Must not do |
|---|---|---|
| `encoder` | Counter differences, 64-bit accumulated counts, trusted sample and speed facts | PWM, direction GPIO, fault policy |
| `wheel_control` | Left/right PI PWM recommendations | Call `Motor_*`, retain output while disabled |
| `wheel_calibration` | Non-blocking calibration state, PWM recommendation, pending result | Call `Motor_*`, write active calibration, clear a fault |
| `safety_manager` | Vehicle state, latched faults, driver permission and final execution mode | Touch PWM/GPIO |
| `chassis_control` | Single arbitration point for PI/calibration/COAST/BRAKE and the only caller of `Motor_*` | Invent safety policy |
| `bsp_motor` | TIM3 and TB6612 electrical operation | State or calibration policy |

No second actuator owner is permitted. A transition into any non-DRIVE action supersedes all previously computed PI or calibration recommendations.

## 3. TB6612 execution semantics

| Logical action | TB6612 pins | Meaning |
|---|---|---|
| `Motor_Drive()` | Direction by sign, PWM duty by magnitude, `STBY=1` | Requested drive torque |
| `Motor_Coast()` | `IN1=0`, `IN2=0`, `PWM=1`, `STBY=1` | High-impedance coast |
| `Motor_Brake()` | `IN1=1`, `IN2=1`, `STBY=1` | Short brake |
| Standby | `STBY=0` | Driver standby only; not normal coast |

Writing hardware PWM compare to zero is not the implementation of `Motor_Coast()`. A calibration recommendation of zero means zero torque intent; `chassis_control` resolves it to `Motor_Coast()`.

## 4. Safety state machine

Normal states are `BOOT`, `CALIBRATION_REQUIRED`, `CALIBRATION`, `STANDBY`, and `DRIVE`. Exceptional conditions preempt normal transitions in this order only:

```text
ESTOP > FAULT > SAFE_STOP > normal state machine
```

| Condition/state | Final action | Recovery rule |
|---|---|---|
| `BOOT` | COAST | Complete hardware/health checks, then normal re-arbitration |
| no valid calibration | COAST in `CALIBRATION_REQUIRED` | Complete calibration; this alone is not a fault |
| `CALIBRATION` | Recommendation is owned by calibration but executed only by chassis arbitration | ESTOP/FAULT immediately discard recommendation and pending result |
| fresh, normal all-zero STOP in DRIVE | `STANDBY + COAST` | A later fresh nonzero command may enter DRIVE if permitted |
| CAN timeout / communication degradation | `SAFE_STOP + COAST` | Requires fresh all-zero STOP, then re-arbitration; nonzero command cannot recover directly |
| ESTOP | `ESTOP + BRAKE` | Physical/CAN cause clear, explicit RESET, then re-arbitration |
| `CONTROL_OVERRUN` | `FAULT + BRAKE` | Latching recovery policy and RESET, then re-arbitration |
| `ENCODER_FAULT` | `FAULT + COAST` | Latching recovery policy and RESET, then re-arbitration |
| `MOTOR_STALL` | `FAULT + COAST` | Latching recovery policy and RESET, then re-arbitration |
| `CALIBRATION_FAILED` | `FAULT + COAST` | Explicit RESET; without valid calibration, result is `CALIBRATION_REQUIRED` |
| watchdog reset discovered after MCU reboot | `FAULT` with initialized safe output | Software makes no claim about the interval while the MCU was stopped; hardware defaults/watchdog provide that protection |

RESET clears only faults whose physical cause is gone and whose recovery condition is satisfied. It never restores DRIVE directly: remaining fault stays FAULT; invalid calibration selects `CALIBRATION_REQUIRED`; otherwise selection is STANDBY. Leaving DRIVE always invalidates targets and previous PWM and clears both PI controllers.

CAN feedback transports only the fault currently selected by `safety_manager`; lower-priority diagnostic evidence remains internal. No module other than `safety_manager` selects a fault code, and no module other than `chassis_control` turns the selected execution mode into a motor action.

## 5. Scheduler, encoder, and PI

TIM6 ISR increments only a monotonic tick. The main loop observes elapsed ticks, samples and controls at most once, and uses measured elapsed time as `dt`. It must never replay missed iterations.

- `elapsed_tick > 1`: record overrun evidence and run exactly once with real `dt`.
- `dt > CONTROL_MAX_DT`: latch `CONTROL_OVERRUN`.
- Encoder counter wrap validity is independent of scheduler overrun.
- TIM1 modulo delta is unambiguous only when `abs(delta) < 32768`; TIM2 only when `abs(delta) < 2^31`.
- A trusted sample must also satisfy `abs(delta) <= max_physical_delta(dt)`, derived from maximum trusted wheel speed, margin, and real `dt`.
- Each encoder maintains `int64_t` accumulated counts. No counter-overflow interrupt is used for position extension.

PI control uses raw or lightly filtered speed; CAN feedback uses a separate smoother filter. Both filters recompute their coefficients from real `dt`. PI includes output limiting and conditional-integration anti-windup; its integral is cleared when disabled, when leaving DRIVE, on a fault/brake/calibration transition, and before a new DRIVE enable.

Encoder direction fault requires DRIVE, a stable target direction above threshold, post-reversal settling time, and sustained opposite trusted speed for a debounce interval. A single opposite count is evidence only.

`MOTOR_STALL` is armed only in DRIVE after its arm delay. It requires a valid encoder, stable sufficiently high target, high PI recommendation, and persistently low measured speed. It means sustained driven operation without credible movement response; it does not prove a purely mechanical jam.

## 6. Calibration state machine

```text
PRECHECK
 -> LEFT_FORWARD -> LEFT_SETTLE -> LEFT_REVERSE
 -> RIGHT_FORWARD -> RIGHT_SETTLE -> RIGHT_REVERSE
 -> VALIDATE -> COMPLETE
```

`LEFT_SETTLE` and `RIGHT_SETTLE` issue zero-torque recommendations and wait for the tested wheel to satisfy trusted-still hold before the following reverse stage. Positive and reverse measurements must be based only on trusted encoder samples. The state machine validates count response and command-relative direction only; it cannot establish separate A/B channel health and does not implement high-speed edge diagnostics.

Only explicitly proven validation failures latch `CALIBRATION_FAILED`. Safety/operator abort, CAN timeout, and encoder-invalid safety preemption discard pending data and return through `CALIBRATION_REQUIRED` after their normal recovery conditions.

## 7. Calibration protocol and implementation appendix

### 7.1 Service command and feedback

`0x122 CMD_CALIBRATION` and `0x185 FB_CALIBRATION` are specified byte-for-byte in [CAN_PROTOCOL.md](CAN_PROTOCOL.md#14-calibration-service-amendment). The calibration service sequence is independent of `command_seq`; calibration frames never refresh the ordinary command watchdog.

Duplicate Start, Query, Cancel, busy rejection, terminal caching, modular sequence comparison, and response timing are protocol behavior, not host conventions.

### 7.2 Preconditions and trusted samples

A calibration transaction waits in PRECHECK until both conditions have held continuously:

1. A fresh all-zero STOP exists: a complete `0x120 + 0x121` command group with matching sequence/flags, inside the watchdog freshness window, STOP mode, and both wheel targets equal to zero.
2. Both wheels are credibly stationary.

| Parameter | Safe default |
|---|---:|
| `CAL_STOP_HOLD_MS` | 300 ms |
| `CAL_WHEEL_STILL_HOLD_MS` | 300 ms |
| `CAL_STILL_SPEED_MAX_MMPS` | 10 mm/s |
| `CAL_INVALID_CONSECUTIVE_SAMPLES_MAX` | 5 samples |
| `CAL_INVALID_ACCUMULATED_MAX_MS` | 100 ms |
| `CAL_VALID_RECOVERY_HOLD_MS` | 100 ms |

An invalid sample immediately breaks stillness hold and contributes neither motion count nor direction evidence. Five continuous invalid samples, or 100 ms accumulated invalid duration before 100 ms continuous valid recovery, raises `ENCODER_FAULT`; it is never recast as `CALIBRATION_FAILED`.

### 7.3 Timing and bounded output

| Parameter | Safe default |
|---|---:|
| `CAL_PRECHECK_TIMEOUT_MS` | 5 000 ms |
| `CAL_FORWARD_TIMEOUT_MS` | 2 000 ms |
| `CAL_SETTLE_TIMEOUT_MS` | 1 000 ms |
| `CAL_REVERSE_TIMEOUT_MS` | 2 000 ms |
| `CAL_WHEEL_TIMEOUT_MS` | 6 000 ms |
| `CAL_TOTAL_TIMEOUT_MS` | 20 000 ms |
| `CAL_PWM_MAX_PERMILLE` | 250 |
| `CAL_PWM_START_PERMILLE` | 100 |
| `CAL_PWM_STEP_PERMILLE` | 25 |
| `CAL_PWM_STEP_HOLD_MS` | 200 ms |
| `CAL_MAX_NONZERO_STAGE_MS` | 1 500 ms |
| `CAL_MIN_RESPONSE_COUNTS` | 560 counts |
| `CAL_START_RESPONSE_COUNTS` | 280 counts |

PWM must never exceed `CAL_PWM_MAX_PERMILLE`. `CAL_REQUIRE_MAINTENANCE_CONFIRM` and `CAL_REQUIRE_WHEELS_LIFTED_CONFIRM` default to true. They are operator attestations, not sensor proof. A CAN Cancel requires maintenance confirmation; local safety mechanisms can always abort.

### 7.4 Timeout and exit classification

| Event | Transaction result | Vehicle result |
|---|---|---|
| Trusted evidence proves insufficient motion, wrong direction, no effective start response, failed settle, or inconsistent wheel result | `FAILED_VALIDATION` | `CALIBRATION_FAILED -> FAULT + COAST` |
| PRECHECK does not establish holds | `ABORTED / PRECHECK_TIMEOUT` | `CALIBRATION_REQUIRED` |
| CAN command watchdog expires | `ABORTED / COMMUNICATION_TIMEOUT` | `SAFE_STOP + COAST`, then `CALIBRATION_REQUIRED` after recovery |
| Encoder invalid threshold | `ABORTED / ENCODER_INVALID` | existing `ENCODER_FAULT` safety path |
| ESTOP, other fault, or local/CAN cancel | `ABORTED` or `CANCELED` | existing safety path, then `CALIBRATION_REQUIRED` when permitted |

A generic timeout is a validation failure only when uninterrupted trusted evidence proves the stage's measurement requirement was not met. It must not obscure encoder, communication, or safety failures.

### 7.5 Pending and atomic commit

```c
typedef struct {
    uint8_t  valid;
    int8_t   left_encoder_polarity;
    int8_t   right_encoder_polarity;
    uint16_t left_min_start_pwm_permille;
    uint16_t right_min_start_pwm_permille;
    uint16_t version;
    uint16_t reserved;
} CalibrationData;
```

The state machine writes an independent `pending` structure only. Once both wheels pass all tests and cross-wheel validation, `active = pending` is performed once inside a critical section. No field-by-field or single-wheel update is allowed. Existing active data is preserved during recalibration and on every abort or failure; it does not authorize DRIVE while calibration is active. Flash persistence is intentionally excluded; the structure reserves versioning for future CRC, dual-slot, valid-marker, and power-loss recovery support.

Validated calibration is persisted in two independent Flash sectors (`0x080C0000` / sector 10 and `0x080E0000` / sector 11). Each 32-byte record contains an explicit serialized payload, generation, CRC32, and a commit marker programmed last. Startup selects the newest record whose format, CRC, commit marker, and calibration payload are all valid; an incomplete or corrupt newest record falls back to the other slot. `CommitPending` writes the inactive slot before replacing the active RAM snapshot, so a failed write preserves the prior active calibration.

## 8. Required verification

Implementation and codec tests must cover: duplicate and wraparound service sequences; Query; busy Start; Cancel and restart gate; one-sample stillness; invalid sample interruption and escalation; settle-before-reverse; CAN timeout; ESTOP/FAULT preemption; each single-wheel failure; one wheel success/other failure; pending discard; atomic commit visibility; preservation of old active calibration; and validation-timeout versus sensor-invalid-timeout classification.
