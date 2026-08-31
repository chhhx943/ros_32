# Minimum Runnable Closed Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current CAN-to-open-loop rear motor bridge with a minimum runnable 100 Hz closed-loop rear-wheel velocity path.

**Architecture:** Keep `chassis_control` as the only actuator caller. Evolve the existing `encoder.c`, `PID.c`, and `bsp_motor.c` boundaries in place: `encoder` produces trusted wheel samples, `PID` produces bounded PWM recommendations using measured `dt`, and `chassis_control` arbitrates STOP/timeout/E-stop versus DRIVE before calling `Motor_*`.

**Tech Stack:** STM32F407 HAL, TIM1/TIM2 encoder mode, TIM3 PWM, TIM6 100 Hz tick shape, host-side C snippets compiled by Python unittest.

---

### Task 1: Encoder Trusted Sample API

**Files:**
- Modify: `BSP/encoder.h`
- Modify: `BSP/encoder.c`
- Test: `tests/test_encoder.py`

- [x] **Step 1: Write the failing encoder tests**

  Add host tests that include `BSP/encoder.c` with fake TIM handles and counters. Cover these behaviors:
  - `Encoder_Init()` starts TIM1 and TIM2 encoder channels.
  - Left wheel reads TIM1, right wheel reads TIM2.
  - A 560 count delta over 10 ms reports about 212 mm/s using `COUNTS_PER_WHEEL_REV = 56000`.
  - Accumulated counts are signed 64-bit and update by the trusted delta.
  - A physically impossible delta returns `trusted = 0` and does not update accumulation.

  Run: `python -m unittest discover -s tests -p test_encoder.py -v`

  Expected before implementation: FAIL because the new `Encoder_Init` and `Encoder_Sample` API does not exist.

- [x] **Step 2: Implement the minimal encoder API**

  Add:

  ```c
  typedef struct {
      int32_t delta_counts;
      int64_t accumulated_counts;
      int32_t velocity_mmps;
      uint8_t trusted;
  } EncoderSample_t;

  void Encoder_Init(void);
  void Encoder_Reset(void);
  EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms);
  ```

  Implementation requirements:
  - Motor 1 maps to `htim1`; motor 2 maps to `htim2`.
  - Start both timers with `HAL_TIM_Encoder_Start(..., TIM_CHANNEL_ALL)`.
  - Use modulo delta based on each timer period plus one.
  - Convert counts to mm/s with integer math: `delta * wheel_circumference_mm * 1000 / (56000 * dt_ms)`.
  - Use a conservative maximum trusted wheel speed and margin for `max_physical_delta(dt_ms)`.
  - Preserve `float Encoder_Get(uint8_t num)` as a compatibility wrapper over a 20 ms sample.

- [x] **Step 3: Verify encoder tests pass**

  Run: `python -m unittest discover -s tests -p test_encoder.py -v`

  Expected after implementation: PASS.

### Task 2: Fixed-dt PID API

**Files:**
- Modify: `BSP/PID.h`
- Modify: `BSP/PID.c`
- Test: `tests/test_pid_controller.py`

- [x] **Step 1: Write the failing PID tests**

  Add host tests that include `BSP/PID.c` and cover:
  - `PID_Reset()` clears previous error, integral, and output.
  - `PID_UpdateDt()` multiplies integral by `dt_s`.
  - Output clamps to `OutMin`/`OutMax`.
  - Conditional integration does not keep winding the integral deeper into saturation.
  - Existing `PID_Update()` still compiles and delegates with a 1.0 second compatibility `dt`.

  Run: `python -m unittest discover -s tests -p test_pid_controller.py -v`

  Expected before implementation: FAIL because `PID_Reset` and `PID_UpdateDt` do not exist.

- [x] **Step 2: Implement the minimal fixed-dt PID**

  Add:

  ```c
  void PID_Reset(PID_t *p);
  void PID_UpdateDt(PID_t *p, float dt_s);
  ```

  Implementation requirements:
  - Treat nonpositive `dt_s` as no integration and no derivative blow-up.
  - Use position-form PID with `Ki * integral` and `Kd * derivative`.
  - Clamp `Out` to configured bounds.
  - For anti-windup, accept an integral candidate only when output is not saturated or when the current error would move the saturated output back toward range.
  - Keep the existing struct fields and `PID_Update(PID_t *p)` ABI.

- [x] **Step 3: Verify PID tests pass**

  Run: `python -m unittest discover -s tests -p test_pid_controller.py -v`

  Expected after implementation: PASS.

### Task 3: Chassis 100 Hz Closed-loop Velocity Step

**Files:**
- Modify: `BSP/chassis_control.h`
- Modify: `BSP/chassis_control.c`
- Test: `tests/test_chassis_closed_loop.py`
- Update: `tests/test_chassis_control.py`

- [x] **Step 1: Write the failing closed-loop chassis tests**

  Add host tests with fake CAN, encoder, PID, and motor functions. Cover:
  - `Chassis_ControlInit()` initializes encoder and resets both wheel controllers.
  - A velocity command does not call `Motor_Drive` immediately on command drain; it waits for the 10 ms control step.
  - At 10 ms, measured low speed causes positive PWM recommendations for positive targets.
  - STOP and command timeout coast both motors and reset the wheel controllers.
  - E-stop brakes both motors and resets the wheel controllers.
  - Missed ticks are not replayed: elapsed 30 ms produces one control update with `dt_ms = 30`.

  Run: `python -m unittest discover -s tests -p test_chassis_closed_loop.py -v`

  Expected before implementation: FAIL because `chassis_control` still maps velocity directly to open-loop PWM.

- [x] **Step 2: Implement the minimum 100 Hz closed-loop path**

  Implementation requirements:
  - Keep `BSP_BXCAN_Process(now_ms)` and command draining in `Chassis_ControlProcess`.
  - Store the latest accepted target command in `chassis_control`.
  - Run wheel control only when at least 10 ms elapsed since the previous wheel step.
  - If more than one period elapsed, run exactly once using the real elapsed `dt_ms`.
  - In DRIVE, sample each encoder, feed measured `velocity_mmps` into each PID, and call `Motor_Drive(1, left_pwm)` and `Motor_Drive(2, right_pwm)`.
  - If either encoder sample is untrusted in this minimum slice, coast both wheels and reset both controllers. Latching `ENCODER_FAULT` remains a later safety-manager task.
  - For STOP, SAFE_STOP, command timeout, or zero targets, coast both wheels and reset both controllers.
  - For E-stop, brake both wheels and reset both controllers.

- [x] **Step 3: Update old open-loop chassis tests**

  Revise `tests/test_chassis_control.py` expectations so velocity commands are verified through the closed-loop 10 ms step instead of immediate open-loop mapping.

- [x] **Step 4: Verify chassis tests pass**

  Run: `python -m unittest discover -s tests -p "test_chassis*.py" -v`

  Expected after implementation: PASS.

### Task 4: Firmware Integration Verification

**Files:**
- Verify: `CMakeLists.txt`
- Verify: `Core/Src/main.c`
- Verify: `BSP/*.c`

- [x] **Step 1: Run the full host suite**

  Run: `python -m unittest discover -s tests -v`

  Expected: all host tests pass.

- [x] **Step 2: Build Debug firmware**

  Run: `cmake --build build\Debug --target ros`

  Expected: Debug firmware links with 0 errors and 0 warnings.

- [x] **Step 3: Record implementation progress**

  Append a short dated note to `progress.md` and `findings.md` describing the closed-loop slice, verification commands, and remaining gaps: calibration service, full safety manager, physical E-stop, feedback population, and board tuning.

