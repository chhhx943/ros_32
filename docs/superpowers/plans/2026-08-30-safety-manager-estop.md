# Safety Manager and Physical E-stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a tested safety decision layer and PE1 physical E-stop path without giving the safety module direct motor ownership.

**Architecture:** `physical_estop` samples the active-low normally-closed PE1 input and latches an asynchronous event. `safety_manager` owns the vehicle safety state, E-stop recovery gate, fault/action priority, and drive permission. `chassis_control` remains the only module that turns the selected action into `Motor_Drive`, `Motor_CoastAll`, or `Motor_EmergencyBrakeAll`.

**Tech Stack:** STM32F407 HAL GPIO/EXTI, C11, host-side GCC snippets executed by Python unittest.

---

### Task 1: Define and test the physical E-stop input contract

**Files:**
- Create: `BSP/physical_estop.h`
- Create: `BSP/physical_estop.c`
- Test: `tests/test_physical_estop.py`

- [x] **Step 1: Write failing host tests**

  Cover active-low input detection, EXTI assertion, release without implicit clear, and explicit event consumption.

- [x] **Step 2: Run the focused test and verify RED**

  Run: `python -m unittest discover -s tests -p test_physical_estop.py -v`

- [x] **Step 3: Implement the smallest host-compatible input module**

  Expose `Physical_EStop_Init`, `Physical_EStop_Process`, `Physical_EStop_OnExti`, `Physical_EStop_IsAsserted`, and `Physical_EStop_ConsumeAssertEvent`. The input is active when PE1 reads low; release never clears a safety latch owned by `safety_manager`.

- [x] **Step 4: Verify the focused test is GREEN**

  Run the same focused command and require all tests to pass.

### Task 2: Add the safety state machine and test its priority/recovery rules

**Files:**
- Create: `BSP/safety_manager.h`
- Create: `BSP/safety_manager.c`
- Test: `tests/test_safety_manager.py`

- [x] **Step 1: Write failing state-machine tests**

  Cover boot-to-standby, physical E-stop BRAKE priority, CAN E-stop BRAKE priority, latched E-stop release requiring explicit all-zero STOP reset, fault COAST, timeout SAFE_STOP, and DRIVE permission only in the normal state.

- [x] **Step 2: Run the focused test and verify RED**

  Run: `python -m unittest discover -s tests -p test_safety_manager.py -v`

- [x] **Step 3: Implement the state machine**

  Use the frozen states `BOOT`, `STANDBY`, `DRIVE`, `SAFE_STOP`, `ESTOP`, and `FAULT`; return an execution action rather than calling any motor API. Apply priority `ESTOP > FAULT > SAFE_STOP > normal command`.

- [x] **Step 4: Verify the focused test is GREEN**

  Run the same focused command and require all tests to pass.

### Task 3: Wire PE1 EXTI and safety arbitration into firmware

**Files:**
- Modify: `Core/Inc/gpio.h`
- Modify: `Core/Src/gpio.c`
- Modify: `Core/Inc/stm32f4xx_it.h`
- Modify: `Core/Src/stm32f4xx_it.c`
- Modify: `BSP/chassis_control.c`
- Modify: `CMakeLists.txt`
- Test: `tests/test_chassis_closed_loop.py`

- [x] **Step 1: Extend integration tests**

  Verify a physical E-stop event prevents DRIVE and results in BRAKE, release alone does not resume, and a fresh reset STOP is required before a later velocity command can drive.

- [x] **Step 2: Run the integration test and verify RED**

  Run: `python -m unittest discover -s tests -p "test_chassis*.py" -v`

- [x] **Step 3: Configure PE1 and connect the safety decision**

  Configure GPIOE pin 1 as pull-up, falling-edge EXTI; enable `EXTI1_IRQn`; call `Physical_EStop_Process` and `Safety_Manager_Process` from the normal loop; pass accepted commands to the manager; make chassis control execute the manager's action.

- [x] **Step 4: Verify integration tests are GREEN**

  Run the same integration command.

### Task 4: Firmware and regression verification

**Files:**
- Verify: `BSP/*.c`, `Core/Src/main.c`, `docs/CHASSIS_CONTROL_DESIGN.md`

- [x] **Step 1: Run the complete host suite**

  Run: `python -m unittest discover -s tests -v`

- [x] **Step 2: Build normal Debug firmware**

  Run: `cmake --build build\\Debug --target ros`

- [x] **Step 3: Record the hardware test boundary**

  Document that PE1 electrical behavior must be checked with the vehicle powered but wheels mechanically restrained, then test E-stop assertion/release/reset before any DRIVE command.

## Stage Boundary

This plan implements the minimum physical-E-stop and safety-arbitration slice. Calibration gating, the complete frozen safety state machine (`CALIBRATION_REQUIRED` and latched fault taxonomy), encoder-fault/stall detection, and vehicle PID parameter tuning remain follow-up work.
