# Chassis Control Design Documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the frozen chassis closed-loop, safety, encoder, and calibration design as a repository baseline, with its CAN service extension defined at byte level.

**Architecture:** `docs/CHASSIS_CONTROL_DESIGN.md` is the authoritative cross-module design. `docs/CAN_PROTOCOL.md` remains the authoritative wire-format contract and receives only a backward-compatible calibration-service amendment.

**Tech Stack:** Markdown, Classic CAN 2.0A, STM32F407 HAL, TB6612FNG.

---

### Task 1: Publish the chassis-control architecture baseline

**Files:**
- Create: `docs/CHASSIS_CONTROL_DESIGN.md`

- [x] **Step 1: Write the frozen ownership, safety, motor, encoder, PI, and calibration rules**

  Include the only actuator path `wheel_* -> chassis_control -> bsp_motor`, the `ESTOP > FAULT > SAFE_STOP` exception priority, the 100 Hz one-shot real-`dt` scheduler rule, and the `500 x 4 x 28` encoder definition.

- [x] **Step 2: Add the calibration transaction appendix**

  Define `CMD_CALIBRATION`, `FB_CALIBRATION`, steady holds, invalid-sample handling, timeout classification, pending/atomic commit, safe defaults, and the conformance matrix.

- [x] **Step 3: Review for contradicting actuator and fault semantics**

  Verify that COAST is `IN1=0, IN2=0, PWM=1, STBY=1`; BRAKE is `IN1=1, IN2=1, STBY=1`; no invalid encoder sample counts as stationary; and no abort writes active calibration.

### Task 2: Add the backward-compatible CAN V1 calibration amendment

**Files:**
- Modify: `docs/CAN_PROTOCOL.md`

- [x] **Step 1: Register `0x122 CMD_CALIBRATION` and `0x185 FB_CALIBRATION`**

  Document both as standard, 8-byte V1 frames and state that old receivers ignore the new identifiers.

- [x] **Step 2: Define the exact field layouts and sequence rules**

  Specify opcode, service sequence modular comparison, duplicate handling, cached terminal replies, response timing, stage/state/result fields, and all reserved-byte requirements.

- [x] **Step 3: Add protocol conformance cases**

  Cover duplicate requests, busy rejection, cancel/restart gate, zero-command hold, invalid encoder samples, timeout classes, atomic commit, and sequence wrap.

### Task 3: Verify documentation consistency

**Files:**
- Verify: `docs/CHASSIS_CONTROL_DESIGN.md`
- Verify: `docs/CAN_PROTOCOL.md`

- [x] **Step 1: Search for the frozen identifiers and semantic anchors**

  Run: `rg -n "0x122|0x185|CALIBRATION_FAILED|56000|Motor_Coast|service_seq" docs/CHASSIS_CONTROL_DESIGN.md docs/CAN_PROTOCOL.md`

  Expected: both documents contain the shared identifiers and no missing calibration service definition.

- [x] **Step 2: Inspect the documentation diff**

  Run: `git diff --check -- docs/CHASSIS_CONTROL_DESIGN.md docs/CAN_PROTOCOL.md docs/superpowers/plans/2026-08-30-chassis-control-design-docs.md`

  Expected: no whitespace errors.
