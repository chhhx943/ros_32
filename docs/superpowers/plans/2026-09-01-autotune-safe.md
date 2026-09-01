# AutotuneSafe PID Hard Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an MCU-owned, compile-time isolated `AutotuneSafe` PID tuning system with conservative staged unlock, independent abort/recovery, host safety checks, telemetry, tests, and two build profiles.

**Architecture:** `BSP/autotune_safe.c/.h` owns the MCU tuning session, level, local evidence, abort/recovery, and output shaping. `Motor_Drive()` remains the final actuator gate in `AUTOTUNE_SAFE_PROFILE`; coast/brake safety outputs remain immediately available. Host code mirrors the protocol/session contract but never supplies limits or safety authority.

**Tech Stack:** STM32F407 C11/HAL/CMake, existing `PID`, `chassis_control`, `safety_manager`, `bsp_motor`, bxCAN V1 extension, Python 3 `unittest`, existing simulator/transport/reporting modules.

---

## Files and responsibilities

- Create `BSP/autotune_safe.h/.c`: MCU state machine, compile-time level table, bootstrap/best-safe PID policy, target/PWM shaping, local evidence, session watchdog, abort/recovery, telemetry snapshot.
- Modify `BSP/bsp_motor.h/.c`: final drive gate hook for AutotuneSafe; all coast/brake paths remain immediate.
- Modify `BSP/chassis_control.c/.h`: route every closed-loop/calibration/static/bench drive through the gate, provide local 10 ms feedback and reset hooks, never drive after an autotune abort.
- Modify `BSP/pid_tuning.c/.h`: absolute/Q8.8/bootstrap/relative gain validation and best-safe transaction handoff.
- Modify `BSP/bsp_bxcan.c/.h`, `tools/pid/protocol.py`, and `tools/pid/transport.py`: control/status/metrics frames with session and snapshot sequencing.
- Modify `CMakeLists.txt`, `CMakePresets.json`, and guarded `Core/Src/main.c` wiring: compile-time `AutotuneSafe` profile only; ordinary Debug/Release cannot activate it.
- Create/update `tests/test_autotune_safe_*.py`: host simulator, C host-test harness, protocol, abort, watchdog, bypass, and profile isolation coverage.
- Modify `tools/pid/{model,experiment,search,analysis,autotune}.py` and `tools/pid/default_config.json`: safety-first runner/search/report configuration.
- Update `docs/CAN_PROTOCOL.md` and add sample report schema under tests/fixtures if needed; do not touch Ackermann equations.

### Task 1: Freeze design and build-profile contract

**Files:** `docs/AUTOTUNE_SAFE_DESIGN.md`, `docs/superpowers/plans/2026-09-01-autotune-safe.md`, `CMakeLists.txt`, `CMakePresets.json`, `tests/test_autotune_safe_profile.py`.

- [ ] **Step 1: Write failing profile-isolation tests** asserting `AUTOTUNE_SAFE_PROFILE` exists only in the dedicated preset, `autotune_safe.c` is part of the target, normal Debug/Release do not define the macro, and normal `main.c` cannot route a CAN frame to an autotune actuator mode.
- [ ] **Step 2: Run `python -m unittest discover -s tests -p test_autotune_safe_profile.py -v` and confirm failure is due to missing profile/source wiring.**
- [ ] **Step 3: Add the `AUTOTUNE_SAFE_PROFILE` CMake option, dedicated `AutotuneSafe` configure/build preset, source list entry, and guarded main-loop initialization/process calls.** Keep all existing bench options OFF in the preset.
- [ ] **Step 4: Run the focused profile tests and inspect `CMakeCache.txt` for Debug vs AutotuneSafe definitions.**
- [ ] **Step 5: Commit only the design/profile changes with `git add docs/AUTOTUNE_SAFE_DESIGN.md docs/superpowers/plans/2026-09-01-autotune-safe.md CMakeLists.txt CMakePresets.json tests/test_autotune_safe_profile.py && git commit -m "docs: freeze AutotuneSafe profile contract"` if the worktree policy permits a focused commit.

### Task 2: MCU safety core and deterministic host-test harness

**Files:** Create `BSP/autotune_safe.h/.c`, `tests/test_autotune_safe_core.py`.

- [ ] **Step 1: Write failing C host-test cases** for L0 drive rejection, L1 target/PWM envelope, bootstrap seed acceptance, absolute gain rejection, 25% relative step rejection, first safe result enabling relative checks, `REQUEST_PROMOTE` not directly changing level, and serious abort preventing promotion.
- [ ] **Step 2: Run the C harness through the Python test and confirm expected missing-header/symbol failures.**
- [ ] **Step 3: Implement pure deterministic APIs: `AutotuneSafe_Init`, `AutotuneSafe_OnControlFrame`, `AutotuneSafe_RequestPromote`, `AutotuneSafe_ValidateCandidate`, `AutotuneSafe_BeginExperiment`, `AutotuneSafe_RecordSample`, `AutotuneSafe_RequestStop`, `AutotuneSafe_Process`, `AutotuneSafe_GetStatus`, and `AutotuneSafe_GetTelemetry`.** Define explicit enums and compile-time level constants; do not expose a setter for level or limits.
- [ ] **Step 4: Run the focused C harness and verify all core cases pass.**
- [ ] **Step 5: Refactor only after green so abort priority and bootstrap state remain explicit and testable.**

### Task 3: Final actuator gate, ramp/slew, and abort recovery

**Files:** `BSP/bsp_motor.h/.c`, `BSP/chassis_control.c/.h`, `BSP/PID.c/.h`, `tests/test_autotune_safe_actuator.py`, `tests/test_autotune_safe_bypass.py`.

- [ ] **Step 1: Write failing tests** for the exact output chain, 0-to-large PWM slew limiting, target ramp, immediate coast/brake bypass of slew, reset of integral/history before recovery, and all direct drive callers passing through the final gate.
- [ ] **Step 2: Run focused tests and confirm they fail before the gate exists.**
- [ ] **Step 3: Add the AutotuneSafe gate inside `Motor_Drive()` before TB6612 direction/PWM writes; add explicit target-ramp and PWM-slew state in `autotune_safe.c`; make `chassis_control` submit local target/actual/PID observations before the final write.** Every calibration/static/bench drive call must use this same path. `Motor_Coast` and `Motor_Brake` must remain immediate.
- [ ] **Step 4: Add abort sequencing that zeroes/freeze outputs and PID state before waiting for stillness, then restores the best-safe gains.**
- [ ] **Step 5: Run actuator/bypass tests and inspect source with `rg -n "Motor_Drive\\("` to prove no AutotuneSafe build path writes around the gate.**

### Task 4: PID transaction hardening and local safety evidence

**Files:** `BSP/pid_tuning.c/.h`, `BSP/safety_manager.c/.h`, `BSP/chassis_control.c`, `tests/test_autotune_safe_abort.py`, `tests/test_pid_tuning_protocol.py`.

- [ ] **Step 1: Write failing tests** for Q8.8 overflow/invalid values, absolute bounds, bootstrap step limits, relative best-safe limits, repeated-failure baseline protection, stall, saturation timeout, overspeed ratio/ceiling, oscillation, encoder invalid, E-stop precedence, and thermal/runtime proxy.
- [ ] **Step 2: Run focused tests and record the expected failures.**
- [ ] **Step 3: Implement all evidence from local 10 ms data only; use existing Safety/E-stop state as an input and never use host telemetry for decisions.** Keep watchdog and E-stop priority above autotune state.
- [ ] **Step 4: Implement persistent-in-process cooling/energy accounting that survives session/host identifiers and clears only by an explicit MCU boot-safe reset policy, not by CAN session start.**
- [ ] **Step 5: Run focused abort tests and verify each reason maps to the fixed abort action and cannot update best-safe.**

### Task 5: CAN control, session freshness, and sequenced telemetry

**Files:** `BSP/bsp_bxcan.h/.c`, `BSP/autotune_safe.h/.c`, `tools/pid/protocol.py`, `tools/pid/transport.py`, `tests/test_autotune_safe_protocol.py`, `docs/CAN_PROTOCOL.md`.

- [ ] **Step 1: Write failing round-trip tests** for unique session/experiment IDs, PID candidate/START/STOP/heartbeat/REQUEST_PROMOTE control frames, duplicate/out-of-order rejection, session watchdog expiry, snapshot sequence consistency, and all required telemetry fields split across Classic CAN frames.
- [ ] **Step 2: Run the protocol tests and confirm missing frame codecs/handlers.**
- [ ] **Step 3: Add explicit frame IDs and byte layouts without changing existing `0x180..0x188` feedback frames; route accepted frames to MCU AutotuneSafe only in the compile-time profile.**
- [ ] **Step 4: Add MCU status/metrics snapshot construction with `session_id`, `experiment_id`, `snapshot_seq`, effective target/limit, flags, abort reason, safety state/fault, and command age.**
- [ ] **Step 5: Run protocol tests and update the normative CAN documentation with payload tables and stale/partial snapshot rules.**

### Task 6: Host simulator, safety-first runner, search, and persistence

**Files:** `tools/pid/model.py`, `tools/pid/experiment.py`, `tools/pid/search.py`, `tools/pid/analysis.py`, `tools/pid/autotune.py`, `tools/pid/default_config.json`, `tests/test_autotune_safe_runner.py`, `tests/test_pid_search_and_cli.py`, `tests/test_pid_experiment.py`.

- [ ] **Step 1: Write failing simulator/runner tests** for preflight, ramped one-direction scenarios, STOP-and-zero confirmation, MCU abort precedence, host-loss heartbeat, best-safe recovery, safety-first score exclusion, and JSON/CSV required fields.
- [ ] **Step 2: Run the focused host tests and confirm missing safety session behavior.**
- [ ] **Step 3: Extend the simulator and transport interface with profile/status/session/heartbeat/start/stop/promote/effective telemetry operations; model local MCU aborts independently of host polling.**
- [ ] **Step 4: Update `ExperimentRunner` to use bootstrap then relative limits, preserve baseline after failures, and reject direct reverse/high-speed/default scenarios.**
- [ ] **Step 5: Implement P-only -> PI -> optional D local refinement and safety-first leaderboard/report persistence with the frozen telemetry keys.**
- [ ] **Step 6: Run focused host tests and the CLI simulator with only L1 defaults.**

### Task 7: Full regression, static checks, and two firmware builds

**Files:** `tests/test_autotune_safe_integration.py`, `CMakeLists.txt`, `CMakePresets.json`, `docs/AUTOTUNE_SAFE_DESIGN.md`.

- [ ] **Step 1: Add integration tests** proving normal Debug/Release cannot enter AutotuneSafe, AutotuneSafe can only be enabled by its build profile, all direct drive sources are gated, and host loss causes MCU abort in the C harness.
- [ ] **Step 2: Run `python -m unittest discover -s tests -v` and fix only regressions caused by this feature.**
- [ ] **Step 3: Run static checks: `git diff --check`, `rg -n "AUTOTUNE_SAFE_PROFILE|Motor_Drive\\(|REQUEST_PROMOTE|session_id|experiment_id|abort_reason" BSP Core tools tests docs`.**
- [ ] **Step 4: Configure/build ordinary Debug and the dedicated AutotuneSafe preset; verify both exit code 0 and inspect their compile definitions.**
- [ ] **Step 5: Run the final verification checklist against every frozen requirement and record evidence in `progress.md` and `findings.md`.**

### Task 8: Manual flash boundary and supervised L1 procedure

**Files:** `progress.md`, `docs/AUTOTUNE_SAFE_DESIGN.md`.

- [ ] **Step 1: Stop after successful software verification and present the exact `AutotuneSafe` ELF path and build evidence.**
- [ ] **Step 2: Before any flash, confirm manually: rear wheels fully lifted, E-stop reachable/tested, no H5, CAN/logging path ready, and L1 target/PWM limits unchanged.**
- [ ] **Step 3: Only after explicit hardware confirmation, flash the dedicated image and run the first one-direction L1 sequence under human supervision; do not issue direct reverse or level promotion.**
- [ ] **Step 4: Capture telemetry/abort behavior, stop the session, and flash ordinary Debug back; record that software limits are proxies, not current protection.**
