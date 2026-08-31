# PID Automatic Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic, safety-aware PID autotuning loop with simulator and optional CAN transport, plus an MCU maintenance-only gain update path.

**Architecture:** Keep the existing 10 ms dual-wheel PID and safety owner. Add focused Python modules for transport, experiment orchestration, metrics, search, and reporting; add a separate maintenance CAN extension that is accepted only during a safe STOP transaction.

**Tech Stack:** Python 3.9 standard library, NumPy only if useful but not required, existing C99 host-test pattern, STM32 HAL/CMake.

---

### Task 1: Host contract and failing tests

**Files:**
- Create: `tests/test_pid_autotune.py`
- Create: `tools/pid/__init__.py`
- Create: `tools/pid/model.py`
- Create: `tools/pid/analysis.py`

- [ ] Write tests for step metrics, safety abort classification, deterministic simulator response, and bounded candidate generation before implementation.
- [ ] Run `python -m unittest tests.test_pid_autotune -v`; confirm failures are due to missing modules/behaviors.
- [ ] Implement the smallest pure data structures and functions required by those tests.
- [ ] Re-run the focused test until green, then run `python -m unittest discover -s tests -q`.

### Task 2: Protocol and transports

**Files:**
- Create: `tools/pid/protocol.py`
- Create: `tools/pid/transport.py`
- Modify: `BSP/bsp_bxcan.h`, `BSP/bsp_bxcan.c`, `BSP/chassis_control.h`, `BSP/chassis_control.c`, `CMakeLists.txt`
- Create: `tests/test_pid_can_extension.py`

- [ ] Add failing Python/C tests for fixed-point PID frame round-trip, rejected non-maintenance update, and accepted safe update.
- [ ] Implement host frame codec and deterministic simulator transport first.
- [ ] Implement optional `python-can` transport with clear dependency/interface errors and no implicit installation.
- [ ] Add MCU runtime gain storage, validated frame assembly, safe-state gate, feedback serializer, and apply gains to both controllers without changing output limits.
- [ ] Build and run focused tests, then CMake Debug.

### Task 3: Experiment runner and search

**Files:**
- Create: `tools/pid/experiment.py`
- Create: `tools/pid/search.py`
- Create: `tools/pid/default_config.json`
- Create: `tools/pid/autotune.py`
- Modify: `docs/CAN_PROTOCOL.md`

- [ ] Add tests for `Observe → Analyze → Hypothesis → Select Candidate → Run Test → Evaluate → Compare → Next Action`, state persistence, STOP/finally restoration, and CSV/JSON fields.
- [ ] Implement fixed scenarios: low-speed step, high-speed step, acceleration, deceleration, and optional reverse.
- [ ] Implement seeded random search followed by local neighbors within configured bounds.
- [ ] Add CLI options for transport, iterations, seed, config, output directory, and dry-run.
- [ ] Document the maintenance extension and clearly mark current/voltage as unavailable when absent.

### Task 4: Verification and handoff

**Files:**
- Modify: `.planning/pid-autotune/task_plan.md`, `.planning/pid-autotune/progress.md`, `progress.md`, `findings.md`

- [ ] Run focused and full Python tests.
- [ ] Run `python tools/pid/autotune.py --transport sim --iterations 3 --seed 7` and inspect report/history/CSV.
- [ ] Run CMake Debug build and focused MCU host tests.
- [ ] Confirm no live-hardware claim is made unless a real CAN backend and feedback are available.
