# PID autotune task plan

**Goal:** Add a repeatable, safety-aware PID tuning loop that runs as `python tools/pid/autotune.py`, works offline in a deterministic simulator, and can use an optional CAN transport when the MCU exposes the maintenance gain extension.

**Scope:** Host-side tooling plus the smallest MCU protocol/runtime hook needed to make `set_pid` real. No flash persistence, current/voltage fabrication, or bypass of existing MCU safety limits.

## Phases

| Phase | Status | Description |
|---|---|---|
| 1. Context and design | complete | Audited existing PID, CAN, bench, encoder, safety, and test code; chose simulator-first host loop and maintenance-only PID extension. |
| 2. Host metrics/search tests | complete | Added failing-first tests for analysis, safety aborts, deterministic search, and experiment state. |
| 3. Host implementation | complete | Implemented protocol, transports, experiment runner, analysis, search, persistence, and CLI. |
| 4. MCU gain extension | complete | Added maintenance-only PID set frames and runtime gain application with tests, including right-wheel isolation. |
| 5. Verification | complete | Full host tests, simulator autotune, firmware build, and generated report inspection passed. |
| 6. Live CAN readiness | in progress | Installed `python-can`, applied the 33.25 mm wheel radius, hardened live ACK/safety feedback checks, and verify a physical adapter before any motor command. |

## Errors Encountered

| Error | Attempt | Resolution |
|---|---|---|
| Initial skill path used aliases as filesystem directories | 1 | Resolved aliases to their actual skill roots and reread the required files. |
| Full unittest discovery exceeded the first 30-second observation window | 1 | Polled the existing process; it completed successfully with 73 tests in 34.994 seconds. |
| `python-can` install through the configured mirror returned HTTP 403 | 1 | Installed `python-can 4.6.1` from PyPI with approved network access. |
| Windows CAN adapter discovery | 1 | Read-only device enumeration found only Bluetooth virtual COM ports; no recognized CAN adapter/driver is currently present. |

## Files to create/modify

- Create `docs/superpowers/specs/2026-08-31-pid-autotune-design.md`.
- Create `docs/superpowers/plans/2026-08-31-pid-autotune.md`.
- Create `tools/pid/{__init__.py,model.py,protocol.py,transport.py,analysis.py,experiment.py,search.py,autotune.py}`.
- Create `tools/pid/default_config.json` and `tests/test_pid_autotune.py`.
- Modify `BSP/bsp_bxcan.h/.c`, `BSP/chassis_control.h/.c`, `CMakeLists.txt`, `docs/CAN_PROTOCOL.md`, and add focused MCU protocol tests only after host behavior is specified by failing tests.
