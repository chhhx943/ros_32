# Ackermann Chassis Motion Implementation

## Goal
Implement the approved host-side R3X Ackermann geometry, solver, configuration, CAN command adaptation, audit records, deterministic safe-stop signalling, and the required zero-speed static-steering behavior.

## Phases

| Phase | Status | Description |
| --- | --- | --- |
| 1. Context and design baseline | complete | Checked the pasted specification, repository protocol, MCU decoder, tests, dirty worktree, and prior hardware evidence; archived the complete approved specification. |
| 2. Detailed implementation plan | complete | Mapped files, APIs, red/green cycles, commits, verification, and software/hardware acceptance boundaries. |
| 3. Geometry and configuration | complete | Added immutable validated geometry, one JSON source for R3X dimensions, derived tire radius, and the 600 mm/s protocol-bounded profile. |
| 4. Pure Ackermann solver | complete | Added ordered runtime validation, immutable results/status, steering limiting, curvature/radius, directional wheel splitting, reverse semantics, and uniform scaling. |
| 5. CAN command adapter | complete | Added frozen V1 frame reuse, same-result pairing, ordered sending, per-frame outcomes, safe-stop request/operation, watchdog fallback, audit records, and stable exports. |
| 6. Regression and build verification | complete | Compileall passed, full Python suite passed 119/119, ordinary Debug build exited 0, scoped diff check passed, and section 69 software items were audited. |
| 7. Review and handoff | complete | Reviewed all new source/tests against the full specification; no critical or important correction was required. |

## Constraints

- Preserve all unrelated user changes in the dirty worktree.
- Do not implement Ackermann mathematics in STM32 firmware.
- Reuse `tools/pid/protocol.py`; do not duplicate CAN payload encoding.
- Use TDD: observe every new behavior test fail for the expected reason before production edits.
- Do not automatically run motors or steering hardware; hardware runs require the operator safety gate.
- Keep Python 3.9 compatibility.
- Use the conservative currently verified 600 mm/s lifted-bench command ceiling; 3000 mm/s remains only the protocol ceiling.

## Decisions

- Work in place because required CAN/PID/MCU files are untracked or modified and would be absent from a clean worktree.
- Static geometry/configuration errors raise `ValueError`; runtime solver inputs return an invalid safe result.
- Partial motion transmission returns a safe-stop request; the actual STOP group uses a separate fresh sequence.
- Zero-speed steering is allowed only in a valid STANDBY command with fresh CAN data, no fault, and both rear-wheel targets at zero; wheels remain COAST and stop/estop paths still neutralize steering.

## Completion Checklist

- [x] Context and design baseline complete.
- [x] Detailed implementation plan complete.
- [x] Geometry and configuration complete.
- [x] Pure Ackermann solver complete.
- [x] CAN command adapter complete.
- [x] Regression and build verification complete.
- [x] Review and handoff complete.

## Hardware Acceptance Extension

| Phase | Status | Description |
| --- | --- | --- |
| H1. Non-motion connection check | complete | Confirmed Debug/bench macros are OFF and enumerated ST-LINK SN `3E3703013212354D434B4E00` without reset or download. |
| H2. Physical safety gate | complete | Operator confirmed all wheels lifted/fixed, linkage clear, and PE1 E-stop immediately reachable. |
| H3. Steering validation | complete | Dedicated default-off loopback bench passed: `0 → +87 → 0 → −87 → 0` mrad, PWM `1500 → 1572 → 1500 → 1428 → 1500` us, five phases complete, 245 zero-wheel-speed groups, zero TX failures; normal Debug firmware restored. |
| H4. Lifted-wheel Ackermann validation | needs_investigation | CAN vectors, PWM, direction pins, zero faults, and positive encoder response passed; measured left/right encoder differential did not match the requested Ackermann ratio at the first conservative 200 mm/s trial. |
| H5. Ground low-speed validation | blocked_by_H4 | Do not proceed until encoder/channel mapping or low-speed differential response is explained and H4 is repeated successfully. |

## Errors Encountered

| Error | Attempt | Resolution |
| --- | ---: | --- |
| PowerShell default decoding rendered the UTF-8 attachment as mojibake | 1 | Re-read with UTF-8 encoding. |
| Dynamic `apply_patch` omitted `*** Begin Patch` | 1 | Added the required patch header. |
| One patch tried to delete and add the same path | 1 | Used separate delete and add patch operations. |
| Full-repository `git diff --check` reported Keil dependency-file whitespace | 1 | Confirmed the warnings are pre-existing outside this change; scoped diff check exits 0. |
| Planning completion checker reported `0/0 phases` for the scoped Windows plan | 2 | Retained explicit table statuses and checked completion items as the authoritative record. |
| First steering bench sampled before the control period | 1 | Waited a complete 20 ms refresh period before phase sampling. |
| Existing standby action forced zero-speed steering to centre | 1 | Added the constrained static-steering path required by the approved specification. |
