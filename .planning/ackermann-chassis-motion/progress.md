# Progress

## 2026-08-31

- Read the workflow skills and inspected the attachment, repository structure, protocol encoder, MCU decoder, tests, git history, and current dirty state.
- Compared the brief existing Ackermann design with the full approved attachment.
- Replaced the brief design with the complete specification, added explicit repository implementation decisions, self-reviewed it, and committed it as `5e99a02`.
- Began the detailed TDD implementation plan.
- Wrote and self-reviewed `docs/superpowers/plans/2026-08-31-ackermann-chassis-motion.md`; it covers geometry/config, pure solver behavior, CAN V1 reuse, partial-send outcomes, fresh-sequence safe stop, audit records, full regression, firmware build, review, and the operator-only hardware boundary.
- Geometry/config RED: focused test failed with `ModuleNotFoundError: tools.vehicle` as expected.
- Geometry/config GREEN: 4/4 focused tests passed; committed as `36c8f2f`.
- Solver contract RED failed because `tools.vehicle.ackermann` was absent; minimal validation/straight behavior then passed 3/3 tests.
- Turning RED produced the expected 6 failures for missing curvature, direction, clamp, scaling, radius, and status behavior.
- Full solver GREEN passed 10/10 solver tests and 4/4 geometry tests; committed as `6b7e40a`.
- Command frame RED failed because `tools.vehicle.command` was absent; exact V1 frame adaptation then passed 3/3 new tests and 5/5 existing Python protocol tests.
- Transmission RED failed on missing cycle status/send APIs; implementation then passed 11/11 command tests.
- Public API RED failed because `tools.vehicle.__init__` had no exports; stable package exports then passed.
- All vehicle tests passed 26/26 and C/host protocol regression passed 12/12; command slice committed as `e17b4cb`.
- Python compileall passed and the complete regression suite passed 119/119 in 70.154 seconds.
- Ordinary Debug firmware build exited 0 (`ninja: no work to do`); no flash or hardware action was performed.
- Reviewed all new code and tests against specification section 69. Scoped diff checks passed and no review-driven production correction was required.
- Kept the completed commits on the current `main` checkout; did not push, create a PR, remove worktrees, or alter unrelated dirty files.
- The planning helper did not detect phases in the scoped Windows plan (`0/0`) even after adding checkboxes; the on-disk plan itself records all 7 phases complete.
- User authorized continuation to hardware testing. Verified `build/Debug/CMakeCache.txt` has loopback and CAN motor bench modes OFF, found the normal Debug ELF, and enumerated ST-LINK SN `3E3703013212354D434B4E00` without reset/download. Waiting at the physical safety gate before any actuator-affecting operation.
- Operator confirmed the physical safety gate. Rechecked all four automatic-test macros OFF, rebuilt ordinary Debug, flashed and verified the 37.40 KiB ELF at `0x08000000`, then software-reset at 3.19 V. Startup commands motor COAST and servo 1500 µs neutral; centre was visually confirmed normal.
- The first ±5° bench attempt exposed that zero-speed VELOCITY commands entered STANDBY and the existing safety-action path forced servo neutral. Added a constrained static-steering path (STANDBY, fresh command, no fault, both wheel targets zero) while retaining wheel COAST and all stop/estop neutralization.
- Added the default-off `SteeringBench` preset and one-shot loopback firmware. After correcting phase sampling to wait for a complete control period, target result at `0x2000027C` was `magic=0x57EEB001`, `passed=1`, `status=0`, `phase_reached=5`, `groups_sent=245`, `tx_failures=0`, with PWM `1500,1572,1500,1428,1500` us. Restored and verified ordinary Debug firmware; all automatic bench macros are OFF.
- Final read-only RAM check on restored Debug firmware showed `g_servo_pulse_us=1500` µs at `0x2000002A` (read from aligned block `0x20000028`), confirming neutral after the bench.
- Repeated the complete steering bench on operator request; operator confirmed the physical direction is correct (`+` left, `−` right). Readback again showed `passed=1`, `status=0`, and PWM `1500,1572,1500,1428,1500` µs. Normal Debug firmware was flashed back and verified.
- H4 lifted-wheel trial ran with host-derived vectors: straight `0 mrad, 200/200 mm/s`; left `+200 mrad, 183/217 mm/s`; right `−200 mrad, 217/183 mm/s`; final stop. CAN applied sequence advanced, all five faults were zero, PWM and direction pins were correct, and both encoders reported positive motion. The measured turn deltas were left `2568` vs right `2500` counts for the left-turn phase and left `752` vs right `821` for the right-turn phase, opposite the requested differential ordering, so the bench result was `passed=0`, `status=3`. Ordinary Debug firmware was restored immediately; H5 ground testing is not authorized pending investigation.
