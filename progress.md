# Progress

## 2026-09-02 open-loop scan follow-up
- Added MCU-local `OPEN_LOOP_SCAN` before PID candidates, using fixed `40/60/75/100‰` positive-direction breakaway points under the existing L1 envelope.
- Scan samples are excluded from candidate scoring; no response produces `NO_MOTION` abort and cannot update best-known-safe.
- Verification: `190/190` Python tests; Debug and AutotuneSafe builds pass. Live ST-LINK request did not produce a valid new session; ordinary Debug was restored.

## 2026-08-31
- Started PID automatic tuning work.
- Read the required workflow skills; brainstorming requires project-context discovery and design approval before implementation, while TDD will govern each new executable behavior.
- Audited repository structure, Git status, recent commits, and PID/CAN/motor/test matches.
- Confirmed substantial pre-existing uncommitted changes and recorded them as out of scope.
- Confirmed reusable firmware CAN/PID/encoder/bench/safety modules exist, but no host-side PID autotune runner exists and the frozen CAN V1 protocol has no PID-gain command.
- Added a simulator-first host autotune stack under `tools/pid/`, with deterministic search, safety-aware experiment history, and CSV/JSON reports.
- Added the volatile maintenance-only `0x123/0x124` PID gain transaction and `0x187` acknowledgement; gains are consumed by the existing `chassis_control` PID instances.
- Verified 85 Python tests and a Debug firmware build after integration; found and fixed right-wheel-only gain isolation during regression testing.

## 2026-08-22
- Loaded required `using-superpowers`, `brainstorming`, and `planning-with-files` workflows.
- Ran planning session catch-up; no unsynchronized prior session was reported.
- Inspected repository root and git status.
- Confirmed the worktree is dirty and established a read-only boundary for existing firmware changes.
- Started phase 1: context discovery.
- Read the CAN protocol attachment; detected a PowerShell default-encoding mismatch and retained the unambiguous protocol structure and values.
- Enumerated relevant project files, confirmed no repository guidance file, and inspected the single initial commit.
- Re-read the protocol explicitly as UTF-8 and confirmed all safety, timing, feedback, and ownership rules.
- Inspected CubeMX peripheral assignments and the current host-side CAN protocol tests.
- A parallel source query failed because Windows wildcard paths were passed literally to `rg`; recorded the error and changed the search strategy.
- Inspected current CAN symbols/diff and motor/encoder implementations.
- Confirmed the control loop is not implemented yet and recorded the timer mapping inconsistency plus driver/API gaps.
- Inspected PWM and PID layers plus generated timer setup; identified encoder counter, PWM frequency, steering resource, and PID timing gaps.
- Confirmed the latest Keil build log is clean and all 6 current CAN protocol host tests pass.
- Confirmed none of the actuator, encoder, or PID APIs are called by the application and identified duplicate PWM modules.
- Completed repository context discovery; moving to requirements clarification.
- User accepted the browser visual companion for later architecture/state/timing diagrams.
- Session catch-up found only the just-completed visual-companion exchange; no design decisions were missing.
- User selected a standard PWM steering servo directly driven by STM32; exact model/pulse limits and output pin remain to be confirmed.
- User accepted `PB6 / TIM4_CH1` for the steering servo PWM output.
- User fixed CAN1 at 500 kbit/s, with matching ROS configuration and short-bus wiring constraints.
- User chose centralized wheel/encoder calibration and required `calibration_invalid` to remain latched until bench calibration is complete.
- User selected tiered zero-output behavior: coast for normal/safe/timeout stops and brake for E-stop.
- User requested reservation of a dedicated physical E-stop GPIO rather than relying solely on CAN.
- Rejected the proposed `PC13` E-stop assignment because that pin is not exposed on the actual board.
- User assigned the physical E-stop input to `PE1 / EXTI1`.
- Identified the motor driver as TB6612FNG; official data confirms 20 kHz PWM support and requires explicit coast/brake control rather than PWM-zero semantics.
- User chose to retain the original TB6612 wiring and not add an MCU-controlled STBY line.
- Completed the hardware requirements clarification needed for architectural planning.
- User approved architecture approach A (bare-metal layered controller with fixed periodic scheduling).
- User approved the detailed visual architecture boundary (`architecture-approved` browser choice).
- User approved the safety state machine (`state-machine-approved`) and asked to continue text-only.
- User approved the scheduling/data-flow section and asked where PID executes; fixed it in the main-loop 10 ms wheel-control task.
- User corrected the module migration strategy: existing PID, encoder, and motor modules are closest to the current hardware/control implementation and must be reused. Updated the design to evolve these files in place rather than create parallel replacements.
- User approved the fault detection and CAN feedback semantics section.
- User approved the testing and staged rollout section.
- Wrote the complete design specification to `docs/superpowers/specs/2026-08-22-stm32-bottom-controller-design.md`.
- Self-review found no placeholders or contradictory decisions; UTF-8 and `git diff --check` validation passed.
- Initial commit attempt was blocked by read-only `.git`; scoped escalation succeeded.
- Committed only the design spec as `34b93b3` (`docs: add STM32 bottom controller design`).
- Waiting for user review before invoking `writing-plans`.
- Verified the newly generated CMake setup on request. Fresh Debug configure succeeds outside the managed sandbox; Debug build fails at `main.c` because BSP include paths/sources are absent from CMake.
- Confirmed the in-sandbox configure hang is an execution-environment Ninja process-notification issue using a minimal diagnostic build, not a project or compiler failure.
- Fixed the CMake build by adding the Keil-equivalent BSP/Hardware source set and include directories in the top-level user-owned `CMakeLists.txt`; left the CubeMX-generated CMake subproject unchanged.
- Excluded duplicate `pwm_app.c` and retained `bsp_pwm.c`, matching the Keil project and avoiding duplicate symbols.
- Scoped `-Wno-missing-braces` to the intentionally flat font tables in `Hardware/OLED_Data.c`; all other sources retain `-Wall`.
- Fresh clean Debug and Release CMake builds now pass with 0 errors and 0 warnings. Existing 6 CAN protocol tests pass. Both ELF/map files and compile databases were generated, with all 10 expected BSP/Hardware sources present.
- Configured VS Code run/debug support:
  - `.vscode/tasks.json` now provides Debug/Release configure and build tasks plus `STM32: Flash & Run Debug`.
  - `.vscode/launch.json` now provides `STM32: Build, Flash & Debug` and `STM32: Attach to Running Target` using Cortex-Debug with ST-LINK.
  - `.vscode/settings.json` now points CMake/Cortex-Debug/C/C++ IntelliSense at the CubeCLT toolchain and Debug compile database.
- Hardened VS Code task commands to absolute CubeCLT paths so VS Code launched outside the terminal does not depend on PATH.
- Verified VS Code-equivalent Debug configure and build outside the sandbox; Debug ELF links successfully with FLASH 18384 B and RAM 1960 B.
- Verified the VS Code flash task equivalent with `STM32_Programmer_CLI`: ST-LINK SN 6 connected at 3.24 V, STM32F405/407-class target detected, ELF programmed, verified, and reset successfully.
- Verified the ST-LINK GDB debug chain with a Cortex-Debug-like server invocation and `arm-none-eabi-gdb`: target halted in `Reset_Handler`, PC/SP readable, `main` symbol resolved at `0x080030cc`, and the flash vector table read from `0x08000000`.
- Confirmed no `ST-LINK_gdbserver` process was left running after validation.
- User asked to resolve the duplicate `bsp_pwm.c` / `pwm_app.c` ownership problem before continuing with the rest of the controller.
- Confirmed the actual project builds compile `BSP/bsp_pwm.c` and `BSP/bsp_pwm_driver.c`, while old application includes still referenced `pwm_app.h`.
- Added `tests/test_pwm_ownership.py` first; the ownership tests failed as expected because `pwm_app.c` also defined the PWM API, `pwm_app.h` duplicated the handle/API declarations, and the active chain included `pwm_app.h`.
- Consolidated PWM ownership by changing `Core/Src/main.c` and `BSP/bsp_motor.h` to include `bsp_pwm.h`, deleting the duplicate `BSP/pwm_app.c`, and reducing `BSP/pwm_app.h` to a compatibility include of `bsp_pwm.h`.
- Verified `python -m unittest discover -s tests -p test_pwm_ownership.py -v`: 3/3 PWM ownership tests pass.
- Verified `python -m unittest discover -s tests -v`: 9/9 tests pass, including the existing 6 CAN protocol tests.
- Found the interrupted build left idle `cmake` and `ninja` processes; stopped those residual processes before continuing verification.
- Verified `cmake --build build\Debug --target ros` outside the sandbox: Debug ELF links successfully with FLASH 18576 B and RAM 2032 B.
- User asked to test whether bxCAN loopback works before continuing implementation.
- Added `tests/test_bxcan_loopback_selftest.py` first; it failed as expected because no loopback self-test module, CMake option, or macro-gated `main.c` path existed.
- Implemented `BSP/bsp_bxcan_loopback.c/.h`, exposing `g_bxcan_loopback_result` for debugger/RAM inspection and `BSP_BXCAN_RunLoopbackSelfTest(&hcan1)` for an internal CAN loopback transmit/receive check.
- Added CMake option `BSP_BXCAN_RUN_LOOPBACK_SELF_TEST`, default OFF, and wired `main.c` so the self-test firmware runs only when that macro is enabled.
- Verified `python -m unittest discover -s tests -p test_bxcan_loopback_selftest.py -v`: 3/3 loopback structure tests pass.
- Verified `python -m unittest discover -s tests -v`: 12/12 tests pass.
- Built normal Debug firmware with self-test OFF: `cmake --build build\Debug --target ros` links successfully with FLASH 18576 B and RAM 2032 B.
- Configured Debug with `-DBSP_BXCAN_RUN_LOOPBACK_SELF_TEST=ON`, built loopback self-test firmware, flashed it via `STM32_Programmer_CLI`, and verified download successfully.
- Initial GDB-server read was invalid because GDB timed out on `localhost:61234` and printed zero-initialized values; confirmed symbol `g_bxcan_loopback_result` is at `0x200001B0`.
- Read target RAM directly with `STM32_Programmer_CLI -r32 0x200001B0 64`; loopback result reported `magic=0xBCA11B00`, `passed=1`, `status=0`, `hal_error=0`, TX/RX `StdId=0x321`, `DLC=8`, and matching payload `A5 5A 10 01 23 45 67 89`.
- Reconfigured Debug with `-DBSP_BXCAN_RUN_LOOPBACK_SELF_TEST=OFF`, rebuilt normal firmware, flashed it back to the board, and verified self-test is OFF in `build/Debug/CMakeCache.txt`.
- Confirmed no `ST-LINK_gdbserver`, `arm-none-eabi-gdb`, `STM32_Programmer_CLI`, `ninja`, or `cmake` processes remained after verification.
- User asked to implement the practical bxCAN protocol path for controlling the car.
- Scoped the first implementation slice to protocol-to-rear-motor execution only: accepted CAN commands enter a new `chassis_control` layer and then call explicit motor modes; encoder/PID closed-loop control remains a later slice.
- Added `tests/test_chassis_control.py` first; it failed as expected because `BSP/chassis_control.c` did not exist.
- Implemented `BSP/chassis_control.c/.h`, with `Chassis_ControlInit()` and `Chassis_ControlProcess(now_ms)`.
- Updated `main.c` to call `Chassis_ControlInit()` and `Chassis_ControlProcess(HAL_GetTick())` in normal firmware while preserving the macro-gated bxCAN loopback self-test path.
- Extended `bsp_motor.c/.h` with explicit TB6612 actions: `Motor_Init`, `Motor_Drive`, `Motor_Coast`, `Motor_Brake`, `Motor_CoastAll`, and `Motor_EmergencyBrakeAll`; retained `Motor_SetPWM()` as a compatibility wrapper.
- Mapped VELOCITY commands open-loop from `±3000 mm/s` to signed PWM `±1000`; STOP/SAFE_STOP/command timeout coast both rear wheels; E-stop brakes both rear wheels.
- Added `BSP/chassis_control.c` to the CMake source list.
- Verified `python -m unittest discover -s tests -p test_chassis_control.py -v`: 4/4 chassis control tests pass.
- Updated the bxCAN loopback structure test to expect `main.c` to call `Chassis_ControlInit()` in the normal path instead of directly calling `BSP_BXCAN_Init()`.
- Verified `python -m unittest discover -s tests -v`: 16/16 tests pass.
- Verified `cmake --build build\Debug --target ros` outside the sandbox: Debug build links successfully with FLASH 21000 B and RAM 2136 B.

## 2026-08-24
- Compared the upper ROS2 B-stage plan with the STM32 design and current implementation; identified the resolved 500 kbit/s bitrate decision, distinct host/MCU timeout semantics, and incomplete closed-loop/safety/feedback implementation.
- Re-ran the full host suite: 16/16 tests pass; the Debug target remains current and builds successfully.
- User requested freezing the CAN protocol before designing RK3588-to-STM32 USB-DFU OTA.
- Added `docs/CAN_PROTOCOL.md` as the normative CAN V1 source of truth: 500 kbit/s, frame layouts, units, atomic command rules, sequence freshness, safety/recovery, feedback grouping, watchdog thresholds, fault codes, calibration ownership, conformance vectors, and current firmware gaps.
- Kept USB DFU outside the CAN control protocol and started requirements clarification for a separate maintenance-plane OTA design.
- Checked ST AN2606 for STM32F407 ROM bootloader resources. Confirmed ROM USB DFU uses PA11/PA12, which currently belong to CAN1 and are physically connected to the CAN transceiver; OTA architecture now depends on whether board wiring can change.
- Evaluated moving vehicle CAN to CAN2. Rejected it as the default because PB5/PB6 conflicts with steering PB6 and PB12/PB13 conflicts with TB6612 direction outputs; identified CAN1 remap to free PB8/PB9 as the cleaner way to reserve PA11/PA12 for USB DFU.
- Confirmed the host is ElfBoard ELF2, not an unspecified RK3588 carrier. Public ELF2 documentation lists 40-pin and 20-pin expansion headers plus USB host interfaces; exact GPIO lines still require the ELF2 pin-allocation table/device tree.
- User selected SHA-256 + ECDSA P-256 signed images with the public key fixed in the Bootloader, plus watchdog-supervised trial boot and rollback.
- Corrected the OTA architecture after checking ST AN4701/RM0090: RDP1 prevents ROM DFU from modifying main Flash, so secure production OTA must use a custom Flash-resident USB DFU/IAP Bootloader; ROM DFU cannot be the normal updater under RDP1.
- User approved the custom USB DFU/IAP Bootloader architecture for production OTA.
- User approved strict anti-rollback with a separate maintenance key for deliberate downgrade.
- User approved the sector-aligned 32 KiB Bootloader, dual 16 KiB metadata, 192 KiB slot A, and 256 KiB slot B partition, with a common 192 KiB release-image cap.
- User approved separate IWDG execution supervision and delayed application confirmation for trial images.
- User approved the `2 s / 30 s / 3 attempts` watchdog and trial-confirmation parameters.
- User approved CAN STOP plus independent force-Bootloader GPIO entry, with automatic DFU fallback when no valid confirmed image exists; no new CAN maintenance command will be added.
- User approved JSON distribution metadata plus a fixed MCU descriptor. Corrected the trust boundary so the publishing system signs the exact MCU binary descriptor rather than allowing ELF2 to convert signed JSON into a new header.
- Identified the absolute-link-address constraint for internal A/B execution; selecting the inactive slot requires slot-specific builds unless a more complex copy/relocation architecture is chosen.
- Wrote the complete OTA design report covering ELF2/Linux, custom USB DFU, A/B partition, fixed descriptor/signature, ECDSA/SHA-256, RDP/WRP, watchdog trial confirmation, rollback, safe entry, power, testing, implementation phases, and unresolved hardware values.
- Self-reviewed the OTA report and clarified custom-Bootloader entry, pre-jump trial-attempt persistence, slot selection order, and IWDG feeding boundaries.
- Added the required post-update firmware-identity feedback boundary after confirming current CAN V1 heartbeat alone cannot prove which signed slot/version is running.
- Verified the design artifacts with `git diff --check`; the complete existing Python suite remains 16/16 passing. No firmware behaviour was changed by the report work.

## 2026-08-30
- User selected the minimum runnable closed-loop slice before the full safety/calibration implementation.
- Wrote the execution plan to `docs/superpowers/plans/2026-08-30-minimum-runnable-closed-loop.md`.
- Added host tests for the new encoder sample API, fixed-dt PID behavior, and 100 Hz closed-loop chassis scheduling/arbitration.
- Reworked `BSP/encoder.c/.h` so motor 1 samples TIM1 and motor 2 samples TIM2, exposes trusted delta/velocity/64-bit accumulation, starts encoder mode through `Encoder_Init()`, and keeps `Encoder_Get()` as a compatibility wrapper.
- Updated TIM1/TIM2 generated configuration and `ros.ioc` to use no encoder prescaler, full counter ranges, and `TIM_ENCODERMODE_TI12`.
- Reworked `BSP/PID.c/.h` with `PID_Reset()` and `PID_UpdateDt()`, preserving `PID_Update()` compatibility while adding explicit `dt`, output clamping, and conditional-integration anti-windup.
- Reworked `BSP/chassis_control.c` so velocity commands are stored as targets and executed at the 10 ms control step through encoder feedback and per-wheel PID output. STOP, SAFE_STOP, command timeout, zero targets, invalid encoder samples, and E-stop clear controller state before COAST/BRAKE actions.
- Updated `tests/test_chassis_control.py` from immediate open-loop expectations to closed-loop control-step expectations.
- Verified `python -m unittest discover -s tests -v`: 41/41 tests pass.
- Verified `cmake --build build\Debug --target ros`: Debug firmware links successfully with FLASH 23832 B and RAM 2176 B.
- User confirmed the TB6612 forward/reverse GPIO truth table has already been tested successfully on hardware, so the next vehicle test does not need to block on direction-pin validation.
- Added the first CAN feedback snapshot from the 10 ms closed-loop step: trusted encoder velocity and accumulated wheel position are published with freshness flags; an untrusted wheel clears its own validity flags while both wheels coast and both PIDs reset.
- Added host coverage for valid and partially invalid feedback snapshots. Verified `python -m unittest discover -s tests -v`: 43/43 tests pass. Verified `cmake --build build\Debug --target ros`: Debug target is up to date and exits successfully.
- Connected to the vehicle controller through ST-LINK (SN `3E3703013212354D434B4E00`, target STM32F405/407-class, 512 KiB flash, 3.20 V) and flashed the normal Debug `build/Debug/ros.elf`; download verification and software reset both succeeded.
- Added a closed-loop board bench test behind the existing `CAN_MOTOR_BENCH_TEST` opt-in: 600 mm/s two-wheel forward, STOP, reverse, STOP, watchdog COAST, and E-stop, with per-phase encoder count deltas and direction checks recorded in RAM. `build/BenchCan/ros.elf` builds successfully; the normal Debug image remains motion-free.
- First BenchCan run showed real motion but failed direction validation because TIM2 raw counts were inverted: forward deltas were left `+6790` / right `-6895`, reverse deltas were left `-6927` / right `+6943`. Added the vehicle-direction polarity correction for the right encoder.
- Reflashed and reran BenchCan. Board RAM result at `0x200001F0` reported `passed=1`, `status=0`, `phase_reached=6`, `groups_sent=240`, forward deltas left/right `+6735/+6620`, reverse deltas `-6963/-6758`, watchdog fault `0x0004`, and E-stop fault `0x0001`.
- Flashed the corrected normal Debug image back to the vehicle and verified download successfully. The MCU is ready for manual CAN bring-up; no automatic motion test remains active.
- Added the PE1 physical E-stop module: active-low normally-closed input, EXTI1 assertion event, periodic level re-check, and release without clearing the safety latch.
- Added the minimum `safety_manager`: E-stop priority over faults, timeout `SAFE_STOP`, explicit all-zero RESET STOP recovery, and drive permission only in the normal `DRIVE` state. The module returns an action; `chassis_control` remains the sole motor API owner.
- Configured PE1 as GPIOE pull-up/falling-edge EXTI, enabled `EXTI1_IRQn`, and connected the HAL callback to the physical E-stop module.
- Integrated safety arbitration into the 10 ms chassis path. Physical or CAN E-stop executes TB6612 emergency brake; timeout/stop/fault paths execute coast and reset both wheel PIDs.
- Added feedback snapshot status propagation for physical E-stop and safe-stop states while retaining the latest encoder velocity/position values.
- TDD verification: focused PE1/safety tests passed, then the complete host suite passed 50/50. Normal Debug CMake build passed with FLASH 25576 B and RAM 2224 B.
- Hardware boundary: PE1 electrical assertion/release/reset still requires a powered vehicle test with wheels restrained and no DRIVE command until the operator confirms the switch behavior. No automatic-motion image is active.
- Follow-up remains: full calibration gating, complete frozen safety state machine and fault taxonomy, encoder fault/stall latching, and real-vehicle PID tuning.
- Rebuilt and flashed the normal Debug image containing the PE1 safety path; STM32CubeProgrammer detected the connected STM32F405/407-class target at 3.19 V, downloaded `ros.elf` to `0x08000000`, and completed software reset. `CAN_MOTOR_BENCH_TEST` and loopback self-test are both OFF.
- Extended the board BenchCan test to 11 phases: internal CAN loopback forward/stop/reverse/stop/timeout/CAN-E-stop, automatic RESET STOP, post-reset DRIVE, then physical PE1 assert/release/recovery verification. This removes the need for external bxCAN command injection.
- Built and flashed the extended BenchCan image twice. The recorded runs reached `phase=8/status=3`: automatic phases through CAN-E-stop and RESET recovery completed, but no physical PE1 assertion was detected during the timed wait. The test safely issued STOP and the normal Debug image was flashed back afterward.
- User manually confirmed that pressing PE1 during other automatic test stages stopped the motors. This confirms the physical input-to-brake path in operation, while the dedicated BenchCan phase result remains timing-dependent and was not completed.
- Added `wheel_calibration` gating: normal firmware initializes without valid calibration and remains in `CALIBRATION_REQUIRED + COAST`; only a validated `CalibrationData_t` can authorize DRIVE. BenchCan receives explicit `CALIBRATION_BENCH_DEFAULTS` only in its opt-in build.
- Added host coverage for invalid-calibration inhibition and valid-data re-enable. Full host suite now passes 51/51; Debug links at FLASH 25876 B and RAM 2240 B; BenchCan links at FLASH 29724 B and RAM 2520 B.
- Flashed the updated normal Debug firmware after gating changes. No automatic BenchCan or loopback mode is enabled on the board.
- Added the real `0x122 CMD_CALIBRATION` / `0x185 FB_CALIBRATION` codec, cookie/reserved-byte validation, request routing, 20 ms response scheduling, duplicate/busy/cancel gates, and the non-blocking PRECHECK -> left forward/settle/reverse -> right forward/settle/reverse transaction.
- Added `pending` calibration staging with one-shot active commit after both wheel directions pass; abort, cancel, safety preemption, and validation failure do not replace the previous active calibration. Validation failure is promoted by `safety_manager` to latched `0x000A`.
- Added the independent `BenchCalibration` CMake preset. On the lifted vehicle it ran through internal CAN loopback and reported `passed=1`, `status=0`, transaction `SUCCEEDED`, and exit reason `SUCCESS`; the normal Debug image was then flashed back and verified.
- Host verification after the service integration is `59/59 PASS`. Debug links at FLASH `29908 B` and RAM `2312 B`; BenchCan links at FLASH `33788 B` and RAM `2616 B`; BenchCalibration links at FLASH `32360 B` and RAM `2600 B`.
- Remaining calibration work is richer measured vehicle parameters; the current service validates direction/response and commits the frozen polarity/minimum-start-PWM profile through persistent Flash storage.
- Added dual-slot persistent calibration storage with explicit 32-bit-word serialization, generation selection, CRC32 validation, and a commit marker programmed last. The two slots are Flash sectors 6/7 at `0x08040000` and `0x08060000` on the 512 KiB target.
- Connected persistence to `Wheel_Calibration_Init()` and `Wheel_Calibration_CommitPending()`. Startup loads the newest valid record; a failed or partial write cannot replace the previous active calibration.
- Added host tests for reboot reload, damaged newest-slot fallback, and uncommitted-record rejection. Full host verification now passes `63/63`.
- Debug, BenchCan, and BenchCalibration all build with the application Flash region constrained to `256 KiB`; the largest image uses `36728 B` in that earlier build.
- BenchCalibration was flashed on the lifted vehicle and Flash readback confirmed the committed calibration record. The normal Debug image is restored and remains the board's active image.

## Complete Safety Manager (2026-08-30)
- Expanded `safety_manager` with latched fault priority and evidence APIs for encoder invalidity/direction (`0x0002`), motor stall (`0x000B`), control overrun (`0x000C`), and watchdog reset (`0x0008`). Control overrun selects BRAKE; the other latched device faults select COAST.
- Added the frozen 50 ms physical E-stop release hold before RESET recovery, plus cause-aware encoder recovery before clearing an encoder fault. RESET always re-arbitrates to `STANDBY` or `CALIBRATION_REQUIRED`, never directly to DRIVE.
- Wired actual control `dt`, encoder trust/speed, target speed, and PID PWM observations from `chassis_control` into the safety manager; a newly detected fault is applied before the next motor drive write.
- Full host verification passed `69/69`. The updated BenchCan image completed automatic phases through `phase_reached=8` with `tx_failures=0`; the board was then restored to ordinary Debug. The physical PE1 wait was not exercised in this run because no button press was issued.

## Feedback and Diagnostics (2026-08-30)
- Added backward-compatible read-only `0x186 FB_DIAGNOSTICS` to the immutable 20 ms feedback snapshot. It carries safety state/action, drive permission, calibration gate, command freshness/age, encoder validity, selected control/stall evidence, and CAN error class.
- Added saturating RX-invalid and TX-failure counters plus CAN warning/error-passive/bus-off/TX-failure classification. Production bxCAN polling maps HAL CAN error state into the diagnostic snapshot; invalid command frames and failed TX enqueue operations increment counters.
- Published diagnostics from `chassis_control` on every safety arbitration path, including before the first 10 ms wheel-control sample, without changing existing base feedback publication semantics.
- Host verification: `73/73 PASS`. Debug, BenchCan, and BenchCalibration images all link successfully; the board remains on the ordinary Debug image and was not reflashed for this change.
# 2026-09-01 engineering closeout

- Added RED tests in `tests/test_engineering_closeout.py` for duplicate command sequence, legal Flash slots, TIM3 IRQ removal, TIM6 event path, PB8/PB9 documentation, servo envelope rejection, and TB6612 dead-time state.
- Fixed duplicate/out-of-order command groups so they are ignored without watchdog refresh, target reapplication, applied-sequence update, or protocol-fault injection; natural wrap remains fresh.
- Corrected SAFE_STOP arbitration to `SAFE_STOP + COAST`; recovery remains restricted to a fresh all-zero STOP.
- Moved calibration dual slots to legal F407 sectors 6/7 (`0x08040000/0x08060000`), constrained linker application Flash to 256 KiB, and expanded power-fail-safe records with measured drivetrain parameters.
- Added calibration parameter API and encoder consumption; service derives measured polarity from forward motion and records observed start PWM.
- Removed TIM3 Base IRQ/start and added TIM6 1 kHz ISR event posting with bounded backlog/control-overrun handling; main loop now processes events and feeds a register-level IWDG only after scheduler work.
- Added TB6612 direction reversal dead-time state machine and checked/rate-limited servo angle API; static steering STOP now re-centres servo.
- Added Ackermann bench actual L/R velocity fields for synchronized target/actual/PWM/encoder evidence; updated CAN docs and design notes for PB8/PB9 and storage.
- Focused closeout tests pass; full regression now passes 139 tests with adaptive calibration, snapshot-freeze, and 2 s IWDG checks, Debug firmware build passes (`FLASH 40744 B / 256 KiB`, `RAM 2648 B`).
- H4 follow-up boundary review: ST-LINK sees the 512 KiB F405/407-class target at 3.19 V with the normal Debug image. Added explicit `0xFFFF -> 0x0000` fresh-sequence coverage (11/11 focused closeout tests pass). Existing AckermannBench remains a 5-phase closed-loop smoke test and does not yet export the requested PID P/I/D telemetry or open-loop characterization phases; no motion firmware was flashed.
- Built the opt-in `AckermannBench` preset successfully (`FLASH 43,756 B / 256 KiB`, `RAM 2,840 B`) after adding its explicit calibration bench default. Full host regression after the wraparound and bench-preset checks is 140/140 passing. H4 execution remains gated on physical setup confirmation and richer characterization logging.

# H4 hardware characterization (2026-09-01)

- Operator confirmed the lifted/mechanically restrained setup and reachable physical E-stop. The H4 image was built with automatic bench macros opt-in only; ordinary Debug was restored after the run.
- Manual boundary review: `command_seq` uses unsigned modular distance (`0xFFFF -> 0x0000` fresh, duplicate zero stale); calibration writes the inactive legal Sector6/7 slot and programs CRC/commit marker last; IWDG feed occurs only after a processed scheduler event; PE1 EXTI latches immediately and the next TIM6 event applies the brake/coast action. Power-cut, watchdog-stall reset, and pin-to-PWM latency still require dedicated hardware instrumentation.
- H4 sequence executed: single-wheel open-loop L/R, dual-wheel open-loop 200/200, closed-loop 200/200, 180/220, 220/180, Ackermann 150/250 and 250/150. The target/PWM/PID/safety record completed 8/8 with `passed=1/status=0`, `tx_failures=0`, and no fault code.
- Root cause of the initial no-motion result was a TIM3 initialization regression: deleting the Base IRQ also deleted `HAL_TIM_Base_Init(&htim3)`, so the generated Base MSP clock hook never enabled TIM3. The Base init was restored while the Base IRQ/start remains removed; the H4 rerun showed CCR `2100` at open-loop PWM=500 and nonzero encoder deltas.
- H4 rerun is physically active, but not yet accepted for H5: steady speeds are only a few mm/s and the single-sample `180/220` result is `4/5`, so repeated-window strict differential stability is still unproven. Keep PID/Ackermann mapping unchanged while measuring wheel circumference and minimum-start PWM.
- Servo is PWM-only open-loop; H4 Ackermann entries record steering command plus rear-wheel evidence, not servo feedback. H5 ground testing remains prohibited.
- Latest verification: full Python regression `143/143 PASS`; normal Debug links at `FLASH 41248 B / 256 KiB`, `RAM 2712 B`; H4 image links at `FLASH 44776 B / 256 KiB`, `RAM 3272 B`; board restored to normal Debug via STM32CubeProgrammer.

# H4 follow-up hardware evidence (2026-09-01)

- Repeated H4 windows were compared: `180/220` produced `4/4` in one run and `4/5` in the next; `220/180` produced `5/4` in both. The trend is plausible, but strict differential ordering is not stable at the present few-mm/s bench response, so H5 remains prohibited.
- Open-loop PWM scan at 25/40/50/60/75/100 produced the first bilateral response around PWM 60; PWM 75 was the first more repeatable low response and PWM 100 was the robust observed starting point. No automatic calibration write was made.
- Confirmed configuration values remain `PPR=500`, quadrature factor 4, gear ratio `28.0:1`; effective tire circumference still requires a measured wheel-turn test. Firmware's 33.25 mm radius (208.9 mm circumference) must be reconciled with the 32.5 mm Ackermann design value.
- `IWDG_STALL_TEST` hardware probe passed: result `0x49574447`, `RCC_CSR.IWDGRSTF=1`. `SAFE_STOP_WATCHDOG_TEST` passed: result `0x53414645`, state `SAFE_STOP`, fault `0x0004`, and no IWDG reset after 4 s.
- Sector6/7 power-cut atomicity and PE1-to-PWM/方向脚 oscilloscope latency remain unmeasured hardware gates; do not mark them accepted from host tests alone.
-
## Autotune hard safety layer (2026-09-01)

- Read the existing PID autotune design and implementation. Host simulation/search and MCU volatile gain transactions exist; the requested independent tuning safety envelope does not.
- Confirmed the current code has 10 ms closed-loop PID, ±1000 motor output, TB6612 reversal dead-time, MCU safety manager/watchdog, but lacks staged tuning unlock, absolute/relative gain limits, tuning PWM cap/slew/ramp, autotune-specific stall/saturation/overspeed/oscillation/runtime gates, and the requested abort telemetry.
- Waiting for scope confirmation before presenting the design and writing implementation code, per brainstorming hard gate.
-
## AutotuneSafe prompt loaded (2026-09-01)

- Read the complete attached prompt and reconciled it with the repository. The implementation target is the full host + MCU safety stack, followed by a dedicated manually flashed `AutotuneSafe` lifted-wheel validation image.
- The earlier “do not flash” assumption is superseded: flashing is required for real PID debugging, but only after software verification and only with the dedicated safe profile. Normal Debug/Release remains motion-safe and autotune-disabled.
- Design approval is still the brainstorming gate before source implementation.

## AutotuneSafe TDD implementation (2026-09-01)

- Committed frozen design/plan as `c4a7a02`.
- Completed RED/GREEN cycles for profile isolation (5/5), MCU core bootstrap/level policy (1/1), actuator gate/ramp/slew/reversal (3/3), MCU local abort and session watchdog (2/2), CAN protocol snapshot (3/3), and host L1 runner policy (3/3).
- Added the initial dedicated CMake `AutotuneSafe` preset with all automatic bench macros disabled; no firmware has been flashed or motion started during the software phase.
- Next: run full regression, update compatibility failures, then implement MCU CAN routing and live transport/session telemetry before the final two-profile builds.

## AutotuneSafe final software boundary (2026-09-01)

- Runtime PID transactions in the profile pass through MCU `AutotuneSafe_ValidateCandidate`/`SetCandidate`; abort reads restore gains from `best_known_safe` and reset controller dynamic state.
- MCU emits a sequence-correlated Classic CAN telemetry snapshot on IDs `0x189..0x190`; host codec rejects incomplete or mixed snapshots.
- Added MCU session stillness confirmation, 3 s session runtime / 1 s high-PWM local energy proxy, and persistent in-process cooldown lock for conservative sensorless protection.
- Host live transport sends START/heartbeat/STOP with session/experiment IDs and waits for stillness; search order is bounded P-only -> PI -> optional D, with no random live probing.
- Verification complete: `170/170` Python tests, Debug build, AutotuneSafe build. AutotuneSafe artifact is flashable but not flashed; follow `docs/AUTOTUNE_SAFE_FIRST_L1.md`.

## Automated build/flash/tune/restore checkpoint (2026-09-01)

- Added `tools/pid/automated_runner.py` and TDD coverage. The runner owns the host sequence and artifact generation but cannot replace the MCU actuator gate.
- The runner completed `173/173 PASS`, ordinary `Debug` build, and `AutotuneSafe` build. It detected the connected ST-LINK/STM32F405/407 target.
- Hardware execution stopped before Flash because the configured Windows `socketcan/can0` backend is unavailable (`WinError 10047`). No START, heartbeat, motor command, AutotuneSafe Flash, or Debug restore was issued by this run.
- The run artifact is `autotune_runs/20260901T090312Z/`; status is `BLOCKED_PREFLIGHT`. No independent TB6612 STBY/driver-enable GPIO was found, so any later valid CAN run remains low-energy L1-only and cannot expand the envelope.

## Self-loop PID checkpoint (2026-09-01)

- Added `--self-loop` to the automated runner. It never flashes, opens CAN, or marks hardware motion; it exercises the deterministic host model and bounded P-only → PI → optional D search.
- Full self-loop run completed `SELF_LOOP_PASS`: simulated best PID `Kp=0.25, Ki=0.60, Kd=0.00`, score `674.734925997457`, simulated PWM peak below `150‰`, and no simulated safety abort.
- This is not a real motor/MCU safety or tuning result; `mcu_safety_proven=false`. Real flashing and PID testing remain blocked until a supported CAN device is available.

# MCU-local AutotuneSafe executor (2026-09-01)

- Added `BSP/autotune_safe_local.c/.h` under `AUTOTUNE_SAFE_PROFILE`; it runs fixed L1 single-wheel candidates locally using encoder/PID/Motor/Safety, with target ramp, PWM hard limit/slew through the final gate, stillness, abort recovery, cooling, and a bounded RAM sample ring.
- Added encoder speed evidence (`raw delta`, `dt`, MCU conversion, offline recomputation, counts/rev, circumference) and a C host test proving the two speed paths agree.
- Added `tools/pid/local_swd_runner.py`; default is dry-run. Explicit execution only writes the local request mailbox through ST-LINK and never exposes level, PWM limit, speed limit, or Safety-threshold writes.
- Fixed local telemetry axis mapping, actual gated PWM reporting, explicit `RAMP_DOWN`, core best-safe update only after a complete four-point local candidate pass, and abort-time recovery PID restoration.
- Verification: full Python regression `186/186 PASS`; GDB/MI local SWD tests `6/6 PASS`; Debug and AutotuneSafe builds both pass. No CAN is required for the local request path, but the only physical START attempt was invalidated before mailbox consumption and produced no accepted motor result.
- Generated report: `AUTOTUNE_SAFE_LOCAL_PID_RUN.md`. One left-wheel START attempt was invalidated because the GDB session lost communication before the mailbox was consumed; ordinary Debug was restored. Real encoder sanity, PWM→speed characterization, left/right PID, and dual-wheel L1 validation remain pending; H5 and higher envelope remain prohibited.
- Fixed the GDB/MI monitor after root-cause reproduction: pipe-mode prompt parsing now handles a newline-less `(gdb)` prompt, `mi-async` plus `-exec-interrupt --all` provides bounded same-session polling, and monitor timeout writes STOP before closing.
- A follow-up hardware START was not issued because ST-LINK re-enumeration returned `DEV_CONNECT_ERR` during the preflight read. No mailbox write or motor motion occurred; the last successful image on the board is AutotuneSafe idle, not ordinary Debug.

## AutotuneSafe no-CAN SWD execution retry (2026-09-02)

- Full regression after the local boot handoff, heartbeat, STOP keepalive, and candidate-table fixes: `189/189 PASS`.
- AutotuneSafe and ordinary Debug builds both link successfully. The AutotuneSafe image was flashed and verified at `0x08000000`; left-wheel-only L1 session `2026090211` completed 6 fixed candidates and 625 MCU-local samples without Safety abort.
- Real result is negative and intentionally not accepted as PID: actual encoder speed stayed `0`, peak-to-peak stayed `0`, candidate max PWM was `17/19/21/30/34/34‰`, and all candidates were `passed=0`. No best-known-safe update, promotion, right-wheel run, H4/H5, or aggressive search occurred.
- Root causes fixed during retry: CubeProgrammer `-run` clearing `.bss` request mailbox (profile-only `.noinit` boot handoff), first-heartbeat duplicate sequence, stale zero-command CAN freshness during cooling, and PI/D built-in candidates violating the frozen relative step policy.
- The ordinary Debug image was restored and verified after the negative run; AutotuneSafe remains available as a separately built, flashable artifact only.
- A second identical L1 retry (`2026090212`) again completed safely with 627 samples but zero actual encoder speed and zero candidate passes; ordinary Debug was flashed and verified again afterward.

## Deterministic boot/session validation phase started (2026-09-02)

- Scope frozen to build -> flash/verify -> run -> boot READY -> non-reset attach -> MCU-confirmed session/start -> synthetic smoke capture -> generation/session validation -> artifact -> Debug restore.
- PID search, open-loop PWM scan, actuator fault conclusions, and encoder fault conclusions are paused for this phase.
- Initial code audit shows the existing local SWD implementation is insufficient to prove all lifecycle boundaries; design approval is required before implementation changes.

## AutotuneSafe SWD session contract implementation (2026-09-02)

- Corrected the approved specification for latched terminal states, removal of ambiguous experiment generations, START/sample ordering, seqlock/CRC retry, result magic-last commit, generation scope, and wrap-aware comparison.
- Added `docs/superpowers/plans/2026-09-02-autotune-safe-swd-session-orchestration.md` and executed TDD RED/GREEN for the required race and stale-identity cases.
- Added `BSP/autotune_safe_session.[ch]`, isolated `BSP/autotune_safe_local_smoke.c`, and `Core/Src/main_local_session_smoke.c`; Smoke builds without actuator/control/CAN/PID/encoder sources. ELF `.noinit` is RAM `NOBITS` and forbidden actuator symbols are absent.
- Extended `tools/pid/local_swd_runner.py` with fixed-layout validators, command evidence, no-reset ST-LINK GDB server proof, request magic-last writes, conservative reporting, and a concrete hardware smoke backend. The old legacy local-request path remains compatible for now.
- Focused verification currently passes: session contract `10`, runtime C harness `2`, runner orchestration `4`, build isolation `4`, legacy SWD `8`. The CMake compatibility fix is applied; fresh full regression later reached `210/210`.
- `--dry-probe` passed for installed CubeProgrammer/GDB/GDB server help/version. No `--execute`, real PWM, PID, H4/H5, or motor/encoder fault judgment was performed in this phase.

## Corrected session contract and smoke checkpoint (2026-09-02)

- [x] Correct the six race/consistency issues and add all requested RED tests; focused contract `10/10` and runtime `2/2` pass.
- [x] Build isolated LocalSessionSmoke; `.noinit` is RAM `NOBITS`, actuator/control/PID/encoder/CAN symbols are absent, and the smoke target now restores interrupts after inherited IRQ cleanup.
- [x] Implement runner order, no-reset attach proof, stable snapshot retry, generation/session validation, final-magic result capture, artifact persistence, and fail-closed Debug restore retry.
- [x] Software stress smoke passes 50 repeated lifecycle cycles with stale/duplicate request protection and latched-result immutability.
- [x] A live session artifact passed full boot -> attach -> READY -> ARMED -> RUNNING -> 32 samples -> COMPLETE_LATCHED -> full readback -> ACK -> Debug restore.
- [ ] Final hardware 10/10 with per-session artifacts: not accepted yet. The current ST-LINK connection returns `DEV_USB_COMM_ERR` at direct CubeProgrammer connect/flash, so the batch stopped before session creation.
- [ ] Latest rerun reached `9/10`: sessions 1-9 have complete artifacts; session 10 completed session/result capture but failed Debug restore with three `DEV_USB_COMM_ERR` attempts. Hardware stress was correctly not started.
- [x] Hardened `local_swd_runner.py` restore lifecycle: stage-specific failure codes, explicit GDB detach/teardown, separate process logs, command timeline/process snapshot, bounded ST-LINK reopen/readiness probes, retry evidence, timeout evidence, and primary-error/cleanup-error separation without session rerun.
- [x] Added restore failure-injection and timeout evidence coverage; runner orchestration focused tests now pass `11/11`. Latest full regression passes `217/217 OK` (`56.294 s`).
- [x] Executed hardware stress with the required stop-at-first-failure rule: sessions 1-5 passed with artifacts; session 6 completed session/result capture but all three ST-LINK reopen attempts returned `DEV_USB_COMM_ERR`, classified as `RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT`; accepted stress result is `5/50`, not pass.
- [ ] Actuator admission: blocked. No real PWM scan, PID, H4/H5, or motor/encoder fault judgment until hardware session acceptance is re-established and the required 10/10 + stress gate is complete.

## SWD session rerun (2026-09-03)

- [x] Completed one standalone LocalSessionSmoke transaction with full result/ring/ACK and successful ordinary Debug restore; artifact saved under `autotune_runs/20260903T071454Z_session_smoke_001_91f5dab8/`.
- [ ] Formal 10-session acceptance remains incomplete: sessions 1-3 passed, session 4 completed the session contract but failed all three Debug-restore ST-LINK reopen probes with `DEV_USB_COMM_ERR`; strict result `3/10`.
- [x] Stopped immediately at the first failure and preserved the complete session-4 artifact at `autotune_runs/20260903T071552Z_session_smoke_004_0721fea4/`.
- [ ] Post-failure ordinary Debug recovery was attempted once plus three bounded retries; all four CubeProgrammer flash/verify/run attempts returned `DEV_USB_COMM_ERR`. A subsequent read-only dry probe enumerated ST-LINK, but Debug image/state is not proven and must be treated as unknown.
- [ ] Actuator admission remains blocked; do not run PWM, PID, H4/H5, or motor/encoder diagnosis from this batch.

## CAN V1 bitrate migration (2026-09-03)

- [x] Verified STM32F407 clock path as `SYSCLK=168 MHz`, `APB1=HCLK/4`, `CAN kernel clock=42 MHz`; calculated final timing `6 × (1+11+2)` for exact `500000 bit/s` and approximately 85.7% sample point.
- [x] Updated `ros.ioc`, `Core/Src/can.c`, `BSP/bsp_bxcan.h`, `tools/pid/transport.py`, and `tools/pid/automated_runner.py` to make 500 kbit/s the sole CAN V1 rate.
- [x] Added `tests/test_can_bitrate.py`; RED observed against the former host implementation, then GREEN with 5/5 timing/default/document tests. Existing 10 ms command-pair and 100 ms watchdog semantics remain unchanged.
- [x] Updated CAN protocol, chassis, H4, AutotuneSafe, design, bench, findings, progress, and task-plan documentation; documented normal estimated load 13% and AutotuneSafe estimated load 27.3% at 500 kbit/s.
- [ ] External CAN hardware validation: endpoint bitrate agreement, error-passive/bus-off counters, command/feedback/watchdog/E-stop, termination, wiring/common-ground/transceiver checks, and long-duration communication.
- [x] Verification complete for the software/configuration phase: CAN bitrate tests `5/5`; full regression `222/222 OK` (`45.067 s`); Debug build `FLASH 41248 B / RAM 2712 B`; AutotuneSafe build `FLASH 54908 B / RAM 13200 B`.
- [ ] Physical CAN acceptance remains pending because no external CAN hardware/vehicle bus was available in this run; do not infer bus health from the software builds.

## 2026-09-07 标定失败恢复修复

- [x] 新增标定失败复位恢复测试，先在旧逻辑上确认复现，再实现最小修复。
- [x] `Wheel_Calibration_Service_ClearFailedValidation()` 与 Safety Manager 的 0x000A 专用 reset 路径已接通。
- [x] 相关测试 `35/35`、全量回归 `224/224`，Debug 编译通过；未执行烧录。

## 2026-09-07 扩展板接口表

- 新增 `docs/STM32F407_EXPANSION_BOARD_PINOUT.md`，按当前 CubeMX/Debug 固件整理 CAN、双电机、双编码器、舵机、急停、SWD、USB 预留、供电和引脚冲突。
- 已执行 `git diff --check` 与关键引脚检索，文档落盘正常；未修改固件引脚配置。
