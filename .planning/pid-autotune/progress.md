# PID autotune progress

## 2026-08-31

- Completed repository audit and baseline test run: 73 tests passed.
- Recorded pre-existing dirty worktree and preserved all unrelated changes.
- Confirmed no live host CAN backend is installed; selected optional `python-can` with deterministic simulator default.
- Wrote initial design/implementation scope; ready for TDD red phase.
- Added TDD metrics/score tests and implemented `tools/pid/model.py` plus `analysis.py`.
- Added Q8.8 PID transaction codec, deterministic first-order simulator, and optional `python-can` command/feedback transport.
- Added experiment runner with state/history, STOP/finally restoration, safety and communication abort recording, seeded search, default scenarios, and JSON/CSV CLI reporting.
- Added MCU `BSP/pid_tuning.c/.h`, routed `0x123/0x124` through bxCAN, added `0x187` feedback, and refreshed the existing 10 ms controller with runtime gains.
- Full host regression reached 85 passing tests; Debug CMake firmware build passed.
- Added and fixed a right-wheel-only gain update regression discovered during review.
- Verified Python syntax, focused PID tests, full regression (87 tests, all passed), 3-iteration simulator run with 1,440 CSV samples, and the final Debug firmware link.
- Removed temporary simulator output directories; kept only source, tests, docs, and planning records.
- Added read-only `0x188 FB_CONTROL_OUTPUT` feedback so live logs can record actual left/right `Motor_Drive` output; preserved one feedback publication per control cycle.
- Resolved the feedback-structure test fixture regression and reverified the final full suite: 89 tests passed in 37.675 seconds; final Debug link passed with FLASH 37,760 B and RAM 2,512 B.
- Updated encoder velocity conversion for the supplied 33.25 mm wheel radius (208.915 mm circumference); the focused encoder test passed after the expected red failure.
- Installed and verified `python-can 4.6.1`; Windows enumeration found only Bluetooth virtual COM ports, so no real CAN channel is currently available for motor motion.
- Added live PID transaction acknowledgement validation and runtime health/diagnostic gating; live trials now require accepted `0x187`, valid velocity/output, and safe `0x181`/`0x186` feedback.
- Verified the normal Debug image on the connected STM32F405/407-class board via ST-LINK after the wheel-radius change; download and verification succeeded.
- Hardened experiment setup failures so a candidate PID acknowledgement/communication error is stopped, recorded, and cannot replace `current_pid`; full regression now passes 91 tests.
- Attempted the real `pcan` channel `PCAN_USBBUS1` without sending frames; it is blocked by the missing `PCANBasic` driver and no physical CAN adapter was present in the Windows device list.
- Created project `.venv` with `python-can==4.6.1` and `pyserial==3.5`; CH340 `COM16` was opened in SLCAN mode for a receive-only probe, but no valid SLCAN frame could be decoded (`Could not read from serial device`). No CAN data frame was transmitted.
- Confirmed that the connected TJA1050 is only a CAN transceiver and cannot be made into a host CAN controller by installing Python packages. Built and flashed the existing MCU internal bxCAN loopback image with the ARM toolchain; RAM result at `0x200001BC` reported magic `0xBCA11B00`, `passed=1`, `status=0`, `hal_error=0`, RX `StdId=0x321`, DLC 8, and matching payload `A5 5A 10 01 23 45 67 89`.
- Restored and verified the normal Debug firmware; loopback self-test is OFF and no motor-control test image remains active.
