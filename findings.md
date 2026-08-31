# Findings

## Repository State
- STM32CubeMX/Keil project rooted at `D:\STM32cubemx\Project\ros`.
- Main directories: `BSP`, `Core`, `Drivers`, `Hardware`, `MDK-ARM`, and `tests`.
- Existing uncommitted changes include `BSP/bsp_bxcan.c`, `BSP/bsp_bxcan.h`, `Core/Src/main.c`, tests, IDE settings, build outputs, and `debug.log`.
- These changes are user-owned and must not be reverted or overwritten.
- No `AGENTS.md` was found in or above the project subtree.
- Git history contains one commit (`51703b0`, 2026-08-22, `first commit`), so the uncommitted files are important current context.

## CAN Protocol V1 (confirmed structure)
- Classic CAN 2.0A, 11-bit standard identifiers, fixed DLC 8, little-endian fields, `protocol_version = 1`.
- Host commands are an atomic two-frame group: `0x120` steering plus `0x121` rear-wheel targets, paired by `command_seq` and identical `mode_flags`.
- MCU feedback is a five-frame cycle: `0x180` status, `0x181` health, `0x182` rear velocity, `0x183` left position, and `0x184` right position, paired by `feedback_seq`.
- Default timing: 20 ms host control, 20 ms MCU feedback, 10 ms command-pair completion window, 100 ms MCU command timeout.
- Safety priority: E-stop, safe stop, fault reset, then control mode. A single valid-format frame carrying E-stop must zero rear-wheel output immediately without waiting for its pair.
- MCU duties include strict command validation, steering-angle calibration/mapping, dual rear-motor closed loop, encoder sampling, local timeout/safety state, latched faults, and periodic feedback.
- R3X V1 has no steering position sensor; firmware must not fabricate steering feedback.
- Protocol test vectors include forward/reverse command groups, E-stop, normal feedback, command sequence wrap, and device time wrap.

## CubeMX and Existing Test Baseline
- MCU is STM32F407VET6 at 168 MHz using STM32Cube FW_F4 V1.28.3 and Keil MDK-ARM project generation.
- CAN1 uses PA11/PA12, automatic bus-off recovery enabled, receive FIFO0/FIFO1, SCE, and TX interrupts enabled.
- CubeMX currently calculates CAN1 at **1,000,000 bit/s** (`Prescaler=3`, `BS1=10TQ`, `BS2=3TQ`, `SJW=1TQ`); this differs from the protocol's non-binding 500 kbit/s bring-up suggestion and needs an explicit integration decision.
- TIM1 and TIM2 are configured as the two encoder interfaces; TIM3 CH1/CH2 are PWM outputs. Four GPIO outputs on PB12-PB15 likely provide H-bridge direction/control.
- Existing host-side Python tests compile `bsp_bxcan.c` directly with GCC and cover matching command pair decode, all five feedback vectors, single-frame immediate E-stop, inconsistent sequence rejection, 100 ms command timeout, and exact 10 ms pair-window acceptance.
- Current tests do not yet cover motor/encoder/steering control, state-machine transitions, most invalid protocol fields, fault reset policy, rollover, CAN transmit failure, or end-to-end scheduling.

## Current Firmware Shape
- `main.c` currently only calls `BSP_BXCAN_Init()` after CubeMX initialization and `BSP_BXCAN_Process(HAL_GetTick())` in the superloop. It does not start PWM/encoder peripherals or consume commands into actuators.
- `bsp_bxcan` already implements much of the protocol codec and transport lifecycle: two-frame pairing, strict header/version/reserved checks, ranges, E-stop short circuit, command timeout, five-frame feedback serialization, CAN RX callback, and non-blocking TX mailbox draining.
- The CAN API currently combines hardware transport, protocol codec, command assembler, timeout state, safety flags, fault storage, and feedback scheduler in one module; this is a useful bring-up implementation but too broad as the long-term controller boundary.
- `Motor_SetPWM()` drives PB12-PB15 direction pins and two PWM handles. It accepts raw signed PWM but has no explicit clamp, neutral/brake/coast policy, initialization contract, or driver fault input.
- `Encoder_Get()` uses floating-point conversion and hard-coded PPR, wheel radius, sample period, and gear ratio. It only returns instantaneous velocity and keeps private previous counts, so it cannot directly provide protocol cumulative position or robust validity diagnostics.
- Hardware mapping inconsistency: `Encoder_Get(1)` reads TIM3, but CubeMX configures TIM3 for PWM and TIM1/TIM2 as encoder interfaces. The left encoder mapping must be corrected/confirmed before controller integration.
- Both encoder timers are currently generated with prescaler 167 and auto-reload 999, while `Encoder_Get()` assumes 16-bit modulo-65536 deltas. The timer configuration and software rollover model are inconsistent and the short counter period is unsuitable for robust cumulative position.
- TIM3 CH1/CH2 run from an 84 MHz timer clock with prescaler 167 and period 999, yielding approximately 500 Hz PWM (not 1 kHz); both channels are already assigned to rear motors. No configured PWM timer/channel remains for the steering servo.
- The PID is a generic float position-form controller whose integral omits explicit `dt` and anti-windup. It may be adapted for initial bench work only after giving the loop an explicit fixed period and safe reset/saturation behavior.
- `bsp_pwm.c` and `pwm_app.c` duplicate the same high-level PWM implementation. Keil compiles `bsp_pwm.c`, while `main.c` and `bsp_motor.h` include `pwm_app.h`; the identical ABI happens to link, but ownership and naming should be consolidated before control integration.
- PWM ownership has been consolidated: `BSP/bsp_pwm.c` is now the only implementation of `PWM_Register`, `PWM_Start`, `PWM_Stop`, `PWM_SetDuty`, `PWM_StartRamp`, `PWM_StopRamp`, and `PWM_Process`. `BSP/pwm_app.c` was removed, while `BSP/pwm_app.h` remains only as a compatibility include of `bsp_pwm.h`.
- Active firmware sources now include `bsp_pwm.h` directly from `Core/Src/main.c` and `BSP/bsp_motor.h`, so the current control chain no longer names `pwm_app.h`.
- `chassis_control.c` now owns the first protocol-to-actuator execution slice: it initializes CAN and motor BSP, calls `BSP_BXCAN_Process(now_ms)`, drains accepted commands with `BSP_BXCAN_GetCommand()`, and maps them to motor actions without letting `bsp_bxcan` directly own actuator hardware.
- First bring-up mapping is intentionally open-loop: `rear_*_velocity_mmps` maps linearly from `±BSP_BXCAN_MAX_REAR_VELOCITY_MMPS` to signed PWM `±1000`. STOP, SAFE_STOP, and command timeout map to `Motor_CoastAll()`. CAN E-stop maps to `Motor_EmergencyBrakeAll()`. This is a temporary execution bridge until encoder/PID closed-loop control is added.
- `bsp_motor.c` now exposes explicit TB6612 modes: `Motor_Drive`, `Motor_Coast`, `Motor_Brake`, `Motor_CoastAll`, and `Motor_EmergencyBrakeAll`. Legacy `Motor_SetPWM()` remains as a compatibility wrapper over `Motor_Drive()`.
- The latest Keil build log reports 0 errors and 0 warnings. This proves the current selected sources link, but not that motor/encoder hardware is initialized or controlled.

## Verification Baseline
- `python -m unittest discover -s tests -v`: 6/6 CAN protocol tests pass on 2026-08-22.
- `python -m unittest discover -s tests -v`: 9/9 tests pass after adding 3 PWM ownership checks on 2026-08-22.
- `python -m unittest discover -s tests -v`: 12/12 tests pass after adding 3 bxCAN loopback self-test structure checks on 2026-08-22.
- `python -m unittest discover -s tests -v`: 16/16 tests pass after adding 4 CAN-to-chassis execution checks on 2026-08-23.
- Board-level bxCAN internal loopback self-test passed on 2026-08-22 using ST-LINK SN 6 at 3.23-3.24 V. Test firmware sent standard frame `StdId=0x321`, `DLC=8`, data `A5 5A 10 01 23 45 67 89`; target RAM result at `0x200001B0` reported `magic=0xBCA11B00`, `passed=1`, `status=0`, `hal_error=0`, received `StdId=0x321`, `DLC=8`, and matching payload.
- The loopback self-test path is gated by CMake option `BSP_BXCAN_RUN_LOOPBACK_SELF_TEST` and compile definition of the same name. It is OFF in `build/Debug/CMakeCache.txt` after verification, and the board was flashed back to the normal Debug firmware.
- No call sites exist for `HAL_TIM_Encoder_Start`, `Encoder_Get`, or `PID_Update`; the current implementation can drive rear motor PWM open-loop from accepted CAN commands, while encoder/PID feedback remains unimplemented and feedback remains default zero/invalid.

## Required Hardware Decisions
- Steering uses a standard PWM servo driven directly by STM32 (user choice A), assigned to `PB6 / TIM4_CH1`. Plan TIM4 with a 1 MHz counter base and 20 ms period, with MCU-owned center/direction/limit/calibration mapping. Exact servo pulse limits remain open.
- CAN bitrate is fixed at `1 Mbit/s`, preserving the current CubeMX bit timing. ROS/SocketCAN must use the same bitrate; wiring must be short with correct 120-ohm termination at both bus ends, and bench validation must record CAN error counters/bus-off behavior. The protocol's 500 kbit/s value remains a non-binding suggestion.
- Encoder PPR, gear ratio, wheel radius, and left/right polarity will be centralized in a vehicle calibration configuration rather than accepted from current hard-coded values. Firmware starts with latched `0x000A calibration_invalid` until bench calibration produces a valid configuration; velocity mode remains inhibited while invalid.
- Rear motor zero-output policy is tiered: normal STOP, safe stop, and command timeout use coast; E-stop uses electrical brake. Both paths first force PWM to zero, direction changes require configurable dead time, and the PB12-PB15 H-bridge truth table must be bench verified.
- A dedicated physical E-stop is assigned to `PE1 / EXTI1`. Use an input pull-up, falling-edge interrupt, active-low normally-closed fail-safe circuit, plus periodic level re-check and debounce. Physical and CAN E-stop inputs are ORed; either triggers immediate braking and latched `0x0001`. Production configuration must not permit bypass; a clearly marked bench-only build may temporarily bypass during bring-up. `PC13` was considered but rejected because it is not broken out on the user's board.
- Still confirm motor PWM frequency and hardware driver fault/enable inputs.

## TB6612FNG Driver Decision
- User identified the rear motor module as TB6612FNG-based.
- Toshiba's official data sheet specifies PWM switching up to 100 kHz, so a 20 kHz TIM3 PWM carrier is supported.
- TB6612 truth table matters: with a direction selected, `PWM=L` produces short brake, not coast. Explicit modes are required: drive (`IN1/IN2` select direction plus PWM), coast/stop (`IN1=IN2=L`, STBY high), short brake (`IN1=IN2=H`), and standby/high impedance (`STBY=L`).
- The current signed `Motor_SetPWM()` API cannot safely represent the selected tiered stop policy and should be replaced by an explicit motor mode API.
- TB6612FNG operating output current is 1.0 A per channel in the documented operating range; higher peak figures are pulse/absolute limits, not continuous capability. Motor stall current must be checked during bench qualification.
- Keep the original wiring: PB12-PB15 for the two H-bridge direction pairs and TIM3 CH1/CH2 for motor PWM. Do not allocate an STM32-controlled STBY signal. The design assumes the module straps STBY high; bench bring-up must verify that assumption.
- User confirmed on 2026-08-30 that the TB6612 forward/reverse GPIO truth table has already been tested successfully on hardware. Remaining motor-side bring-up risk is therefore mainly closed-loop polarity, PWM scaling, stop/brake behavior under load, and driver current/thermal margin.

## Pending Discovery
- User clarification of missing hardware and operating constraints.

## CAN Protocol Freeze (2026-08-24)
- `docs/CAN_PROTOCOL.md` is now the normative CAN V1 contract for both the Linux codec and STM32 firmware.
- Bitrate is frozen at 1 Mbit/s, matching CubeMX and the approved bottom-controller design; the previous upper-layer 500 kbit/s YAML suggestion must be changed rather than treated as a second valid default.
- Host upstream command freshness (currently 500 ms) and the MCU command watchdog (100 ms) are intentionally separate safety layers and now have distinct semantics.
- The document freezes the existing IDs and layouts and records implementation gaps rather than changing the wire contract to match incomplete firmware.
- USB DFU is explicitly outside CAN V1 and must remain a maintenance-plane path with no actuator ownership.

## USB DFU OTA Discovery (2026-08-24)
- The current board assigns STM32F407 PA11/PA12 to CAN1 RX/TX.
- ST AN2606 defines STM32F40xxx/41xxx ROM USB DFU on USB OTG FS PA11/PA12; STM32F407 is included in that device family.
- This is an electrical pin conflict, not merely a CubeMX alternate-function conflict. Jumping to system memory or deinitializing CAN does not disconnect the external CAN transceiver from PA11/PA12.
- A direct RK3588 USB-host to STM32 ROM-DFU design therefore requires a hardware change or a different transport/bootloader architecture.
- AN2606 also requires an external HSE multiple of 1 MHz in the supported range for DFU execution on this family; the existing 8 MHz HSE is suitable in principle.
- CAN2 is technically available on PB5/PB6 or PB12/PB13, but both mappings conflict with approved resources: PB6 is the planned TIM4_CH1 steering output and PB12/PB13 are TB6612 direction outputs.
- CAN1 can instead be remapped from PA11/PA12 to currently unused PB8/PB9. This preserves the CAN peripheral and protocol while releasing PA11/PA12 for USB OTG FS/ROM DFU, and is the preferred board revision if PB8/PB9 are physically accessible.
- Using CAN2 also retains an STM32 bxCAN hardware dependency on the CAN1 clock domain and requires correct shared filter-bank partitioning, so it adds firmware complexity without solving pin allocation better than CAN1 PB8/PB9.

## ELF2 Host Constraints (2026-08-24)
- ELF2 is the ElfBoard RK3588 product with a 40-pin Raspberry-Pi-compatible header and an additional 20-pin expansion header; its published specification lists USB OTG and two USB 2.0 host interfaces.
- ELF2 GPIO names and Linux line numbers must come from the board's pin-allocation table/device tree. Do not assume Raspberry Pi BCM numbering or invent a GPIO number from the RK3588 SoC name.
- Use one reserved ELF2 GPIO for STM32 `NRST` and one for `BOOT0`, preferably from the 20-pin/40-pin expansion headers and not from a console, camera, or boot-critical peripheral.
- The ELF2 USB host port can be the DFU transport, but the STM32 development board's USB VBUS and external 5 V supply must not be paralleled without power isolation. A data-only cable may defeat ROM DFU VBUS detection; validate VBUS sensing before choosing it.
- `dfu-util` and the OTA downloader belong on ELF2/Linux. STM32 CAN remains the vehicle control plane; entering DFU requires a prior safe-stop and disables normal CAN actuation until the new application boots and reports fresh health.

## OTA Trust and Watchdog Decisions (2026-08-24)
- User selected mandatory SHA-256 plus ECDSA P-256 verification, with the verification public key compiled into the immutable Bootloader.
- User selected watchdog-supervised trial execution. A newly installed slot is `TESTING`, not immediately `CONFIRMED`; watchdog reset or missing confirmation causes the Bootloader to fall back to the last confirmed slot.
- ST AN4701/RM0090 states that with STM32F4 RDP Level 1, main Flash cannot be read, programmed, or erased through debug, SRAM boot, or the system-memory ROM bootloader. Therefore ROM USB DFU cannot remain the normal writer if RDP1 is enabled.
- The secure architecture must use a Flash-resident custom USB DFU/IAP Bootloader for normal OTA. It can remain compatible with Linux `dfu-util`, writes only the inactive slot, and verifies the signed manifest/image before marking it pending.
- ROM DFU is limited to development/factory recovery while RDP0 is used. Production RDP1 recovery must go through the custom Bootloader; returning RDP1 to RDP0 mass-erases main Flash.
- Protect Bootloader sectors using Flash write protection. RDP Level 2 is excluded because it is irreversible and disables system-memory boot; production RDP1 is the recommended target after recovery procedures are proven.
- User approved the corrected production architecture: a custom Flash-resident USB DFU/IAP Bootloader compatible with `dfu-util`; ROM DFU is not the production A/B writer.
- User approved strict anti-rollback: normal release signatures authorize only increasing security versions; automatic rollback is limited to the previous `CONFIRMED` slot; deliberate downgrade requires a separate maintenance signing key.
- User approved the STM32F407VE 512 KiB partition: sectors 0-1 Bootloader (32 KiB), sector 2 metadata replica 1 (16 KiB), sector 3 metadata replica 2 (16 KiB), sectors 4-5 slot A (192 KiB), and sectors 6-7 slot B (256 KiB). Signed release images are capped at 192 KiB.
- Metadata uses redundant, generation-numbered, CRC-protected append records so a torn write leaves an older valid record. Bootloader size is a hard 32 KiB budget; security checks must not be removed to meet it.
- User approved separating watchdog liveness from image confirmation: IWDG supervises every execution, while a `TESTING` image becomes `CONFIRMED` only after a sustained application health window.
- User approved `2 s` IWDG timeout, `30 s` sustained health before confirmation, and at most `3` trial boots. A failed trial consumes an attempt; after three failures the slot becomes `REJECTED` and the last `CONFIRMED` slot boots.
- A watchdog reset from an already confirmed slot is recorded as a runtime fault but does not automatically select an older firmware version.
- User approved the dual Bootloader entry path: normal OTA first sends an ordinary CAN V1 STOP and verifies stationary feedback, then asserts a dedicated ELF2-controlled force-Bootloader GPIO and resets; invalid/no-confirmed-image recovery enters DFU automatically.
- OTA MUST NOT add a maintenance motion command to CAN V1. The CAN path remains vehicle control only; the force GPIO is the maintenance-plane entry signal.
- User approved a server-side JSON manifest plus a fixed binary MCU image descriptor. Security correction: the release signer must sign the exact fixed binary descriptor; ELF2 may verify and transport it but must not transform signed JSON into a different unsigned MCU header. JSON is a readable/index representation bound to the signed descriptor.
- STM32F407 application code is normally linked to an absolute Flash address. A single ordinary `.bin` linked for slot A cannot safely execute from slot B by changing only `VTOR`; absolute code/data references remain wrong. The design must either publish slot-specific A/B binaries, use a copy-to-fixed-execution scheme, or introduce position-independent relocation. Slot-specific builds are the recommended minimal-risk choice.
- Complete design report written to `docs/superpowers/specs/2026-08-24-rk3588-stm32-usb-dfu-ota-design.md`; it is intentionally a design artifact only and does not modify CubeMX, linker scripts, Bootloader code, or ELF2 software.
- Design self-review corrected a critical entry detail: the production force-Bootloader signal must target a normal STM32 GPIO sampled by the custom Bootloader, not STM32 `BOOT0`, because `BOOT0=1` bypasses the custom trust/rollback layer and enters ROM.
- Design self-review found that current CAN V1 cannot attest the running firmware identity. The OTA report now requires a backward-compatible, read-only CAN firmware-identity feedback extension before implementation; it carries no update or motion command.

## Architecture Direction
- User selected approach A: layered bare-metal controller with a fixed TIM6 scheduling tick, short ISRs, event-driven superloop work, 100 Hz rear-wheel control, 50 Hz CAN feedback, and hardware-generated 50 Hz steering PWM.
- Rejected alternatives for this scope: HAL_GetTick-only scheduling has avoidable loop jitter; FreeRTOS adds synchronization and bring-up complexity without a current need.
- User approved the visual module boundaries: CAN transport/protocol, centralized safety manager, steering and independent wheel controllers, board drivers, feedback snapshot builder, fixed scheduler, emergency brake bypass, and IWDG health gate.
- User approved the safety state machine: BOOT, CALIBRATION_REQUIRED, STOPPED, ACTIVE, SAFE_STOP, ESTOP, and FAULT, including two-stage recovery through STOPPED and explicit E-stop rearm conditions.
- User requested all remaining design review in text; no further diagrams or visual companion screens should be used.
- User approved the scheduling/data-consistency design: TIM6 tick, event-driven main loop, 10 ms wheel-control task, immutable 20 ms feedback snapshots, minimal ISRs, and rollover-safe timing.
- Rear-wheel PI runs in the main-loop 10 ms control task, with one independent instance per wheel. Steering has no MCU PID because R3X V1 has no steering feedback and the standard servo closes its own internal position loop.
- Design correction from user review: preserve and evolve existing `PID.c`, `encoder.c`, and `bsp_motor.c` rather than replacing or ignoring them. `encoder.c` and `bsp_motor.c` remain hardware-facing BSP modules; `PID.c` remains the reusable control-algorithm module, called by the 10 ms application control step.
- Reuse is not verbatim: fix the TIM3/TIM1 encoder mapping error, expose raw delta/cumulative counts and validity, add explicit motor DRIVE/COAST/BRAKE operations, and add fixed-dt/anti-windup/reset behavior to PID while retaining compatible naming where practical.
- User approved the fault/feedback design: evidence-based encoder detection, no fabricated TB6612 fault without a diagnostic input, latched calibration/watchdog/E-stop policy, non-latched command errors, deterministic fault priority, freshness-aware flags, and applied sequence updates only for complete committed groups.

## Minimum Runnable Closed Loop Slice (2026-08-30)
- The firmware now has a first closed-loop rear-wheel velocity path: accepted CAN velocity commands are retained as targets, then executed from `chassis_control` on a 10 ms control cadence using `Encoder_Sample()` and per-wheel `PID_UpdateDt()`.
- `encoder.c` now matches the frozen hardware assignment: motor 1 reads TIM1, motor 2 reads TIM2. It reports signed delta counts, signed 64-bit accumulated counts, integer velocity in mm/s, and a trusted/untrusted flag.
- TIM1/TIM2 are configured for quadrature `TIM_ENCODERMODE_TI12` with no prescaler; TIM1 uses a 16-bit full-scale period and TIM2 uses a 32-bit full-scale period.
- `PID.c` now supports explicit fixed/real `dt`, reset, output limits, and conditional-integration anti-windup while retaining the legacy `PID_Update()` ABI.
- This slice intentionally treats any untrusted encoder sample as a local coast-and-reset condition, not a latched `ENCODER_FAULT`; full `safety_manager` latching, calibration gating, motor-stall detection, feedback population, physical E-stop, and board tuning remain future slices.
- Verification baseline after the slice: `python -m unittest discover -s tests -v` reports 41/41 passing, and `cmake --build build\Debug --target ros` links successfully with FLASH 23832 B and RAM 2176 B.
- Feedback publication is now wired into each executed closed-loop sample. `chassis_control` converts trusted 64-bit encoder counts to wheel position in mrad, publishes measured velocity/position validity through `BSP_BXCAN_SetFeedback()`, and never marks an untrusted wheel's stale value valid. Host coverage now reports 43/43 passing; the Debug target remains buildable.
- This remains a minimum runnable slice: feedback is published on the 10 ms control sample, while the full 20 Hz feedback scheduling policy, safety-manager ownership of status/fault flags, calibration, physical E-stop, and board-level PI tuning are still separate work.
- The minimum closed-loop Debug image is now on the vehicle controller and was verified by STM32CubeProgrammer. No motion command has been sent from this session; physical bring-up must start with the driven wheels lifted and a ready physical E-stop.
- The opt-in `BenchCan` image now exercises the real closed-loop path through CAN internal loopback and actual motor/encoder hardware. It records signed encoder deltas for forward/reverse phases and treats missing or wrong-sign response as a failed phase. It has been built but not flashed because booting it starts automatic motion.
- BenchCan was flashed and run with the wheels lifted. The first run proved both wheels moved but exposed an inverted right encoder sign; after applying right-wheel polarity `-1`, the second run passed all phases. The board reported `passed=1/status=0`; forward and reverse encoder signs now agree with vehicle command signs. The MCU remains in the BenchCan terminal E-stop loop until a normal Debug image is explicitly flashed.
- The corrected normal Debug image has now been flashed back and verified. Manual CAN testing is the next step; the controller will coast until it receives a fresh complete command group.

## Safety Manager and Physical E-stop (2026-08-30)
- PE1 is implemented as an active-low, normally-closed GPIOE input with pull-up and falling-edge EXTI1. The ISR only records the event; the normal loop performs level processing and safety arbitration.
- `safety_manager` now owns the minimum E-stop/timeout/fault decision and recovery gate. It never calls motor APIs; `chassis_control` alone translates `BRAKE` and `COAST` actions into TB6612 operations.
- Physical E-stop and CAN E-stop both latch `BSP_BXCAN_FAULT_ESTOP_ACTIVE`. Release alone cannot re-enable motion; a fresh all-zero STOP with RESET is required before a later velocity command can enter DRIVE.
- Feedback status now exposes the physical E-stop and safe-stop state immediately when the safety action changes, using the latest encoder snapshot for the remaining fields.
- Host verification after this stage is 50/50 tests passing. The normal Debug firmware links at FLASH 25576 B and RAM 2224 B.
- The safety slice has since been extended to the frozen minimum: `CALIBRATION_REQUIRED`, calibration service/gating, encoder invalidity/direction latching, motor-stall evidence, control-overrun braking, watchdog-reset latching, fault priority, and a 50 ms physical E-stop release hold are implemented. Real-vehicle PID tuning remains open.
- PE1 is wired but its electrical behavior has not been independently recorded here. The required next hardware check is powered, wheels restrained: assert the physical switch, observe brake and CAN feedback, release it, verify no resume, send reset STOP, then perform only a low-speed manual CAN command.
- Because external bxCAN injection is currently unavailable, BenchCan now generates all CAN command groups internally through CAN loopback and automatically checks the command watchdog, CAN E-stop, RESET STOP gate, and post-reset DRIVE recovery. The only operator action left is physically pressing and releasing PE1.
- The BenchCan physical phase waits up to 20 seconds for PE1. Two runs reached the physical phase but timed out without a detected PE1 assertion (`phase=8/status=3`); this is an uncompleted physical test, not evidence that PE1 passed. The board was restored to normal Debug after each run.
- The operator confirmed PE1 stopped the motors when pressed during other automatic stages. This is direct operational confirmation of the physical E-stop brake path; it also explains why the BenchCan sequence cannot continue after an early press without a deliberate release-and-reset recovery step.

## Calibration Gating (2026-08-30)
- Normal Debug now starts with no active calibration and enters `CALIBRATION_REQUIRED` with `COAST`. A velocity command cannot authorize DRIVE and does not create `calibration_failed` (`0x000A`); missing calibration is a non-fault inhibit as required by CAN V1.
- `wheel_calibration` validates encoder polarity, minimum-start PWM, version, and the valid marker before atomically replacing the active RAM snapshot. The `0x122` transaction commits through the dual-slot CRC-protected Flash storage path.
- BenchCan uses an explicit `CALIBRATION_BENCH_DEFAULTS` compile definition so its internal-loopback motion test remains intentional and isolated from the normal vehicle image.

## Calibration Flash Persistence (2026-08-30)
- Added `BSP/wheel_calibration_storage.c/.h` with two 32-byte records in Flash sector 10 (`0x080C0000`) and sector 11 (`0x080E0000`). The record serializes calibration fields explicitly, carries a generation counter, CRC32, and a commit marker written last.
- Startup accepts only a record with matching magic/format/length, valid CRC, final commit marker, and a semantically valid `CalibrationData_t`; if the newest record is damaged or incomplete, the other valid slot is selected.
- `Wheel_Calibration_CommitPending()` now writes the inactive slot and changes the active RAM snapshot only after the Flash write succeeds. A failed write leaves the previous active calibration intact.
- The application linker Flash region is limited to 384 KiB so sectors 10 and 11 remain outside the executable image. Host coverage includes reboot-style reload, newest-slot corruption fallback, and missing commit-marker rejection.
- The `BenchCalibration` image was built and flashed on the lifted vehicle. Flash readback confirmed a valid committed record at `0x080C0000` with polarity `+1/-1`, minimum-start PWM `100/100`, CRC `0x1C378883`, and commit marker `0xC0A17ED1`; `0x080E0000` remained erased. Normal Debug is restored afterward.

## Calibration Service (2026-08-30)
- `0x122 CMD_CALIBRATION` uses the frozen V1 layout: version, independent `service_seq`, opcode, confirmation options, cookie `0xC35A`, and zero reserved byte. Transport-invalid frames are dropped; semantic-invalid frames produce `REJECTED_BAD_FORMAT` in `0x185 FB_CALIBRATION`.
- The service is non-blocking and runs from the 100 Hz chassis task. It requires fresh complete all-zero STOP groups and trusted stillness, then executes only one wheel at a time at bounded low PWM with a settle hold between forward and reverse.
- `0x185` is emitted as an event response to a request and periodically every 20 ms while active. It is independent of the five-frame immutable feedback snapshot.
- The state machine writes a pending `CalibrationData_t` and commits it atomically only after both wheels pass. Existing active calibration is preserved on abort, cancel, safety preemption, and failed validation.
- The dedicated `BenchCalibration` image was flashed to the lifted vehicle. RAM at `0x200001F0` reported `magic=0xCA2B0B01`, `passed=1`, `status=0`; the calibration fields reported `state=3 (SUCCEEDED)`, `exit_reason=1 (SUCCESS)`. The ordinary Debug image was restored afterward.

## Complete Safety Manager (2026-08-30)
- Expanded `safety_manager` from a direct state selector into the safety evidence owner. It now latches encoder-invalid/direction faults (`0x0002`), motor stall (`0x000B`), and control overrun (`0x000C`), and recognizes watchdog-reset recovery (`0x0008`).
- Fault selection follows `ESTOP > latched device fault > command timeout > transient protocol fault`; control overrun requests BRAKE, while encoder/stall/watchdog/calibration faults request COAST. Latched faults cannot be cleared by a velocity command.
- Physical E-stop release must remain observed for 50 ms before a fresh all-zero STOP + RESET can re-arbitrate. A command received during the release hold does not restart the hold timer.
- The 10 ms chassis path reports actual elapsed `dt`, trusted encoder samples, measured speed, target speed, and saturated PID PWM to `safety_manager`. A safety fault is checked again before any new motor drive write.
- Encoder invalidity requires five consecutive invalid samples or 100 ms accumulated invalid time, with 100 ms valid recovery required before encoder-fault reset. Direction reversal and stall checks use debounce/arm thresholds from the frozen design.
- Host coverage includes fault reset re-arbitration, 50 ms E-stop release, watchdog reset, control overrun BRAKE, encoder recovery, sustained opposite direction, and sustained high-PWM stall detection.

## Feedback and Diagnostics (2026-08-30)
- The base `0x180..0x184` wire layout remains unchanged. `0x186` is an optional read-only extension and shares the immutable `feedback_seq` snapshot, so legacy hosts can ignore it safely.
- `FB_DIAGNOSTICS.byte6` is command age in 10 ms units, truncated toward zero; `0xFF` means stale/unavailable. Diagnostic counters are internal saturating `u16` values because the Classic-CAN eight-byte extension has no room for both counters.
- The diagnostic CAN error flag is sticky until MCU restart. Command timeout remains the safety recovery mechanism; this change does not claim that a CAN bus-off is an independently validated hardware driver-disable event.
# PID autotune agent context (2026-08-31)

- The worktree is intentionally dirty before this task; existing firmware, docs, tests, and generated build artifacts must be preserved.
- Existing reusable control/transport pieces include `BSP/PID.c/.h`, `BSP/encoder.c/.h`, `BSP/bsp_motor.c/.h`, `BSP/bsp_bxcan.c/.h`, `BSP/can_motor_bench.c/.h`, and the safety manager/state-machine modules.
- Existing host tests are Python `unittest` files under `tests/`; no `tools/pid/autotune.py` or host-side PID experiment runner was found in the initial file inventory.
- CAN V1 currently carries steering, rear-wheel velocity, calibration, status/health/velocity/position/diagnostics frames. It has no PID-parameter command frame, so host-side PID tuning cannot yet change MCU PID gains through the frozen protocol.
- Existing CAN motor bench is firmware-side/internal-loopback oriented and exercises fixed motion/safety phases; it is not yet a reusable host-driven `set_pid -> run_test -> collect -> analyze -> score` loop.
- Safety constraints must remain MCU-owned: E-stop/safe-stop/fault handling, watchdog, output limits, and any current/voltage protection must abort a trial and restore the last known safe gains.
- The PID maintenance extension now includes `0x188 FB_CONTROL_OUTPUT`, reporting actual signed left/right `Motor_Drive` outputs for live logging; current and voltage remain unavailable because the board/protocol exposes neither.
