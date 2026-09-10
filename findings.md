# Findings

## Open-loop scan follow-up (2026-09-02)
- Prior H4/open-loop evidence showed repeatable response around `75‰` and robust response around `100‰`; recent PID-only candidates never exceeded `40‰`, so zero speed in those summaries was not proof that the motor cannot move.
- Added a conservative MCU-local breakaway scan at `40/60/75/100‰`, one positive direction, 120 ms per point, before any PID candidate. Scan data cannot enter candidate scoring; no response ends with `NO_MOTION` and no accepted PID.
- Software verification is green at `190/190`; Debug and AutotuneSafe images build successfully. The follow-up live-board request did not form a valid new session because ST-LINK reset/boot handoff did not reinitialize the application. Debug was restored; no envelope expansion or H5 was performed.

## 2026-09-01 Closeout audit baseline
- `BSP/bsp_bxcan.c` accepts every complete pair without checking freshness against the last committed sequence; duplicate groups refresh `accepted_time_ms`, `g_command_group_accepted`, and `applied_command_seq`.
- `Safety_Manager_AcceptCommand` falls through to `SAFETY_STATE_FAULT` whenever a fresh command has `BSP_BXCAN_FLAG_SAFE_STOP` while not already in `SAFE_STOP`; this violates the protocol's dedicated SAFE_STOP state.
- Resolved: `BSP/wheel_calibration_storage.h` now uses legal Sector6/7 addresses `0x08040000/0x08060000` on the 512 KiB F407, while the linker reserves the first 256 KiB for application code.
- `main.c` starts TIM1/TIM2/TIM3 with `HAL_TIM_Base_Start_IT`; TIM3 is PWM-only and its Base IRQ is meaningless. TIM6 is configured for 1 kHz but no `HAL_TIM_PeriodElapsedCallback` event counter/overrun path exists; control is still driven directly from `HAL_GetTick()` superloop.
- `Motor_Drive` always writes PWM=0 then direction then PWM without a measurable/configurable dead-time state machine; direction reversals can be instantaneous.
- `Servo_SetAngleMrad` clamps out-of-range angles and static-steering cache keeps the last velocity command; STOP handling must explicitly neutralize/clear steering target.
- `encoder.c` hard-codes PPR, quadrature factor, gear ratio, wheel radius and polarity. `CalibrationData_t` only persists polarity/min-start-PWM, so calibration cannot be consumed for conversion/scaling.
- `docs/CAN_PROTOCOL.md` physical layer still says CAN1 PA11/PA12 although approved hardware is CAN1 remap PB8/PB9; documentation and CubeMX GPIO must be synchronized.

## 2026-09-01 Closeout corrections
- The baseline defects above are now covered by RED-first host tests and minimal fixes: fresh command sequencing, dedicated SAFE_STOP arbitration/recovery, legal F407 sector-6/7 dual-slot storage, TIM3 PWM-only ownership, TIM6 1 kHz event scheduling with bounded overrun handling, TB6612 reversal dead-time, servo rejection/rate limiting, static-steering STOP neutralization, CAN bus-off SAFE_STOP, and PB8/PB9 documentation.
- Wheel calibration now ramps PWM from 50 to 250 permille and records the first trusted response as each wheel's minimum-start value; forward delta sign is persisted as encoder polarity and reverse motion is checked relative to it. PPR, quadrature factor, gear ratio, and radius have persisted/consumed calibration fields, but the service still writes nominal defaults for them; measured values require a real-wheel procedure.
- Verification after the corrections: 138 Python tests pass and the Debug target links with 40,720 bytes Flash / 2,648 bytes RAM. No H5 ground test was run; bench macros remain opt-in.

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
- CubeMX now calculates CAN1 at **500,000 bit/s** (`Prescaler=6`, `BS1=11TQ`, `BS2=2TQ`, `SJW=1TQ`) from the verified 42 MHz APB1/CAN kernel clock.
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
- CAN V1 bitrate is fixed at `500 kbit/s`, with CubeMX and ROS/SocketCAN required to use the same rate; wiring must be short with correct 120-ohm termination at both bus ends, and bench validation must record CAN error counters/bus-off behavior.
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
- Bitrate is frozen at 500 kbit/s, matching CubeMX and the approved bottom-controller design; every host path must use the same 500 kbit/s default.
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
- Added `BSP/wheel_calibration_storage.c/.h` with two 32-byte records in Flash sector 6 (`0x08040000`) and sector 7 (`0x08060000`). The record serializes calibration fields explicitly, carries a generation counter, CRC32, and a commit marker written last.
- Startup accepts only a record with matching magic/format/length, valid CRC, final commit marker, and a semantically valid `CalibrationData_t`; if the newest record is damaged or incomplete, the other valid slot is selected.
- `Wheel_Calibration_CommitPending()` now writes the inactive slot and changes the active RAM snapshot only after the Flash write succeeds. A failed write leaves the previous active calibration intact.
- The application linker Flash region is limited to 256 KiB so sectors 6 and 7 remain outside the executable image. Host coverage includes reboot-style reload, newest-slot corruption fallback, and missing commit-marker rejection.
- The `BenchCalibration` image was built and flashed on the lifted vehicle. Flash readback confirmed a valid committed Sector6/7 record with polarity `+1/-1`, minimum-start PWM `100/100`, CRC, and commit marker intact; the inactive slot remained erased. Normal Debug is restored afterward.

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

## H4 hardware characterization (2026-09-01)

- The operator confirmed a lifted, mechanically restrained bench with a reachable physical E-stop. The steering servo is PWM-only open-loop; there is no servo position feedback to include in a closed-loop claim.
- Boundary review is positive in code/host evidence for modular `command_seq`, inactive-slot Sector6/7 calibration writes with commit-last marker, event-gated IWDG feeding, and PE1 EXTI latching. Actual power-cut atomicity, watchdog-stall reset, and PE1-to-PWM latency remain hardware measurements.
- The opt-in H4 firmware completed all eight ordered phases (`open L`, `open R`, `open both`, closed 200/200, 180/220, 220/180, Ackermann 150/250, 250/150) with runner `passed=1/status=0`, zero TX failures, and zero safety faults.
- Initial H4 no-motion was traced to a software regression: deleting the TIM3 Base IRQ also deleted `HAL_TIM_Base_Init(&htim3)`, so the generated Base MSP clock hook never enabled TIM3. Base initialization was restored without reintroducing any TIM3 Base IRQ/start. The rerun captured CCR `2100` for open-loop PWM=500 and nonzero encoder deltas on both wheels.
- The rerun is active but not yet an H5 acceptance: speeds are only a few mm/s under the current bench condition and the single `180/220` sample is `4/5`; repeated steady-window proof of strict differential ordering is still required. PID/Ackermann mapping remains unchanged.
- Verification after the H4 instrumentation: Python regression `143/143 PASS`; normal Debug build `FLASH 41248 B`, `RAM 2712 B`; H4 build `FLASH 44776 B`, `RAM 3272 B`; normal Debug image restored to the board. H5 remains blocked pending repeated differential and safety hardware evidence.
# PID autotune agent context (2026-08-31)

- The worktree is intentionally dirty before this task; existing firmware, docs, tests, and generated build artifacts must be preserved.
- Existing reusable control/transport pieces include `BSP/PID.c/.h`, `BSP/encoder.c/.h`, `BSP/bsp_motor.c/.h`, `BSP/bsp_bxcan.c/.h`, `BSP/can_motor_bench.c/.h`, and the safety manager/state-machine modules.
- Existing host tests are Python `unittest` files under `tests/`; no `tools/pid/autotune.py` or host-side PID experiment runner was found in the initial file inventory.
- CAN V1 currently carries steering, rear-wheel velocity, calibration, status/health/velocity/position/diagnostics frames. It has no PID-parameter command frame, so host-side PID tuning cannot yet change MCU PID gains through the frozen protocol.
- Existing CAN motor bench is firmware-side/internal-loopback oriented and exercises fixed motion/safety phases; it is not yet a reusable host-driven `set_pid -> run_test -> collect -> analyze -> score` loop.
- Safety constraints must remain MCU-owned: E-stop/safe-stop/fault handling, watchdog, output limits, and any current/voltage protection must abort a trial and restore the last known safe gains.
- The PID maintenance extension now includes `0x188 FB_CONTROL_OUTPUT`, reporting actual signed left/right `Motor_Drive` outputs for live logging; current and voltage remain unavailable because the board/protocol exposes neither.

## H4 follow-up acceptance (2026-09-01)

- Two recorded steady-window runs do not yet prove the strict differential inequality: `180/220` was `4/4` then `4/5`; reverse `220/180` was `5/4` in both. The few-mm/s output is below a useful characterization resolution. Do not tune PID or Ackermann math from this data.
- Open-loop scan found bilateral response beginning near PWM 60, with PWM 75 repeatable enough to observe and PWM 100 robust on the lifted bench. These are observations only; production calibration remains unchanged.
- PPR/gear values are configuration (`500 PPR`, quadrature x4, `28.0:1`); effective tire circumference is still a physical measurement task. The 33.25 mm firmware radius/208.9 mm circumference conflicts with the 32.5 mm Ackermann design radius and must be reconciled.
- Hardware IWDG probe is accepted: intentional scheduler stall reset, magic `0x49574447`, and `RCC_CSR.IWDGRSTF=1`. Normal SAFE_STOP probe is accepted: magic `0x53414645`, state 5, timeout fault `0x0004`, no reset after 4 s.
- Sector6/7 power-cut atomicity and PE1-to-PWM/方向脚 delay remain open because no controlled power-cut fixture or oscilloscope/logic analyzer capture was available. Existing host tests and functional PE1 observation are not substitutes for those measurements.
-
# Autotune hard safety layer discovery (2026-09-01)

- The existing autotune stack spans `tools/pid/{protocol,transport,experiment,search,analysis,autotune}.py`; it already restores the previous PID after a transport/MCU safety fault, but its scenarios include 600/1200/1600 mm/s and reverse motion and its reports do not carry the requested hard-abort telemetry.
- MCU PID gains are accepted through a volatile `0x123/0x124` transaction. The current firmware only checks a broad `PID_TUNING_MAX_GAIN=16.0` format limit; it does not enforce absolute per-term bounds or a relative delta from a best-known-safe set.
- `chassis_control.c` runs each wheel PID every 10 ms and currently sets PID output limits to ±1000; the motor driver has a separate ±1000 clamp plus TB6612 reversal dead-time. A tuning-only PWM cap and PWM slew limiter therefore need to sit in the control path before `Motor_Drive`, with MCU ownership independent of host/AI.
- `safety_manager` already owns E-stop, command freshness, watchdog, encoder, direction, and motor-stall evidence, but autotune-specific overspeed, oscillation, saturation duration, staged unlock, stillness/preflight, and cumulative runtime/cooling policy are not present.
- No reliable current or temperature feedback is exposed. Software limits must be explicitly conservative and must not claim to provide current protection; stalled high-PWM windows must be short and latched as failed experiments.
- Worktree contains substantial pre-existing uncommitted firmware/docs changes. This task must be additive and avoid resetting or reformatting unrelated files.
-
# AutotuneSafe prompt reconciliation (2026-09-01)

- The attached implementation prompt confirms the required sequence: host simulation/tests -> MCU host tests -> static/full regression -> build `AutotuneSafe` -> manually flash -> lifted rear-wheel H4 tuning -> restore normal Debug.
- It freezes the authority boundary: MCU owns actuator safety and level unlock; host requests/observes; AI only proposes gains and search direction. Ordinary Debug/Release must not expose the autotune actuator entry by runtime flag alone.
- Required new MCU responsibilities are staged levels, absolute and relative PID validation, tuning PWM cap, 10 ms slew and target ramp, stall/saturation/overspeed/oscillation/reversal/thermal-proxy aborts, freshness/watchdog/E-stop integration, abort recovery, and telemetry.
- Required host responsibilities are safety-aware experiment sequencing, stop-and-zero confirmation, MCU-effective-limit-aware analysis, best-known-safe recovery, safety-first scoring, structured JSON/CSV/leaderboard history, and P-only -> PI -> optional D local refinement.
- Real hardware validation is authorized only through the separately built `AutotuneSafe` image under lifted-wheel/E-stop supervision; no H5 or Ackermann math changes are allowed.

# AutotuneSafe implementation progress (2026-09-01)

- Profile isolation tests are green: `AUTOTUNE_SAFE_PROFILE` is only enabled by the dedicated CMake preset, automatic bench macros are off, and `Motor_Drive()` calls the profile gate.
- MCU host core is green for L0/L1 preflight, 100 mm/s + 150‰ envelope, bootstrap seed/step validation, post-safe relative step validation, MCU-owned promotion request, and severe-abort promotion lockout.
- MCU host actuator/abort tests are green for final gate authorization, target ramp, 20‰/10 ms PWM slew, direct reverse rejection, local stall/saturation/overspeed/oscillation/Safety/encoder aborts, and ordered session heartbeat watchdog.
- Host protocol tests are green for control frames and a Classic-CAN multi-frame telemetry snapshot carrying identity, limits, wheel outputs, PID terms, safety flags/counters, and sequence consistency.
- Host runner tests are green for L1 ramp/cap, high-speed/reverse rejection, safety fields, and failed-candidate baseline protection. Integration with the live CAN transport and MCU CAN routing remains in progress.

## AutotuneSafe final verification (2026-09-01)

- The final gate remains in `Motor_Drive()` under `AUTOTUNE_SAFE_PROFILE`, so velocity, Ackermann, runtime PID, calibration, and static-steering drive requests cannot bypass tuning limits.
- Ordinary Debug/Release do not route CAN `0x125` to the AutotuneSafe handler and do not define the profile macro.
- L1 is the only compiled active envelope: target `<=100 mm/s`, PWM `<=150‰`; L2+ remains deliberately unconfigured rather than assuming 250/350/500‰ safety.
- Fresh verification completed with `170/170 PASS`; Debug FLASH `41248 B`, AutotuneSafe FLASH `48040 B`, both within the 256 KiB application region.
- Generated artifacts: `build/AutotuneSafe/ros.elf`, `ros_autotune_safe.bin`, `ros_autotune_safe.hex`. No physical motion or flash occurred in this software phase.

# Automated runner hardware gate (2026-09-01)

- STM32CubeProgrammer probe succeeded: ST-LINK SN `3E3703013212354D434B4E00`, STM32F405/407-class, 512 KiB Flash, target voltage 3.19 V.
- `python-can` is installed, but the selected Windows `socketcan/can0` transport failed with `WinError 10047`; no CAN telemetry or session freshness can be verified.
- PnP search found no enumerated CAN adapter, and the motor source exposes PB12..PB15 direction pins plus TIM3 PWM only; no independent TB6612 STBY/driver-enable interlock was found.
- Fail-closed result: software PASS, hardware preflight BLOCKED, no AutotuneSafe flash, no motor movement, and no Debug restore were attempted. The report is `autotune_runs/20260901T090312Z/AUTOTUNE_SAFE_AUTOMATED_RUN.md`.

# Self-loop PID result (2026-09-01)

- `python tools\\pid\\automated_runner.py --self-loop` completed successfully with `SELF_LOOP_PASS` and generated `autotune_runs/20260901T091636Z/`.
- Simulated best PID: `Kp=0.25, Ki=0.60, Kd=0.00`; score `674.734925997457`; simulated PWM/slew/ramp and STOP path passed.
- The result is host-model-only. It cannot validate real encoder feedback, MCU-local watchdog/abort execution, TB6612 heating, current, temperature, or physical E-stop behavior.

# MCU-local AutotuneSafe executor (2026-09-01)

- The local experiment request mailbox is present only in the dedicated profile; normal Debug has no local executor symbol. ST-LINK is therefore a possible no-CAN request path, while the MCU still owns all actuator decisions.
- The local candidate table is deliberately conservative and fixed in firmware. Candidate validation is per-wheel against the local best-safe baseline with absolute bounds and explicit bootstrap steps; the core only records a local candidate after all four L1 points pass.
- `Motor_Drive()` remains the final gate. Local telemetry now reports the post-gate PWM stored by the gate, while PID output remains separately recorded as the raw feedback output.
- Physical evidence is still absent. Software equality of MCU/offline encoder conversion does not prove wheel circumference, real dt, polarity, current, temperature, TB6612 heating, or actual motor response.
- First no-CAN hardware attempt was fail-closed: AutotuneSafe preflight reached L1/READY, but the GDB session lost communication after the left START request and the mailbox remained unconsumed. No valid motion/telemetry evidence was accepted, no right-wheel run was started, and ordinary Debug was restored.
- GDB/MI root cause was reproduced and fixed in `tools/pid/local_swd_runner.py`: pipe-mode GDB emits `(gdb)` without a newline, so line iteration timed out; character-level prompt parsing plus `mi-async`/`-exec-interrupt --all` now supports bounded same-session monitoring and STOP-on-timeout. A later hardware retry was blocked before START by ST-LINK `DEV_CONNECT_ERR`; no new motion evidence exists, and the board's last successful image is AutotuneSafe idle.

## AutotuneSafe local hardware retry findings (2026-09-02)

- A CubeProgrammer `-run` transaction resets the normal `.bss` mailbox. The profile now exposes a `.noinit` boot handoff, validates command/magic/wheel locally, copies it to the normal mailbox once, and clears the handoff before processing.
- The first retry exposed a real sequence bug: the initial point heartbeat and first sample reused one sequence and correctly triggered the MCU session watchdog. A monotonic local heartbeat sequence fixed this without weakening freshness checks.
- Cooling/idle now refreshes an explicit zero STOP, and abort latches the local session closed after cooling.
- Final L1 session `2026090211` was complete and fail-closed: 625 local samples, six valid summaries, no Safety fault, no candidate pass; all actual speeds were zero and max PWM remained 17/19/21/30/34/34‰. This is not a PID result.
- The board was restored to ordinary Debug. No right-wheel, level promotion, H4/H5, Ackermann, or PWM expansion was performed.
- A second identical left-wheel L1 retry (`2026090212`) reproduced the negative result: six candidates, 627 samples, actual speed zero, no Safety fault, and no accepted PID. Ordinary Debug was restored and verified afterward.

# AutotuneSafe deterministic boot/session validation discovery (2026-09-02)

- The current worktree already contains substantial user changes and generated run artifacts; these must be preserved. Sessions `2026090211` and `2026090212` are explicitly not accepted as motor/PID evidence.
- The current local SWD path has a profile-only `.noinit` boot request and GDB/MI prompt handling, but it does not yet expose a single proven lifecycle contract with boot identity (`boot_magic`, `boot_count`, profile/build identity, reason/state, mailbox version), per-boot/session/sample generations, and host proof that reattach did not reset the target.
- The current MCU local executor exposes ordinary mailbox/status data and a bounded ring, while the host runner still needs an explicit READY -> session ACK/ARMED -> START ACK/RUNNING -> COMPLETE capture contract and fail-closed classifications for handshake/protocol/generation failures.
- The requested next phase is a handoff/session-capture reliability slice. Real PWM and motor/encoder interpretation remain prohibited until the software smoke path is proven repeatable and its evidence is recorded.

## Deterministic boot/session design options (2026-09-02)

- Option A: minimally extend the existing local control struct and request flow. Add identity/handshake fields and synthetic mode in place, then keep the current external GDB-server monitor. This minimizes firmware movement but leaves more lifecycle ownership split between CubeProgrammer, an externally managed server, and the runner.
- Option B (recommended): define a versioned MCU-owned identity/status/session mailbox contract, keep the boot request in `.noinit`, add a compile-time `LOCAL_SESSION_SMOKE` path that cannot drive actuators, and make the runner own the complete flash/verify/run/attach/poll/capture/restore transaction. Reattach uses a documented no-reset GDB-server invocation and the runner rejects missing READY, mismatched ACKs, stale generations, incomplete samples, and CRC failures.
- Option C: use GDB scripts for all flash, reset, request, and reads without a firmware state-machine change. This cannot satisfy MCU-confirmed ARMED/RUNNING state or prove old mailbox data is not being reused, so it is not suitable for this phase.

## Approved detailed scope (2026-09-02)

- User approved Option B and confirmed the implementation target is ST-LINK/SWD only; CAN is not an execution or result-read dependency.
- The pasted acceptance specification freezes the sequence as build -> flash/verify -> run -> non-reset attach -> READY -> MCU-confirmed session/ARMED -> MCU-confirmed experiment/RUNNING -> synthetic samples -> COMPLETE -> full readback/CRC/generation validation -> artifact -> Debug restore.
- `LOCAL_SESSION_SMOKE` must be compile-time isolated and must not reach Motor_Drive, Motor_Brake, Motor_Coast, Servo output, calibration actuator, or real PID output. Real PWM scan remains gated behind 10/10 smoke.
- Any missing boot, attach, generation, session, CRC, or sample evidence is `ORCHESTRATION_FAILURE`; no motor/encoder/PID diagnosis is permitted from such a run.

## Corrected session-contract implementation (2026-09-02)

- Added fixed 44-byte boot identity, 68-byte seqlock status, 68-byte final-magic result, 40-byte sample, retained record, and request layouts in `BSP/autotune_safe_session.h`.
- MCU contract now uses `COMPLETE_LATCHED`/`ABORT_LATCHED`; result remains immutable until matching `ACK_RESULT` or a legal new `CREATE_SESSION`. `start_sample_generation` is captured before the first sample increment, and all ordering helpers are uint32 wrap-aware.
- Added CRC/seqlock retry validators, magic-last request/result publication, POR-safe retained boot generation rebuilding, and an actuator-free `LocalSessionSmoke` target.
- The new software runner has explicit CubeProgrammer process-exit ordering, ST-LINK GDB server `--attach` capability proof, reset-argument rejection, injectable lifecycle orchestration tests, and conservative report classification.
- Verification so far: session contract `10/10`, runtime C harness `2/2`, runner orchestration `4/4`, build isolation `4/4`, legacy SWD `8/8`, and Smoke configure/build/link/symbol inspection pass. The CMake compatibility fix is applied; fresh full regression later reached `210/210`.
- Dry probe found CubeProgrammer 2.20.0, GDB 14.2.90, and `ST-LINK_gdbserver.exe`; server help exposes `--attach`, and the generated attach argv contains no reset/halt/erase/download option. This is tool capability evidence only, not a target session result.

## Corrected session race implementation and hardware smoke evidence (2026-09-02)

- The six approved specification corrections are implemented: latched terminal states, three-generation-only result proof, START sample ordering, seqlock/CRC stable reads, final-magic-last result commit, and scoped wrap-aware generation identity.
- Root causes found in the first deterministic hardware attempts were orchestration/runtime issues: CubeProgrammer `-run` alone resumed an old target state; a dummy TCP readiness probe consumed the single-client GDB port; GDB/MI prompt parsing missed a newline-less `(gdb)`; LocalSessionSmoke inherited IRQ cleanup left global interrupts disabled so SysTick stopped after the first sample; and immediate Debug restore can hit a transient ST-LINK `DEV_USB_COMM_ERR` after server teardown.
- Fixed the lifecycle to use `-rst -run`, explicit GDB server `--attach` proof, no dummy socket probe, character-level MI prompt handling, smoke-specific IRQ/MSP isolation with `__enable_irq()`, 10 ms synthetic cadence, per-session artifact capture, and three bounded restore attempts. No normal-flow startup step uses a fixed sleep as MCU-ready evidence.
- One live transaction passed end-to-end with boot generation `23`, session generation `1`, 32 samples, sample generations `1..32`, COMPLETE_LATCHED, final result magic/CRC, ring CRC, ACK, and Debug restore. Evidence is in `autotune_runs/20260902T083629Z_session_smoke_001_a317d668/`.
- A subsequent live batch reached 10/10 successful sessions before artifact capture was added. The first artifact-enabled batch completed session 1 but session 2 failed Debug restore with three `DEV_USB_COMM_ERR` results; a fresh batch then failed at initial flash with the same USB error. These are orchestration failures and are not counted as a final acceptance run.
- Software stress smoke passes 50 repeated lifecycle cycles with duplicate/stale CREATE, duplicate START, delayed latched-result reads, START-after-terminal attempts, ACK replay, unique session/experiment IDs, and no result/generation contamination.
- Current acceptance remains blocked: ST-LINK direct-connect probe also returns `DEV_USB_COMM_ERR`, despite tool version/help and target serial enumeration succeeding. No 40/60/75/100‰ PWM, PID, H4/H5, or motor/encoder diagnosis is permitted.
- Requested rerun result: artifact-enabled hardware batch reached `9/10`; sessions 1-9 passed end-to-end and were saved, session 10 completed experiment/readback but failed Debug restore three times with `DEV_USB_COMM_ERR`. Hardware stress was not started because strict 10/10 was not met.

## CAN V1 bitrate migration (2026-09-03)

- Verified clock path: `SYSCLK=168 MHz`, `APB1=HCLK/4`, so `PCLK1/CAN kernel clock=42 MHz`. The final bxCAN timing is `Prescaler=6`, `SJW=1 TQ`, `BS1=11 TQ`, `BS2=2 TQ`; `42,000,000/(6*(1+11+2))=500,000 bit/s`, sample point approximately 85.7%.
- Updated `ros.ioc` and generated `Core/Src/can.c`; added `BSP_BXCAN_V1_BITRATE=500000U` and a Python `CAN_V1_BITRATE=500_000` source constant used by `PythonCanTransport` and `automated_runner.py`.
- Protocol semantics remain frozen: Classic CAN 2.0A, 11-bit standard IDs, 8-byte payloads, `0x120/0x121`, 10 ms pairing, 100 ms MCU watchdog, feedback IDs, command freshness, E-stop, Safety, PID tuning and AutotuneSafe logic are unchanged.
- Load estimate at 500 kbit/s uses the current 8 normal feedback frames plus 2 command frames every 20 ms: 10 frames/20 ms, approximately 65 kbit/s or 13%. AutotuneSafe adds 11 telemetry frames: 21 frames/20 ms, approximately 136.5 kbit/s or 27.3%, using 130 nominal wire bits per standard 8-byte frame before retries/other nodes.
- External CAN hardware validation remains pending: verify both endpoints at 500 kbit/s, command/feedback/watchdog/E-stop behavior, error-passive and bus-off counters, termination, CANH/CANL, common ground, transceiver levels, and long-duration error-free operation.

- Verification on 2026-09-03: CAN bitrate focused tests `5/5`, full Python regression `222/222 OK` in `45.067 s`, Debug build `FLASH 41248 B / RAM 2712 B`, and AutotuneSafe build `FLASH 54908 B / RAM 13200 B`. No external CAN adapter/vehicle bus was used, so electrical acceptance is not claimed.

## Runner restore lifecycle hardening and hardware stress boundary (2026-09-02)

- `local_swd_runner.py` now records a reconstructable lifecycle: programmer exit, GDB server stdout/stderr and return state, GDB stdout/stderr, target halt/read/detach state, ST-LINK reopen probe, Debug write/verify, and final cleanup status. Each restore attempt is numbered and bounded; restore never reruns the session.
- Generic restore failure is split into stage-specific codes, including `RESTORE_DEBUG_GDB_DEAD`, `RESTORE_DEBUG_TARGET_DISCONNECTED`, `RESTORE_DEBUG_HALT_FAIL`, `RESTORE_DEBUG_READ_FAIL`, `RESTORE_DEBUG_WRITE_FAIL`, `RESTORE_DEBUG_VERIFY_FAIL`, `RESTORE_DEBUG_DETACH_FAIL`, `RESTORE_DEBUG_SERVER_EXIT_FAIL`, and `RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT`.
- The hardware stress run stopped at its first failure as required: sessions 1-5 completed and were artifacted; session 6 completed the MCU experiment and result capture, detached cleanly, and then failed all three fresh CubeProgrammer ST-LINK reopen probes with `DEV_USB_COMM_ERR`. This localizes the current boundary to ST-LINK USB/target ownership release/reopen, not boot/session/sample capture.
- Session-6 evidence is `autotune_runs/20260902T095211Z_session_smoke_006_4dd7edd4/restore_debug.json`; it records GDB alive/attached before cleanup, target halted and readable, explicit detach, GDB server output returning to its wait state, and three failed reopen attempts. A direct post-failure CubeProgrammer connect probe reproduced `DEV_USB_COMM_ERR`.
- Focused runner orchestration coverage is now `11/11`, including timeout evidence and restore failure injection; the latest full regression is `217/217 OK` in `56.294 s`. No real PWM, PID, H4/H5, or motor/encoder diagnosis is admitted.

## 2026-09-03 SWD session rerun

- Focused session/SWD/runtime/orchestration/stress tests remain green: `10/10`, `8/8`, `2/2`, `11/11`, and `1/1` respectively.
- A standalone LocalSessionSmoke transaction passed end-to-end with 32 samples, final result magic/CRC, ring CRC, ACK, and Debug restore: `autotune_runs/20260903T071454Z_session_smoke_001_91f5dab8/`.
- The formal 10-session batch stopped at the first failure as required: sessions 1-3 passed; session 4 completed session/result capture with `sample_count=32`, then all three Debug-restore ST-LINK reopen probes returned `DEV_USB_COMM_ERR`. Evidence: `autotune_runs/20260903T071552Z_session_smoke_004_0721fea4/`.
- The failure is classified as `RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT` / `ORCHESTRATION_FAILURE`. No related process or TCP port remained afterward, and a subsequent read-only dry probe enumerated the ST-LINK. This is not evidence of a motor, encoder, or PID fault.
- Strict hardware acceptance remains blocked at `3/10`; no actuator admission, PWM scan, PID, H4/H5, or motor/encoder diagnosis is allowed.

## 2026-09-07 标定失败恢复修复

- 根因：`Safety_Manager_AcceptCommand()` 清除了 `g_latched_fault_code` 和 CAN 故障码，但没有清理 `Wheel_Calibration_Service` 的 `TX_FAILED_VALIDATION`；下一次 `Safety_Manager_Process()` 又按该服务状态重新锁存 `0x000A`。
- 修复：仅在合法全零 STOP + `RESET_FAULT` 已通过既有 reset-cause 检查、且当前锁存故障确为 `BSP_BXCAN_FAULT_CALIBRATION_INVALID` 时调用 `Wheel_Calibration_Service_ClearFailedValidation()`；该函数将服务置为 `TX_NONE`、清除阶段/活动请求，保留退出原因供诊断，不触碰急停、编码器和其他故障门控。
- 回归覆盖：标定失败 -> 0x000A -> STOP+RESET -> `CALIBRATION_REQUIRED` -> 后续 `START` 进入 `PRECHECK`/`CALIBRATION`。
- 验证：相关测试 `35 passed`；全量回归 `224 passed`；Debug 固件链接成功，Flash `41212 B`、RAM `2712 B`。本次未烧录 STM32。

## 2026-09-07 STM32F407 扩展板接口核对

- 当前固件的外部信号接口已整理到 `docs/STM32F407_EXPANSION_BOARD_PINOUT.md`。
- 已确认：CAN1 为 `PB8=CAN1_RX`、`PB9=CAN1_TX`；左/右电机 PWM 为 `PA6/PA7`；电机方向为 `PB12..PB15`；左编码器为 `PE9/PE11`；右编码器为 `PA0/PA1`；舵机为 `PB6`；物理急停为 `PE1`。
- `PA13/PA14` 保留 SWD；`PA10/PA11/PA12` 为 USB FS 预留但当前 USB 初始化为空；`PH0/PH1`、`PC14/PC15` 为晶振，不作为扩展 GPIO。
- 旧 OLED/MyI2C 源码存在 PB8/PB9、PA12/PB13 引脚冲突，当前不得直接做成 OLED/I2C 扩展接口。
