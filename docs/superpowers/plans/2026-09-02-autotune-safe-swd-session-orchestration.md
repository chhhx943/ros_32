# AutotuneSafe SWD Session Orchestration Implementation Plan

> For agentic workers: REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Build a deterministic ST-LINK/SWD-only boot/session orchestration path with MCU-confirmed handshake, seqlock/CRC-protected snapshots, wrap-aware generations, an actuator-free LocalSessionSmoke firmware, and a runner that proves or rejects each lifecycle.

**Architecture:** Add a pure session-contract module shared by Smoke and the later AutotuneSafe profile. Keep retained boot data and one-shot boot request in .noinit, runtime request/status separate, and immutable results committed with final magic last. Build LocalSessionSmoke from a source list excluding all actuator/control modules. Refactor local_swd_runner.py into the single build/flash/run/attach/poll/read/validate/restore transaction.

**Tech Stack:** C11, STM32F4 HAL/CMake/Ninja, Python unittest, STM32CubeProgrammer CLI, arm-none-eabi-gdb/MI, ST-LINK GDB server.

---

## Task 1: RED tests for corrected races and identities

**Files:**
- Create: tests/test_autotune_safe_session_contract.py
- Modify: tests/test_autotune_safe_local_swd.py

- [ ] Step 1: Add Python tests before protocol implementation.

~~~python
def test_latched_complete_survives_delayed_poll():
    status = make_status(state=SESSION_COMPLETE_LATCHED, sample_generation=32)
    assert decode_status_snapshot(status)["state"] == SESSION_COMPLETE_LATCHED

def test_latched_abort_survives_delayed_poll():
    status = make_status(state=SESSION_ABORT_LATCHED, abort_reason=7)
    assert decode_status_snapshot(status)["state"] == SESSION_ABORT_LATCHED

def test_start_accepts_first_sample_before_running_read():
    before = 0xFFFFFFFE
    status = make_status(state=SESSION_RUNNING,
                         start_sample_generation=before,
                         sample_generation=0)
    assert validate_running_handshake(status, before, 11, 22)

def test_seqlock_snapshot_rejects_torn_status():
    assert read_stable_snapshot([make_status_bytes(commit_seq=3)]) is None

def test_crc_retry_accepts_later_stable_snapshot():
    assert read_stable_snapshot([bad_crc_snapshot, good_snapshot]) == good_status

def test_result_magic_last_rejects_half_write():
    assert validate_result(make_result_bytes(magic=0), expected_identity) == \
        "RESULT_COMMIT_INCOMPLETE"

def test_generation_wrap_is_forward():
    assert generation_is_forward(0, 0xFFFFFFFF)
    assert generation_delta_u32(0, 0xFFFFFFFF) == 1

def test_por_generation_one_does_not_accept_old_joint_identity():
    assert validate_joint_identity(new_por_result, old_artifact_identity) is False

def test_each_joint_identity_component_is_required():
    for field in ("build_id", "boot_generation", "session_generation",
                  "session_id", "experiment_id"):
        assert reject_with_one_mismatch(field)
~~~

- [ ] Step 2: Add fake-process runner tests for build/flash/verify/run ordering, attach-generation change, transient CRC retry, terminal latch capture, and cleanup restore.

~~~python
def test_runner_waits_for_programmer_exit_before_attach():
    runner = FakeOrchestrator()
    runner.run_smoke_once()
    assert runner.events.index("programmer_exit") < \
        runner.events.index("gdb_attach")

def test_attach_generation_change_is_orchestration_failure():
    result = orchestrate_with_boot_generation_change()
    assert result.code == "ATTACH_RESET_DETECTED"
    assert result.category == "ORCHESTRATION_FAILURE"
~~~

- [ ] Step 3: Run the new tests.

Run: python -m unittest discover -s tests -p "test_autotune_safe_session_contract.py" -v

Expected: FAIL because the new layouts, decoders, validators, and orchestration methods do not exist.

- [ ] Step 4: Record existing SWD test baseline.

Run: python -m unittest discover -s tests -p "test_autotune_safe_local_swd.py" -v

Expected: Existing tests may pass; no RED failure may be hidden.

## Task 2: Implement shared layouts, CRC, seqlock, and generation helpers

**Files:**
- Create: BSP/autotune_safe_session.h
- Create: BSP/autotune_safe_session.c
- Modify: tools/pid/local_swd_runner.py
- Test: tests/test_autotune_safe_session_contract.py

- [ ] Step 1: Define fixed C layouts and static size/offset assertions.

Use 44-byte boot identity, 68-byte status, 68-byte result, and 40-byte sample layouts. Boot/status begin with uint32 commit_seq. Result fields are:

~~~text
magic FINAL_MAGIC at offset 0
version/size
session_id
experiment_id
boot_generation
session_generation
start_sample_generation
first_sample_generation
last_sample_generation
sample_count
ring_start_index
ring_count
final_state
abort_reason
result_flags
ring_crc32
result_crc32
~~~

The result size field is 68. Remove experiment_start_generation and experiment_finish_generation entirely.

- [ ] Step 2: Implement the shared CRC-32/IEEE routine.

~~~c
uint32_t AutotuneSession_Crc32(const void *data, uint32_t size);
~~~

Use initial 0xFFFFFFFF, reflected polynomial 0xEDB88320, and final xor 0xFFFFFFFF. CRC fields and commit_seq are excluded from boot/status CRC; result CRC excludes result_crc32 but uses FINAL_MAGIC at offset 0.

- [ ] Step 3: Implement wrap-aware helpers.

~~~c
uint32_t AutotuneSession_GenerationDelta(uint32_t newer,
                                         uint32_t older);
uint8_t AutotuneSession_GenerationIsForward(uint32_t newer,
                                            uint32_t older);
~~~

GenerationIsForward is true only when delta is nonzero and less than 0x80000000. No signed comparison is allowed for generation ordering.

- [ ] Step 4: Add Python decoders and validators with the same offsets.

Required functions:

~~~python
generation_delta_u32(newer, older)
generation_is_forward(newer, older)
decode_boot_identity(data)
decode_status_snapshot(data)
decode_result(data)
validate_running_handshake(status, generation_before_start,
                           session_id, experiment_id)
validate_complete_result(result, ring, expected_identity)
~~~

The result validator first requires FINAL_MAGIC, then verifies result CRC and ring CRC, then checks the joint build/boot/session/experiment identity and wrap-aware sample ordering.

- [ ] Step 5: Run focused codec tests.

Run: python -m unittest discover -s tests -p "test_autotune_safe_session_contract.py" -v

Expected: Pure CRC/layout/generation tests pass; MCU state tests remain RED until Task 3.

## Task 3: Implement retained boot request, boot identity, and latched state

**Files:**
- Modify: BSP/autotune_safe_session.h
- Modify: BSP/autotune_safe_session.c
- Test: tests/test_autotune_safe_session_contract.py
- Test: tests/test_autotune_safe_local_runtime.py

- [ ] Step 1: Define .noinit retained and boot request layouts.

~~~c
typedef struct {
    uint32_t magic;
    uint32_t boot_generation;
    uint32_t boot_count;
    uint32_t last_consumed_request_id;
    uint32_t crc32;
} AutotuneSessionRetained_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t request_id;
    uint32_t request_type;
    uint32_t session_id;
    uint32_t experiment_id;
    uint32_t payload[3];
    uint32_t boot_generation_hint;
    uint32_t reserved;
    uint32_t crc32;
} AutotuneSessionBootRequest_t;
~~~

Place only these two records in .noinit. Do not place runtime state, PWM, PID, encoder, ring, or result there.

- [ ] Step 2: Implement boot init.

Validate retained CRC; rebuild generation/count from 1 when POR or SRAM loss invalidates retained RAM; increment otherwise, skipping zero. Set EARLY, clear runtime structures, set INIT, and set READY only after the profile/build/mailbox checks pass. Set FAULT on initialization failure.

Copy a valid boot request to local storage, clear all request fields immediately, record last_consumed_request_id, and reject duplicate request IDs without session-generation changes.

- [ ] Step 3: Implement seqlock writers.

~~~c
begin = current_seq | 1U;
barrier();
write_payload();
write_crc();
barrier();
commit_seq = (begin + 1U) & ~1U;
barrier();
~~~

The CRC excludes commit_seq. Provide a Cortex-M4 barrier and a host-test fallback.

- [ ] Step 4: Implement latched state transitions.

CREATE_SESSION is valid only in READY and increments session_generation once. It enters ARMED and clears the old result only after the new request is accepted. START is valid only in ARMED with matching session. ACK_RESULT is valid only in COMPLETE_LATCHED or ABORT_LATCHED with matching current result; it records result_ack_request_id and returns READY. A legal new CREATE_SESSION may replace the latched result and enter ARMED.

- [ ] Step 5: Run C host and focused tests.

Run: python -m unittest discover -s tests -p "test_autotune_safe_local_runtime.py" -v

Expected: Existing local runtime behavior remains compatible.

Run: python -m unittest discover -s tests -p "test_autotune_safe_session_contract.py" -v

Expected: Retained lifecycle, seqlock, consume-once, and latched-state tests pass.

## Task 4: Implement START race handling, Smoke samples, and magic-last result

**Files:**
- Create: BSP/autotune_safe_local_smoke.c
- Modify: BSP/autotune_safe_session.c
- Modify: BSP/autotune_safe_session.h
- Test: tests/test_autotune_safe_session_contract.py
- Test: tests/test_autotune_safe_local_runtime.py

- [ ] Step 1: Implement START ordering.

Host records generation_before_start from a stable ARMED snapshot and places it in request payload. MCU validates the request, sets active_experiment_id, sets start_sample_generation to the current sample_generation, clears sample_count/ring index without clearing sample_generation, commits RUNNING, and only then permits the first sample increment.

- [ ] Step 2: Generate 32 fixed synthetic rows.

~~~text
sample_generation: one forward increment per row
timestamp_ms: monotonic tick
session_id/experiment_id: current identity
sample_index: 0..31
target: 100
actual: 100
pwm: 0
state: RUNNING
flags: SYNTHETIC
~~~

Smoke source must not include or call motor, PID, encoder, Servo, calibration, chassis, CAN, or PWM APIs.

- [ ] Step 3: Implement result commit-last.

Build a local result with FINAL_MAGIC and compute result CRC over that final representation. Then set runtime result.magic=0, copy all remaining fields and CRC, execute a barrier, and atomically write FINAL_MAGIC. Host rejects magic=0 as RESULT_COMMIT_INCOMPLETE and never computes a final CRC using magic=0.

- [ ] Step 4: Implement ABORT_LATCHED.

Stop sample generation, create an immutable result with final_state=ABORT_LATCHED and the current abort reason, commit magic last, and hold state/result until ACK_RESULT or legal CREATE_SESSION.

- [ ] Step 5: Run all race tests.

Run: python -m unittest discover -s tests -p "test_autotune_safe_session_contract.py" -v

Expected: PASS for delayed terminal polling, START-first-sample race, magic-last half write, CRC retry, and generation wrap.

## Task 5: Build compile-time isolated LocalSessionSmoke

**Files:**
- Modify: CMakeLists.txt
- Modify: CMakePresets.json
- Modify: cmake/stm32cubemx/CMakeLists.txt
- Create: Core/Src/main_local_session_smoke.c
- Create: tests/test_autotune_safe_session_build.py
- Modify: tests/test_autotune_safe_local.py

- [ ] Step 1: Add LOCAL_SESSION_SMOKE option and LocalSessionSmoke preset under build/LocalSessionSmoke.

- [ ] Step 2: Select main_local_session_smoke.c in the generated source list only when LOCAL_SESSION_SMOKE is ON; Debug and AutotuneSafe continue to use main.c.

- [ ] Step 3: For Smoke add only HAL/startup/clock, autotune_safe_session.c, and autotune_safe_local_smoke.c. Exclude autotune_safe_local.c, bsp_motor.c, chassis_control.c, PID.c, encoder.c, servo.c, wheel_calibration*.c, safety_manager.c, CAN bench sources, and H4 sources.

- [ ] Step 4: Make smoke main initialize HAL/clock and call only session init/process. It must not initialize or write motor, encoder, PID, Servo, CAN, calibration, or PWM.

- [ ] Step 5: Add source and symbol isolation tests.

~~~python
def test_smoke_source_list_excludes_actuators():
    block = extract_smoke_source_block(read_text("CMakeLists.txt"))
    for name in ("bsp_motor.c", "chassis_control.c", "PID.c",
                 "encoder.c", "servo.c", "wheel_calibration.c"):
        assert name not in block
~~~

- [ ] Step 6: Configure, build, and inspect symbols.

Run: cmake --preset LocalSessionSmoke
Run: cmake --build --preset LocalSessionSmoke --target ros
Run: arm-none-eabi-nm -C --defined-only build/LocalSessionSmoke/ros.elf

Expected: Build succeeds and no Motor_Drive, Motor_Brake, Motor_Coast, Servo_Set, Encoder_Sample, PID_Update, or Wheel_Calibration symbol exists.

## Task 6: Refactor runner into one lifecycle

**Files:**
- Modify: tools/pid/local_swd_runner.py
- Modify: tests/test_autotune_safe_local_swd.py
- Create: tests/test_autotune_safe_runner_orchestration.py

- [ ] Step 1: Add injectable CubeProgrammer, GDB server, and GDB/MI process wrappers. Record argv, stdout/stderr, return code, and exit time.

- [ ] Step 2: Implement build artifacts. Run focused/full regression, Debug build, and LocalSessionSmoke build before any flash. Record ELF, BIN, build_id, git commit, and SHA-256. Build failure short-circuits before flash.

- [ ] Step 3: Implement identify/program/verify/run. Wait for CubeProgrammer process exit and close its handle before starting GDB server. Do not modify option bytes or persistent target settings.

- [ ] Step 4: Implement no-reset attach proof. Start the configured GDB server only after CubeProgrammer exits. Record complete args and output. Reject reset, reset-halt, connect-under-reset, or download configurations. If no installed server can prove no-reset attach, return ATTACH_FAIL before session creation.

- [ ] Step 5: Read boot identity with seqlock before and after attach. Any boot_generation change is ATTACH_RESET_DETECTED. Poll until profile/build/mailbox/version/CRC and READY are valid; transient odd sequence/CRC mismatch retries until deadline.

- [ ] Step 6: Implement CREATE_SESSION and START. Write request fields with magic cleared, CRC, then magic last. Wait for stable ARMED. Record generation_before_start from ARMED snapshot. Send START and accept RUNNING even when sample_generation has already advanced; validate start_sample_generation against generation_before_start.

- [ ] Step 7: Poll stable status. Detect boot changes, identity replacement, state regression, stale session, and lack of sample progress. Capture COMPLETE_LATCHED or ABORT_LATCHED even if the first terminal poll occurs after the transition.

- [ ] Step 8: Read status/result/ring once in a target halt without reset, validate final magic, CRCs, full ring, count, joint identity, and wrap-aware generations. Retry torn status snapshots before deadline.

- [ ] Step 9: Cleanup every path. Send MCU STOP if still running, wait for non-running evidence if possible, close GDB/server, flash/verify/run ordinary Debug, and record restore status. A restore failure is RESTORE_DEBUG_FAIL.

- [ ] Step 10: Run runner tests.

Run: python -m unittest discover -s tests -p "test_autotune_safe_runner_orchestration.py" -v

Expected: PASS for ordering, retry, delayed terminal capture, race handling, failure taxonomy, and cleanup restore.

## Task 7: Update report generation

**Files:**
- Modify: tools/pid/local_swd_runner.py
- Create: AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md
- Test: tests/test_autotune_safe_runner_orchestration.py

- [ ] Step 1: Render the corrected state/generation/CRC rules in every report.

The report must explicitly state COMPLETE_LATCHED/ABORT_LATCHED, three generation scopes, wrap-aware ordering, seqlock retry, final-magic CRC, and ORCHESTRATION_FAILURE classification.

- [ ] Step 2: Use null for missing evidence. Never fill missing identity/status/result/ring fields with zero.

- [ ] Step 3: Add report test.

~~~python
def test_report_for_integrity_failure_never_claims_motor_failure():
    text = render_report(failure_code="DATA_INTEGRITY_FAIL")
    assert "ORCHESTRATION_FAILURE" in text
    assert "NO_MOTION" not in text
    assert "ENCODER_FAIL" not in text
    assert "PID_FAIL" not in text
~~~

## Task 8: Software verification before hardware

**Files:**
- Modify: findings.md
- Modify: progress.md

- [ ] Step 1: Run focused session, SWD, local-runtime, and runner tests.
- [ ] Step 2: Run python -m unittest discover -s tests -v and record exact count.
- [ ] Step 3: Build Debug and LocalSessionSmoke and record both exit codes and artifacts.
- [ ] Step 4: Inspect Smoke ELF sections and symbols. Confirm .noinit is NOLOAD/NOBITS in RAM and no forbidden actuator symbol exists.
- [ ] Step 5: If GDB server/no-reset capability cannot be proven, record ATTACH_FAIL and do not flash.

## Task 9: No-reset dry probe and ten real smoke sessions

**Files:**
- Modify: AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md
- Create: autotune_runs/<unique-session-run>/ artifacts

- [ ] Step 1: Probe tool paths, versions/help, target serial, GDB server arguments, and no-reset capability without flashing.
- [ ] Step 2: Run one LocalSessionSmoke lifecycle and inspect all evidence.
- [ ] Step 3: Run ten independent flash/verify/run/attach/session/read/ACK/Debug-restore transactions. Every transaction needs 32 samples, unique joint identity, valid CRC, stable generations, no stale mailbox, and no attach reset.
- [ ] Step 4: Stop immediately on the first failure. Record 10/10 only when every run passes.

## Task 10: Stress smoke and strict phase gate

**Files:**
- Modify: AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md
- Create: autotune_runs/<unique-stress-run>/ artifacts

- [ ] Step 1: Exercise delayed START, old request_id, duplicate CREATE, duplicate START, ACK_RESULT replay, read-after-latch, half-read then reattach, bad CRC, and invalid requests in READY/ARMED/RUNNING.
- [ ] Step 2: Verify stale requests cannot increment session generation, old experiments cannot enter RUNNING, latched results remain immutable, and a new artifact cannot consume an old ring.
- [ ] Step 3: Set actuator admission true only if 10/10 and all stress cases pass. Otherwise keep it false.
- [ ] Step 4: Do not execute 40/60/75/100‰, interpret CCR/CNT/speed, run PID, H4, or H5 in this plan. Those belong to a separately approved actuator-only stage.
