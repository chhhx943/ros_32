#include "autotune_safe_local.h"

#ifdef AUTOTUNE_SAFE_PROFILE

#include "PID.h"
#include "bsp_bxcan.h"
#include "bsp_motor.h"
#include "encoder.h"
#include "safety_manager.h"

#define LOCAL_PERIOD_MS 10U
#define LOCAL_HOLD_MS 250U
#define LOCAL_COOLING_MS 3000U
#define LOCAL_STILLNESS_MS 50U
#define LOCAL_STILLNESS_MMPS 5
#define LOCAL_START_SPEED_MMPS 5
#define LOCAL_PWM_SAFETY_MARGIN 135
#define LOCAL_OPEN_LOOP_HOLD_MS 120U
#define LOCAL_OPEN_LOOP_TARGET_MMPS 5
#define LOCAL_OPEN_LOOP_RESPONSE_MMPS 5
#define LOCAL_OPEN_LOOP_STEP_COUNT 4U
#define LOCAL_PHASE_P_ONLY AUTOTUNE_SAFE_LOCAL_PHASE_P_ONLY
#define LOCAL_PHASE_PI AUTOTUNE_SAFE_LOCAL_PHASE_PI
#define LOCAL_PHASE_D AUTOTUNE_SAFE_LOCAL_PHASE_D

typedef struct {
    AutotuneSafe_Gains_t gains;
    uint8_t phase;
} LocalCandidate_t;

static const LocalCandidate_t g_candidates[] = {
    {{0.17f, 0.0f, 0.0f}, LOCAL_PHASE_P_ONLY},
    {{0.19f, 0.0f, 0.0f}, LOCAL_PHASE_P_ONLY},
    {{0.21f, 0.0f, 0.0f}, LOCAL_PHASE_P_ONLY},
    {{0.19f, 0.45f, 0.0f}, LOCAL_PHASE_PI},
    {{0.19f, 0.60f, 0.0f}, LOCAL_PHASE_PI},
    {{0.19f, 0.60f, 0.02f}, LOCAL_PHASE_D}
};

volatile AutotuneSafe_LocalControl_t g_autotune_safe_local_control;

__attribute__((section(".noinit"), used))
volatile AutotuneSafe_LocalBootRequest_t g_autotune_safe_local_boot_request;

static PID_t g_local_pid;
static AutotuneSafe_Gains_t g_local_best[2];
static uint32_t g_last_process_ms;
static uint8_t g_time_valid;
static uint8_t g_still_tracking;
static uint32_t g_still_start_ms;
static uint32_t g_hold_until_ms;
static uint32_t g_cooling_until_ms;
static uint32_t g_session_id;
static uint32_t g_experiment_id;
static int32_t g_min_actual;
static int32_t g_max_actual;
static uint8_t g_started;
static uint8_t g_point_started;
static uint16_t g_local_heartbeat_sequence;
static uint8_t g_session_abort_latched;
static uint8_t g_open_loop_step;
static uint8_t g_open_loop_active;
static uint8_t g_open_loop_found;
static uint8_t g_open_loop_completed;
static uint32_t g_open_loop_until_ms;
static uint16_t g_open_loop_start_pwm;

static const uint16_t g_open_loop_pwm[] = {40U, 60U, 75U, 100U};

static int32_t LocalAbs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t LocalElapsed(uint32_t now_ms, uint32_t start_ms, uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - start_ms) >= duration_ms) ? 1U : 0U;
}

static uint8_t LocalStill(const EncoderSample_t *left,
                          const EncoderSample_t *right)
{
    return (left != 0) && (right != 0) &&
           (left->trusted != 0U) && (right->trusted != 0U) &&
           (LocalAbs32(left->velocity_mmps) <= LOCAL_STILLNESS_MMPS) &&
           (LocalAbs32(right->velocity_mmps) <= LOCAL_STILLNESS_MMPS);
}

static void LocalRecordStillness(uint32_t now_ms, uint8_t still)
{
    if (still == 0U) {
        g_still_tracking = 0U;
        return;
    }
    if (g_still_tracking == 0U) {
        g_still_tracking = 1U;
        g_still_start_ms = now_ms;
    }
}

static uint8_t LocalStillConfirmed(uint32_t now_ms)
{
    return (g_still_tracking != 0U) &&
           (LocalElapsed(now_ms, g_still_start_ms, LOCAL_STILLNESS_MS) != 0U);
}

static volatile AutotuneSafe_LocalSummary_t *LocalSummary(uint8_t wheel)
{
    uint8_t candidate = g_autotune_safe_local_control.candidate_id;

    if (candidate >= AUTOTUNE_SAFE_LOCAL_CANDIDATE_CAPACITY) {
        candidate = 0U;
    }
    return &g_autotune_safe_local_control.candidate_summaries[wheel - 1U][candidate];
}

static uint8_t LocalCandidateAllowed(uint8_t wheel, uint8_t candidate_id)
{
    const LocalCandidate_t *candidate;

    if ((wheel < 1U) || (wheel > 2U) ||
        (candidate_id >= (uint8_t)(sizeof(g_candidates) / sizeof(g_candidates[0])))) {
        return 0U;
    }
    candidate = &g_candidates[candidate_id];
    return (AutotuneSafe_ValidateLocalCandidate(&candidate->gains,
                                                &g_local_best[wheel - 1U],
                                                candidate->phase) ==
            AUTOTUNE_SAFE_GAIN_ACCEPTED) ? 1U : 0U;
}

static void LocalAbort(uint8_t reason, uint32_t now_ms);

static void LocalResetPid(void)
{
    g_local_pid.Target = 0.0f;
    g_local_pid.Actual = 0.0f;
    g_local_pid.OutMax = 1000.0f;
    g_local_pid.OutMin = 0.0f;
    PID_Reset(&g_local_pid);
}

static void LocalRestoreBest(void)
{
    AutotuneSafe_Gains_t recovery = g_local_best[
        (g_autotune_safe_local_control.active_wheel == 2U) ? 1U : 0U];

    LocalResetPid();
    g_local_pid.Kp = recovery.kp;
    g_local_pid.Ki = recovery.ki;
    g_local_pid.Kd = recovery.kd;
}

static void LocalSetCommand(uint8_t wheel, int16_t target, uint32_t now_ms)
{
    BSP_BXCAN_Command_t command = {0};

    command.command_seq = (uint16_t)(g_autotune_safe_local_control.request_sequence +
                                     g_autotune_safe_local_control.total_sample_count + 1U);
    command.mode_flags = (target == 0) ? BSP_BXCAN_MODE_STOP : BSP_BXCAN_MODE_VELOCITY;
    command.rear_left_velocity_mmps = (wheel == 1U) ? target : 0;
    command.rear_right_velocity_mmps = (wheel == 2U) ? target : 0;
    command.accepted_time_ms = now_ms;
    BSP_BXCAN_SetLocalVelocityCommand(command.rear_left_velocity_mmps,
                                      command.rear_right_velocity_mmps, now_ms);
    Safety_Manager_AcceptCommand(&command);
}

static void LocalWriteSample(uint32_t now_ms, uint8_t wheel,
                             const EncoderSample_t *sample, int16_t target,
                             int16_t pwm, const AutotuneSafe_Status_t *status)
{
    uint32_t index = g_autotune_safe_local_control.sample_write_index %
                     AUTOTUNE_SAFE_LOCAL_SAMPLE_CAPACITY;
    volatile AutotuneSafe_LocalSample_t *row =
        &g_autotune_safe_local_control.samples[index];
    int32_t actual = (sample != 0) ? sample->velocity_mmps : 0;
    int32_t error = (int32_t)target - actual;
    volatile AutotuneSafe_LocalSummary_t *summary =
        LocalSummary(g_autotune_safe_local_control.active_wheel);
    EncoderSpeedEvidence_t evidence = {0};

    if (sample != 0) {
        Encoder_GetSpeedEvidence(sample, LOCAL_PERIOD_MS, &evidence);
    }

    row->timestamp_ms = now_ms;
    row->wheel = wheel;
    row->candidate_id = (g_open_loop_active != 0U) ? 0xFFU :
                        g_autotune_safe_local_control.candidate_id;
    row->kp = g_local_pid.Kp;
    row->ki = g_local_pid.Ki;
    row->kd = g_local_pid.Kd;
    row->target_mmps = target;
    row->actual_mmps = (int16_t)actual;
    row->encoder_delta = (sample != 0) ? sample->delta_counts : 0;
    row->dt_ms = LOCAL_PERIOD_MS;
    row->mcu_speed_mmps = evidence.mcu_speed_mmps;
    row->recomputed_speed_mmps = evidence.recomputed_speed_mmps;
    row->counts_per_wheel_rev = evidence.counts_per_wheel_rev;
    row->circumference_mm_x1000 = evidence.circumference_mm_x1000;
    row->pwm = pwm;
    row->pid_output = (int16_t)g_local_pid.Out;
    row->state = g_autotune_safe_local_control.state;
    row->stall = (status != 0) ? status->stall : 0U;
    row->saturation = (status != 0) ? status->saturation : 0U;
    row->overspeed = (status != 0) ? status->overspeed : 0U;
    row->oscillation = (status != 0) ? status->oscillation : 0U;
    row->abort_reason = (status != 0) ? status->abort_reason : 0U;

    g_autotune_safe_local_control.sample_write_index++;
    g_autotune_safe_local_control.total_sample_count++;
    if (g_open_loop_active != 0U) {
        return;
    }
    if (summary->sample_count < 0xFFFFFFFFUL) {
        summary->sample_count++;
    }
    summary->error_abs_sum += LocalAbs32(error);
    summary->error_sq_sum += (int64_t)error * (int64_t)error;
    if (target != 0) {
        summary->steady_error_abs_sum += LocalAbs32(error);
        summary->steady_count++;
    }
    if (LocalAbs32(pwm) > summary->max_pwm) {
        summary->max_pwm = LocalAbs32(pwm);
    }
    if (LocalAbs32(actual) > summary->max_speed) {
        summary->max_speed = LocalAbs32(actual);
    }
    if (actual < g_min_actual) {
        g_min_actual = actual;
    }
    if (actual > g_max_actual) {
        g_max_actual = actual;
    }
    summary->peak_to_peak = g_max_actual - g_min_actual;
    if (actual >= LOCAL_START_SPEED_MMPS) {
        g_started = 1U;
    }
}

static void LocalFinishSummary(uint8_t passed, uint8_t abort_reason)
{
    volatile AutotuneSafe_LocalSummary_t *summary =
        LocalSummary(g_autotune_safe_local_control.active_wheel);

    summary->valid = 1U;
    summary->wheel = g_autotune_safe_local_control.active_wheel;
    summary->candidate_id = g_autotune_safe_local_control.candidate_id;
    summary->passed = passed;
    summary->kp = g_local_pid.Kp;
    summary->ki = g_local_pid.Ki;
    summary->kd = g_local_pid.Kd;
    summary->abort_reason = abort_reason;
    if (g_autotune_safe_local_control.active_wheel == 1U) {
        g_autotune_safe_local_control.left_summary = *summary;
    } else {
        g_autotune_safe_local_control.right_summary = *summary;
    }
}

static void LocalStartCandidate(uint32_t now_ms)
{
    const LocalCandidate_t *candidate =
        &g_candidates[g_autotune_safe_local_control.candidate_id];

    LocalResetPid();
    g_local_pid.Kp = candidate->gains.kp;
    g_local_pid.Ki = candidate->gains.ki;
    g_local_pid.Kd = candidate->gains.kd;
    g_min_actual = 0;
    g_max_actual = 0;
    g_started = 0U;
    g_point_started = 0U;
    g_autotune_safe_local_control.point_index = 0U;
    *LocalSummary(g_autotune_safe_local_control.active_wheel) =
        (AutotuneSafe_LocalSummary_t){0};
    g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_WAIT_STILL;
    g_autotune_safe_local_control.candidate_count =
        (uint8_t)(sizeof(g_candidates) / sizeof(g_candidates[0]));
    g_autotune_safe_local_control.candidate_id =
        g_autotune_safe_local_control.candidate_id;
    g_last_process_ms = now_ms;
}

static void LocalBeginPoint(uint32_t now_ms)
{
    static const int16_t points[AUTOTUNE_SAFE_LOCAL_POINT_COUNT] = {25, 50, 75, 100};
    int16_t target = points[g_autotune_safe_local_control.point_index];

    g_session_id = 0x4C000000UL |
                   (g_autotune_safe_local_control.request_sequence & 0x00FFFFFFUL);
    g_experiment_id = (g_autotune_safe_local_control.candidate_id << 8) |
                      g_autotune_safe_local_control.point_index;
    LocalSetCommand(g_autotune_safe_local_control.active_wheel, target, now_ms);
    ++g_local_heartbeat_sequence;
    if ((AutotuneSafe_BeginExperiment(g_session_id, g_experiment_id, target) == 0U) ||
        (AutotuneSafe_Heartbeat(g_session_id, g_experiment_id,
                                g_local_heartbeat_sequence,
                                now_ms) == 0U)) {
        LocalAbort(AUTOTUNE_SAFE_ABORT_SAFETY, now_ms);
        return;
    }
    g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_RAMP_UP;
    g_point_started = 1U;
}

static void LocalBeginOpenLoopStep(uint32_t now_ms)
{
    uint8_t wheel = g_autotune_safe_local_control.active_wheel;

    g_session_id = 0x4B000000UL |
                   (g_autotune_safe_local_control.request_sequence & 0x00FFFFFFUL);
    g_experiment_id = 0xF000U | g_open_loop_step;
    LocalSetCommand(wheel, LOCAL_OPEN_LOOP_TARGET_MMPS, now_ms);
    ++g_local_heartbeat_sequence;
    if ((AutotuneSafe_BeginExperiment(g_session_id, g_experiment_id,
                                      LOCAL_OPEN_LOOP_TARGET_MMPS) == 0U) ||
        (AutotuneSafe_Heartbeat(g_session_id, g_experiment_id,
                                g_local_heartbeat_sequence,
                                now_ms) == 0U)) {
        LocalAbort(AUTOTUNE_SAFE_ABORT_SAFETY, now_ms);
        return;
    }
    g_open_loop_active = 1U;
    g_open_loop_until_ms = now_ms + LOCAL_OPEN_LOOP_HOLD_MS;
}

static void LocalStopPoint(uint32_t now_ms)
{
    AutotuneSafe_RequestStop();
    LocalSetCommand(g_autotune_safe_local_control.active_wheel, 0, now_ms);
    LocalResetPid();
    Motor_CoastAll();
    g_still_tracking = 0U;
    g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_RAMP_DOWN;
}

static void LocalAbort(uint8_t reason, uint32_t now_ms)
{
    g_session_abort_latched = 1U;
    AutotuneSafe_RecordAbort((AutotuneSafe_AbortReason_t)reason);
    AutotuneSafe_RequestStop();
    LocalSetCommand(g_autotune_safe_local_control.active_wheel, 0, now_ms);
    LocalResetPid();
    Motor_CoastAll();
    g_still_tracking = 0U;
    LocalFinishSummary(0U, reason);
    LocalRestoreBest();
    g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_ABORT;
}

void AutotuneSafe_LocalExperiment_Init(void)
{
    uint8_t wheel;
    uint32_t boot_magic = g_autotune_safe_local_boot_request.magic;
    uint32_t boot_command = g_autotune_safe_local_boot_request.command;
    uint32_t boot_sequence = g_autotune_safe_local_boot_request.sequence;
    uint32_t boot_wheel = g_autotune_safe_local_boot_request.wheel;

    g_autotune_safe_local_control = (AutotuneSafe_LocalControl_t){0};
    g_autotune_safe_local_control.status_magic = AUTOTUNE_SAFE_LOCAL_RESULT_MAGIC;
    g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_IDLE;
    g_autotune_safe_local_boot_request.magic = 0U;
    g_autotune_safe_local_boot_request.command = 0U;
    g_autotune_safe_local_boot_request.sequence = 0U;
    g_autotune_safe_local_boot_request.wheel = 0U;
    if ((boot_magic == AUTOTUNE_SAFE_LOCAL_BOOT_MAGIC) &&
        ((boot_command == AUTOTUNE_SAFE_LOCAL_COMMAND_START) ||
         (boot_command == AUTOTUNE_SAFE_LOCAL_COMMAND_STOP) ||
         (boot_command == AUTOTUNE_SAFE_LOCAL_COMMAND_RECOVER)) &&
        (boot_wheel <= 2U)) {
        g_autotune_safe_local_control.request_magic =
            AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC;
        g_autotune_safe_local_control.request_command = boot_command;
        g_autotune_safe_local_control.request_sequence = boot_sequence;
        g_autotune_safe_local_control.request_wheel = (uint8_t)boot_wheel;
    }
    for (wheel = 0U; wheel < 2U; ++wheel) {
        g_local_best[wheel].kp = 0.20f;
        g_local_best[wheel].ki = 0.60f;
        g_local_best[wheel].kd = 0.0f;
    }
    LocalResetPid();
    g_last_process_ms = 0U;
    g_time_valid = 0U;
    g_still_tracking = 0U;
    g_local_heartbeat_sequence = 0U;
    g_session_abort_latched = 0U;
    g_open_loop_step = 0U;
    g_open_loop_active = 0U;
    g_open_loop_found = 0U;
    g_open_loop_completed = 0U;
    g_open_loop_until_ms = 0U;
    g_open_loop_start_pwm = 0U;
}

uint8_t AutotuneSafe_LocalExperiment_IsEngaged(void)
{
    return (g_autotune_safe_local_control.request_magic == AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC) ||
           ((g_autotune_safe_local_control.state != AUTOTUNE_SAFE_LOCAL_IDLE) &&
            (g_autotune_safe_local_control.state != AUTOTUNE_SAFE_LOCAL_COMPLETE));
}

void AutotuneSafe_LocalExperiment_Process(uint32_t now_ms)
{
    EncoderSample_t left;
    EncoderSample_t right;
    EncoderSample_t active_sample;
    AutotuneSafe_Status_t status;
    int16_t raw_pwm;
    uint8_t wheel;
    uint8_t preflight_ok;
    uint8_t drive_ok;
    int16_t target;
    int16_t pwm;
    uint16_t open_loop_pwm;
    float dt_s;
    static const int16_t points[AUTOTUNE_SAFE_LOCAL_POINT_COUNT] = {25, 50, 75, 100};

    if ((g_time_valid != 0U) &&
        ((uint32_t)(now_ms - g_last_process_ms) < LOCAL_PERIOD_MS)) {
        return;
    }
    g_last_process_ms = now_ms;
    g_time_valid = 1U;

    if (g_autotune_safe_local_control.request_command ==
        AUTOTUNE_SAFE_LOCAL_COMMAND_STOP) {
        g_autotune_safe_local_control.request_command = AUTOTUNE_SAFE_LOCAL_COMMAND_NONE;
        if ((g_autotune_safe_local_control.state != AUTOTUNE_SAFE_LOCAL_IDLE) &&
            (g_autotune_safe_local_control.state != AUTOTUNE_SAFE_LOCAL_COMPLETE) &&
            (g_autotune_safe_local_control.active_wheel >= 1U) &&
            (g_autotune_safe_local_control.active_wheel <= 2U)) {
            LocalAbort(AUTOTUNE_SAFE_ABORT_SAFETY, now_ms);
        } else {
            g_autotune_safe_local_control.request_magic = 0U;
            Motor_CoastAll();
            LocalResetPid();
        }
    }

    if (g_autotune_safe_local_control.request_command ==
        AUTOTUNE_SAFE_LOCAL_COMMAND_RECOVER) {
        BSP_BXCAN_Command_t reset_command = {0};

        g_autotune_safe_local_control.request_command =
            AUTOTUNE_SAFE_LOCAL_COMMAND_NONE;
        g_autotune_safe_local_control.request_magic = 0U;
        reset_command.command_seq =
            (uint16_t)g_autotune_safe_local_control.request_sequence;
        reset_command.mode_flags = BSP_BXCAN_MODE_STOP |
                                    BSP_BXCAN_FLAG_RESET_FAULT;
        reset_command.accepted_time_ms = now_ms;
        Safety_Manager_AcceptCommand(&reset_command);
        Motor_CoastAll();
        LocalResetPid();
        return;
    }

    left = Encoder_Sample(1U, LOCAL_PERIOD_MS);
    right = Encoder_Sample(2U, LOCAL_PERIOD_MS);
    preflight_ok = (Safety_Manager_GetState() == SAFETY_STATE_STANDBY) &&
                   (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE) &&
                   (Safety_Manager_IsEstopActive() == 0U);
    drive_ok = (Safety_Manager_GetAction() == SAFETY_ACTION_DRIVE) &&
               (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE) &&
               (Safety_Manager_IsEstopActive() == 0U);
    LocalRecordStillness(now_ms, LocalStill(&left, &right));

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_IDLE ||
        g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_COMPLETE) {
        if ((g_autotune_safe_local_control.request_magic != AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC) ||
            (g_autotune_safe_local_control.request_command != AUTOTUNE_SAFE_LOCAL_COMMAND_START)) {
            LocalSetCommand(0U, 0, now_ms);
            Motor_CoastAll();
            return;
        }
        wheel = g_autotune_safe_local_control.request_wheel;
        if ((wheel < 1U) || (wheel > 2U) || !preflight_ok ||
            !LocalStillConfirmed(now_ms) ||
            (AutotuneSafe_ConfirmPreflight(preflight_ok,
                                           (Safety_Manager_IsEstopActive() == 0U),
                                           LocalStillConfirmed(now_ms),
                                           1U) == 0U)) {
            Motor_CoastAll();
            return;
        }
        /* Keep START pending until all preflight/stillness checks pass. A
           first request can arrive before the 50 ms stillness window has
           completed; consuming it earlier would leave a half-consumed
           mailbox in IDLE and require a second host request. */
        g_autotune_safe_local_control.request_command =
            AUTOTUNE_SAFE_LOCAL_COMMAND_NONE;
        g_autotune_safe_local_control.active_wheel = wheel;
        g_autotune_safe_local_control.candidate_id = 0U;
        g_autotune_safe_local_control.open_loop_start_pwm = 0U;
        g_autotune_safe_local_control.open_loop_max_pwm = 0U;
        g_autotune_safe_local_control.open_loop_response = 0U;
        g_autotune_safe_local_control.open_loop_completed = 0U;
        g_open_loop_step = 0U;
        g_open_loop_active = 0U;
        g_open_loop_found = 0U;
        g_open_loop_completed = 0U;
        g_open_loop_start_pwm = 0U;
        g_autotune_safe_local_control.request_magic = 0U;
        g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_OPEN_LOOP_SCAN;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_OPEN_LOOP_SCAN) {
        wheel = g_autotune_safe_local_control.active_wheel;
        active_sample = (wheel == 1U) ? left : right;
        if (g_open_loop_active == 0U) {
            if (g_open_loop_step >= LOCAL_OPEN_LOOP_STEP_COUNT) {
                g_open_loop_completed = 1U;
                g_autotune_safe_local_control.open_loop_completed = 1U;
                if (g_open_loop_found == 0U) {
                    LocalAbort(AUTOTUNE_SAFE_ABORT_NO_MOTION, now_ms);
                }
                return;
            }
            LocalBeginOpenLoopStep(now_ms);
            return;
        }
        if (!drive_ok || active_sample.trusted == 0U) {
            LocalAbort((active_sample.trusted == 0U) ? AUTOTUNE_SAFE_ABORT_ENCODER :
                                                        AUTOTUNE_SAFE_ABORT_SAFETY,
                       now_ms);
            return;
        }
        if (AutotuneSafe_Heartbeat(g_session_id, g_experiment_id,
                                   ++g_local_heartbeat_sequence,
                                   now_ms) == 0U) {
            LocalAbort(AUTOTUNE_SAFE_ABORT_WATCHDOG, now_ms);
            return;
        }
        LocalSetCommand(wheel, LOCAL_OPEN_LOOP_TARGET_MMPS, now_ms);
        open_loop_pwm = g_open_loop_pwm[g_open_loop_step];
        AutotuneSafe_GetStatus(&status);
        if ((status.pwm_limit > 0U) && (open_loop_pwm > status.pwm_limit)) {
            open_loop_pwm = status.pwm_limit;
        }
        Safety_Manager_ReportDriveObservation(wheel, LOCAL_OPEN_LOOP_TARGET_MMPS,
                                              (int16_t)active_sample.velocity_mmps,
                                              (int16_t)open_loop_pwm, now_ms);
        AutotuneSafe_RecordSample(now_ms, LOCAL_OPEN_LOOP_TARGET_MMPS,
                                  (int16_t)active_sample.velocity_mmps,
                                  (int16_t)open_loop_pwm, active_sample.trusted,
                                  drive_ok, (Safety_Manager_IsEstopActive() == 0U));
        Motor_Drive(wheel, (int16_t)open_loop_pwm);
        Motor_Coast((wheel == 1U) ? 2U : 1U);
        AutotuneSafe_GetStatus(&status);
        if (open_loop_pwm > g_autotune_safe_local_control.open_loop_max_pwm) {
            g_autotune_safe_local_control.open_loop_max_pwm = open_loop_pwm;
        }
        LocalWriteSample(now_ms, wheel, &active_sample,
                         LOCAL_OPEN_LOOP_TARGET_MMPS, (int16_t)open_loop_pwm,
                         &status);
        if (status.abort_flag != 0U) {
            LocalAbort(status.abort_reason, now_ms);
        } else if (LocalAbs32(active_sample.velocity_mmps) >=
                   LOCAL_OPEN_LOOP_RESPONSE_MMPS) {
            g_open_loop_found = 1U;
            g_open_loop_active = 0U;
            g_open_loop_completed = 1U;
            g_open_loop_start_pwm = open_loop_pwm;
            g_autotune_safe_local_control.open_loop_start_pwm = open_loop_pwm;
            g_autotune_safe_local_control.open_loop_response = 1U;
            g_autotune_safe_local_control.open_loop_completed = 1U;
            LocalStopPoint(now_ms);
        } else if ((int32_t)(now_ms - g_open_loop_until_ms) >= 0) {
            g_open_loop_active = 0U;
            if ((uint8_t)(g_open_loop_step + 1U) >= LOCAL_OPEN_LOOP_STEP_COUNT) {
                g_open_loop_completed = 1U;
                g_autotune_safe_local_control.open_loop_completed = 1U;
            } else {
                g_open_loop_step++;
            }
            LocalStopPoint(now_ms);
        }
        return;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_LOAD_CANDIDATE) {
        if (LocalCandidateAllowed(g_autotune_safe_local_control.active_wheel,
                                  g_autotune_safe_local_control.candidate_id) == 0U) {
            LocalAbort(AUTOTUNE_SAFE_ABORT_SAFETY, now_ms);
            return;
        }
        LocalStartCandidate(now_ms);
        return;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_COOLING) {
        LocalSetCommand(g_autotune_safe_local_control.active_wheel, 0, now_ms);
        Motor_CoastAll();
        LocalResetPid();
        if ((int32_t)(now_ms - g_cooling_until_ms) >= 0) {
            if (g_session_abort_latched != 0U) {
                g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_COMPLETE;
                return;
            }
            if (g_autotune_safe_local_control.candidate_id + 1U >=
                g_autotune_safe_local_control.candidate_count) {
                g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_COMPLETE;
            } else {
                g_autotune_safe_local_control.candidate_id++;
                g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_LOAD_CANDIDATE;
            }
        }
        return;
    }

    if ((g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_ABORT) ||
        (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_PASS)) {
        Motor_CoastAll();
        LocalRestoreBest();
        AutotuneSafe_RecordStillness(now_ms, left.velocity_mmps,
                                     right.velocity_mmps);
        if (LocalStillConfirmed(now_ms)) {
            g_cooling_until_ms = now_ms + LOCAL_COOLING_MS;
            g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_COOLING;
        }
        return;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_RAMP_DOWN) {
        Motor_CoastAll();
        LocalResetPid();
        AutotuneSafe_RecordStillness(now_ms, left.velocity_mmps,
                                     right.velocity_mmps);
        if (LocalStillConfirmed(now_ms)) {
            if (g_autotune_safe_local_control.open_loop_completed != 0U) {
                if (g_open_loop_found != 0U) {
                    g_autotune_safe_local_control.state =
                        AUTOTUNE_SAFE_LOCAL_LOAD_CANDIDATE;
                } else {
                    LocalAbort(AUTOTUNE_SAFE_ABORT_NO_MOTION, now_ms);
                }
            } else {
                g_autotune_safe_local_control.state =
                    AUTOTUNE_SAFE_LOCAL_OPEN_LOOP_SCAN;
            }
        }
        return;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_WAIT_STILL) {
        Motor_CoastAll();
        LocalResetPid();
        AutotuneSafe_RecordStillness(now_ms, left.velocity_mmps, right.velocity_mmps);
        if (!LocalStillConfirmed(now_ms)) {
            return;
        }
        if (g_point_started == 0U) {
            LocalBeginPoint(now_ms);
        } else if (g_autotune_safe_local_control.point_index + 1U <
                   AUTOTUNE_SAFE_LOCAL_POINT_COUNT) {
            g_autotune_safe_local_control.point_index++;
            LocalBeginPoint(now_ms);
        } else {
            g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_EVALUATE;
        }
        return;
    }

    if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_EVALUATE) {
        volatile AutotuneSafe_LocalSummary_t *summary = LocalSummary(
            g_autotune_safe_local_control.active_wheel);
        uint8_t passed = (g_started != 0U) &&
                         (summary->max_pwm <= LOCAL_PWM_SAFETY_MARGIN);

        LocalFinishSummary(passed, 0U);
        if (passed) {
            g_local_best[g_autotune_safe_local_control.active_wheel - 1U] =
                g_candidates[g_autotune_safe_local_control.candidate_id].gains;
            g_autotune_safe_local_control.completed_candidate_count++;
        }
        if (passed) {
            AutotuneSafe_RecordLocalCandidatePass(
                &g_candidates[g_autotune_safe_local_control.candidate_id].gains);
        }
        g_cooling_until_ms = now_ms + LOCAL_COOLING_MS;
        g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_COOLING;
        return;
    }

    wheel = g_autotune_safe_local_control.active_wheel;
    active_sample = (wheel == 1U) ? left : right;
    target = AutotuneSafe_GetEffectiveTarget();
    dt_s = ((float)LOCAL_PERIOD_MS) / 1000.0f;
    if (!drive_ok || active_sample.trusted == 0U) {
        LocalAbort((active_sample.trusted == 0U) ? AUTOTUNE_SAFE_ABORT_ENCODER :
                                                    AUTOTUNE_SAFE_ABORT_SAFETY,
                   now_ms);
        return;
    }
    if (AutotuneSafe_Heartbeat(g_session_id, g_experiment_id,
                               ++g_local_heartbeat_sequence,
                               now_ms) == 0U) {
        LocalAbort(AUTOTUNE_SAFE_ABORT_WATCHDOG, now_ms);
        return;
    }
    LocalSetCommand(wheel, target, now_ms);
    g_local_pid.Target = (float)target;
    g_local_pid.Actual = (float)active_sample.velocity_mmps;
    PID_UpdateDt(&g_local_pid, dt_s);
    raw_pwm = (int16_t)g_local_pid.Out;
    pwm = raw_pwm;
    if (pwm < 0) {
        pwm = 0;
    }
    AutotuneSafe_GetStatus(&status);
    if ((status.pwm_limit > 0U) && (pwm > (int16_t)status.pwm_limit)) {
        pwm = (int16_t)status.pwm_limit;
    }
    /* The scan supplies only a measured low-speed breakaway bias. It never
       bypasses the AutotuneSafe PWM envelope or the final actuator gate. */
    if ((g_open_loop_start_pwm > 0U) && (target > 0) &&
        (LocalAbs32(active_sample.velocity_mmps) <= LOCAL_STILLNESS_MMPS) &&
        (pwm < (int16_t)g_open_loop_start_pwm)) {
        pwm = (int16_t)g_open_loop_start_pwm;
    }
    Safety_Manager_ReportDriveObservation(wheel, target,
                                          (int16_t)active_sample.velocity_mmps,
                                          pwm, now_ms);
    AutotuneSafe_RecordSample(now_ms, target,
                              (int16_t)active_sample.velocity_mmps, pwm,
                              active_sample.trusted, drive_ok,
                              (Safety_Manager_IsEstopActive() == 0U));
    Motor_Drive(wheel, pwm);
    Motor_Coast((wheel == 1U) ? 2U : 1U);
    AutotuneSafe_GetStatus(&status);
    AutotuneSafe_RecordWheelTelemetry(wheel,
                                      (int16_t)active_sample.velocity_mmps,
                                      (wheel == 1U) ? status.pwm_left : status.pwm_right,
                                      g_local_pid.Kp,
                                      g_local_pid.Ki,
                                      g_local_pid.Kd,
                                      (float)raw_pwm);
    AutotuneSafe_RecordWheelTelemetry((wheel == 1U) ? 2U : 1U,
                                      (wheel == 1U) ? right.velocity_mmps : left.velocity_mmps,
                                      0,
                                      0.0f, 0.0f, 0.0f, 0.0f);
    LocalWriteSample(now_ms, wheel, &active_sample, target,
                     (wheel == 1U) ? status.pwm_left : status.pwm_right,
                     &status);

    if (status.abort_flag != 0U) {
        LocalAbort(status.abort_reason, now_ms);
    } else if ((g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_RAMP_UP) &&
               (target >= points[g_autotune_safe_local_control.point_index])) {
        g_autotune_safe_local_control.state = AUTOTUNE_SAFE_LOCAL_HOLD;
        g_hold_until_ms = now_ms + LOCAL_HOLD_MS;
    } else if ((g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_HOLD) &&
               ((int32_t)(now_ms - g_hold_until_ms) >= 0)) {
        LocalStopPoint(now_ms);
    }

}

#else

volatile AutotuneSafe_LocalControl_t g_autotune_safe_local_control;

void AutotuneSafe_LocalExperiment_Init(void) {}
void AutotuneSafe_LocalExperiment_Process(uint32_t now_ms) { (void)now_ms; }
uint8_t AutotuneSafe_LocalExperiment_IsEngaged(void) { return 0U; }

#endif
