#include "autotune_safe.h"

#include <stdint.h>

#define AUTOTUNE_SAFE_L1_TARGET_LIMIT_MMPS 100U
#define AUTOTUNE_SAFE_L1_PWM_LIMIT         150U
#define AUTOTUNE_SAFE_TARGET_RAMP_STEP_MMPS 10
#define AUTOTUNE_SAFE_PWM_SLEW_PER_CYCLE    20
#define AUTOTUNE_SAFE_STALL_PWM_THRESHOLD   100
#define AUTOTUNE_SAFE_STALL_SPEED_THRESHOLD 5
#define AUTOTUNE_SAFE_STALL_TIMEOUT_MS      150U
#define AUTOTUNE_SAFE_SATURATION_PWM_THRESHOLD 140
#define AUTOTUNE_SAFE_SATURATION_ERROR_THRESHOLD 10
#define AUTOTUNE_SAFE_SATURATION_TIMEOUT_MS 250U
#define AUTOTUNE_SAFE_SESSION_TIMEOUT_MS    100U
#define AUTOTUNE_SAFE_OVERSPEED_SAMPLES     2U
#define AUTOTUNE_SAFE_OSCILLATION_LIMIT     3U
#define AUTOTUNE_SAFE_PROMOTION_PASS_COUNT 3U
#define AUTOTUNE_SAFE_STILLNESS_THRESHOLD_MMPS 5
#define AUTOTUNE_SAFE_STILLNESS_CONFIRM_MS 50U
#define AUTOTUNE_SAFE_MAX_SESSION_RUNTIME_MS 3000U
#define AUTOTUNE_SAFE_MAX_HIGH_PWM_RUNTIME_MS 1000U
#define AUTOTUNE_SAFE_COOLING_WINDOW_MS 3000U

#define AUTOTUNE_SAFE_SEED_KP 0.20f
#define AUTOTUNE_SAFE_SEED_KI 0.60f
#define AUTOTUNE_SAFE_SEED_KD 0.00f

#define AUTOTUNE_SAFE_BOOTSTRAP_KP_STEP 0.05f
#define AUTOTUNE_SAFE_BOOTSTRAP_KI_STEP 0.15f
#define AUTOTUNE_SAFE_BOOTSTRAP_KD_STEP 0.05f

#define AUTOTUNE_SAFE_KP_MIN 0.0f
#define AUTOTUNE_SAFE_KP_MAX 4.0f
#define AUTOTUNE_SAFE_KI_MIN 0.0f
#define AUTOTUNE_SAFE_KI_MAX 4.0f
#define AUTOTUNE_SAFE_KD_MIN 0.0f
#define AUTOTUNE_SAFE_KD_MAX 1.0f
#define AUTOTUNE_SAFE_RELATIVE_STEP 0.25f

static AutotuneSafe_Status_t g_status;
static AutotuneSafe_Gains_t g_candidate;
static uint8_t g_candidate_accepted;
static uint8_t g_experiment_evidence_valid;
static uint8_t g_complete_passes;
static uint8_t g_drive_authorized;
static int16_t g_requested_target_mmps;
static int16_t g_effective_target_mmps;
static int16_t g_last_pwm[2];
static uint32_t g_session_id;
static uint32_t g_experiment_id;
static uint16_t g_last_command_sequence;
static uint8_t g_heartbeat_seen;
static uint32_t g_last_valid_command_ms;
static uint8_t g_stall_tracking;
static uint32_t g_stall_start_ms;
static uint8_t g_saturation_tracking;
static uint32_t g_saturation_start_ms;
static uint8_t g_overspeed_samples;
static int8_t g_last_error_sign;
static uint8_t g_stillness_tracking;
static uint32_t g_stillness_start_ms;
static uint8_t g_process_time_valid;
static uint32_t g_last_process_ms;
static uint32_t g_session_runtime_ms;
static uint32_t g_high_pwm_runtime_ms;
static uint32_t g_cooling_until_ms;
static uint8_t g_safety_input_ok;
static uint8_t g_estop_clear;

static uint16_t AutotuneSafe_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t AutotuneSafe_ReadI16(const uint8_t *data)
{
    return (int16_t)AutotuneSafe_ReadU16(data);
}

static float AutotuneSafe_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t AutotuneSafe_Finite(float value)
{
    return (value == value) && (value < 100000.0f) && (value > -100000.0f);
}

static uint8_t AutotuneSafe_GainWithin(float value, float min_value, float max_value)
{
    return (AutotuneSafe_Finite(value) != 0U) &&
           (value >= min_value) && (value <= max_value);
}

static uint8_t AutotuneSafe_NextLevelConfigured(void)
{
    /* L2+ remain deliberately unconfigured until a human confirms the next
       compiled safety envelope from lifted-wheel evidence. */
    return 0U;
}

static uint8_t AutotuneSafe_SequenceNewer(uint16_t sequence,
                                          uint16_t previous)
{
    uint16_t distance = (uint16_t)(sequence - previous);
    return (distance != 0U) && (distance < 0x8000U);
}

static uint16_t AutotuneSafe_Elapsed(uint32_t now_ms, uint32_t start_ms)
{
    uint32_t elapsed = now_ms - start_ms;
    return (elapsed > 65535U) ? 65535U : (uint16_t)elapsed;
}

static uint8_t AutotuneSafe_WithinStep(float value,
                                       float baseline,
                                       float bootstrap_step)
{
    float limit = g_status.bootstrap_active ? bootstrap_step :
                  ((baseline > 0.000001f) ?
                   (baseline * AUTOTUNE_SAFE_RELATIVE_STEP) : bootstrap_step);
    return AutotuneSafe_Abs(value - baseline) <= (limit + 0.000001f);
}

static uint8_t AutotuneSafe_WithinExplicitStep(float value,
                                               float baseline,
                                               float bootstrap_step)
{
    float limit = (baseline > 0.000001f) ?
                  (baseline * AUTOTUNE_SAFE_RELATIVE_STEP) : bootstrap_step;
    return AutotuneSafe_Abs(value - baseline) <= (limit + 0.000001f);
}

static void AutotuneSafe_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void AutotuneSafe_WriteI16(uint8_t *data, int16_t value)
{
    AutotuneSafe_WriteU16(data, (uint16_t)value);
}

static int16_t AutotuneSafe_EncodeQ8_8(float value)
{
    int32_t raw;

    if (value > 127.996f) {
        value = 127.996f;
    } else if (value < -128.0f) {
        value = -128.0f;
    }
    raw = (int32_t)(value * 256.0f);
    if (raw > 32767) {
        raw = 32767;
    } else if (raw < -32768) {
        raw = -32768;
    }
    return (int16_t)raw;
}

void AutotuneSafe_Init(void)
{
    g_status = (AutotuneSafe_Status_t){0};
    g_status.state = AUTOTUNE_SAFE_STATE_LOCKED;
    g_status.level = AUTOTUNE_SAFE_LEVEL_L0_LOCKED;
    g_status.bootstrap_active = 1U;
    g_status.best_known_safe.kp = AUTOTUNE_SAFE_SEED_KP;
    g_status.best_known_safe.ki = AUTOTUNE_SAFE_SEED_KI;
    g_status.best_known_safe.kd = AUTOTUNE_SAFE_SEED_KD;
    g_candidate = g_status.best_known_safe;
    g_candidate_accepted = 0U;
    g_experiment_evidence_valid = 0U;
    g_complete_passes = 0U;
    g_drive_authorized = 0U;
    g_requested_target_mmps = 0;
    g_effective_target_mmps = 0;
    g_last_pwm[0] = 0;
    g_last_pwm[1] = 0;
    g_session_id = 0U;
    g_experiment_id = 0U;
    g_last_command_sequence = 0U;
    g_heartbeat_seen = 0U;
    g_last_valid_command_ms = 0U;
    g_stall_tracking = 0U;
    g_stall_start_ms = 0U;
    g_saturation_tracking = 0U;
    g_saturation_start_ms = 0U;
    g_overspeed_samples = 0U;
    g_last_error_sign = 0;
    g_stillness_tracking = 0U;
    g_stillness_start_ms = 0U;
    g_process_time_valid = 0U;
    g_last_process_ms = 0U;
    g_session_runtime_ms = 0U;
    g_high_pwm_runtime_ms = 0U;
    g_cooling_until_ms = 0U;
    g_safety_input_ok = 0U;
    g_estop_clear = 0U;
}

void AutotuneSafe_Process(uint32_t now_ms)
{
    uint32_t elapsed_ms = 0U;
    int16_t max_pwm;

    if (g_status.state == AUTOTUNE_SAFE_STATE_COOLING) {
        if ((int32_t)(now_ms - g_cooling_until_ms) >= 0) {
            g_status.state = AUTOTUNE_SAFE_STATE_READY;
        }
        return;
    }
    if (g_drive_authorized == 0U) {
        return;
    }
    if ((g_heartbeat_seen == 0U) ||
        ((uint32_t)(now_ms - g_last_valid_command_ms) >
         AUTOTUNE_SAFE_SESSION_TIMEOUT_MS)) {
        AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_WATCHDOG);
        return;
    }
    g_status.last_valid_command_age_ms =
        AutotuneSafe_Elapsed(now_ms, g_last_valid_command_ms);
    if (g_process_time_valid != 0U) {
        elapsed_ms = now_ms - g_last_process_ms;
        if (elapsed_ms > 100U) {
            elapsed_ms = 100U;
        }
        g_session_runtime_ms += elapsed_ms;
        max_pwm = (g_last_pwm[0] > g_last_pwm[1]) ? g_last_pwm[0] : g_last_pwm[1];
        if (max_pwm >= AUTOTUNE_SAFE_STALL_PWM_THRESHOLD) {
            g_high_pwm_runtime_ms += elapsed_ms;
        }
        if ((g_session_runtime_ms >= AUTOTUNE_SAFE_MAX_SESSION_RUNTIME_MS) ||
            (g_high_pwm_runtime_ms >= AUTOTUNE_SAFE_MAX_HIGH_PWM_RUNTIME_MS)) {
            AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_THERMAL);
            return;
        }
    }
    g_last_process_ms = now_ms;
    g_process_time_valid = 1U;
    if (g_effective_target_mmps < g_requested_target_mmps) {
        g_effective_target_mmps += AUTOTUNE_SAFE_TARGET_RAMP_STEP_MMPS;
        if (g_effective_target_mmps >= g_requested_target_mmps) {
            g_effective_target_mmps = g_requested_target_mmps;
            g_status.state = AUTOTUNE_SAFE_STATE_RUNNING;
        }
    }
    g_status.effective_target = g_effective_target_mmps;
}

uint8_t AutotuneSafe_ConfirmPreflight(uint8_t safety_ok,
                                      uint8_t estop_clear,
                                      uint8_t still,
                                      uint8_t firmware_ok)
{
    if ((safety_ok == 0U) || (estop_clear == 0U) ||
        (still == 0U) || (firmware_ok == 0U) ||
        (g_status.level != AUTOTUNE_SAFE_LEVEL_L0_LOCKED)) {
        return 0U;
    }
    g_status.level = AUTOTUNE_SAFE_LEVEL_L1_INITIAL;
    g_status.state = AUTOTUNE_SAFE_STATE_READY;
    g_status.target_limit_mmps = AUTOTUNE_SAFE_L1_TARGET_LIMIT_MMPS;
    g_status.pwm_limit = AUTOTUNE_SAFE_L1_PWM_LIMIT;
    g_safety_input_ok = 1U;
    g_estop_clear = 1U;
    return 1U;
}

void AutotuneSafe_GetStatus(AutotuneSafe_Status_t *status)
{
    if (status != 0) {
        *status = g_status;
    }
}

void AutotuneSafe_GetRecoveryGains(AutotuneSafe_Gains_t *gains)
{
    if (gains != 0) {
        *gains = g_status.best_known_safe;
    }
}

AutotuneSafe_GainResult_t AutotuneSafe_ValidateCandidate(
    const AutotuneSafe_Gains_t *candidate)
{
    if (candidate == 0) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_FORMAT;
    }
    if ((AutotuneSafe_GainWithin(candidate->kp,
                                 AUTOTUNE_SAFE_KP_MIN,
                                 AUTOTUNE_SAFE_KP_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->ki,
                                 AUTOTUNE_SAFE_KI_MIN,
                                 AUTOTUNE_SAFE_KI_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->kd,
                                 AUTOTUNE_SAFE_KD_MIN,
                                 AUTOTUNE_SAFE_KD_MAX) == 0U)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_ABSOLUTE;
    }
    if ((AutotuneSafe_WithinStep(candidate->kp,
                                 g_status.best_known_safe.kp,
                                 AUTOTUNE_SAFE_BOOTSTRAP_KP_STEP) == 0U) ||
        (AutotuneSafe_WithinStep(candidate->ki,
                                 g_status.best_known_safe.ki,
                                 AUTOTUNE_SAFE_BOOTSTRAP_KI_STEP) == 0U) ||
        (AutotuneSafe_WithinStep(candidate->kd,
                                 g_status.best_known_safe.kd,
                                 AUTOTUNE_SAFE_BOOTSTRAP_KD_STEP) == 0U)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    return AUTOTUNE_SAFE_GAIN_ACCEPTED;
}

AutotuneSafe_GainResult_t AutotuneSafe_ValidateLocalCandidate(
    const AutotuneSafe_Gains_t *candidate,
    const AutotuneSafe_Gains_t *baseline,
    uint8_t phase)
{
    if ((candidate == 0) || (baseline == 0) ||
        ((phase != AUTOTUNE_SAFE_LOCAL_PHASE_P_ONLY) &&
         (phase != AUTOTUNE_SAFE_LOCAL_PHASE_PI) &&
         (phase != AUTOTUNE_SAFE_LOCAL_PHASE_D))) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_FORMAT;
    }
    if ((AutotuneSafe_GainWithin(candidate->kp, AUTOTUNE_SAFE_KP_MIN,
                                 AUTOTUNE_SAFE_KP_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->ki, AUTOTUNE_SAFE_KI_MIN,
                                 AUTOTUNE_SAFE_KI_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->kd, AUTOTUNE_SAFE_KD_MIN,
                                 AUTOTUNE_SAFE_KD_MAX) == 0U)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_ABSOLUTE;
    }
    if ((phase == AUTOTUNE_SAFE_LOCAL_PHASE_P_ONLY) &&
        ((candidate->ki != 0.0f) || (candidate->kd != 0.0f))) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    if ((phase == AUTOTUNE_SAFE_LOCAL_PHASE_PI) && (candidate->kd != 0.0f)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    if (AutotuneSafe_WithinExplicitStep(candidate->kp, baseline->kp,
                                        AUTOTUNE_SAFE_BOOTSTRAP_KP_STEP) == 0U) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    if ((phase != AUTOTUNE_SAFE_LOCAL_PHASE_P_ONLY) &&
        (AutotuneSafe_WithinExplicitStep(candidate->ki, baseline->ki,
                                          AUTOTUNE_SAFE_BOOTSTRAP_KI_STEP) == 0U)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    if ((phase == AUTOTUNE_SAFE_LOCAL_PHASE_D) &&
        (AutotuneSafe_WithinExplicitStep(candidate->kd, baseline->kd,
                                          AUTOTUNE_SAFE_BOOTSTRAP_KD_STEP) == 0U)) {
        return AUTOTUNE_SAFE_GAIN_REJECTED_STEP;
    }
    return AUTOTUNE_SAFE_GAIN_ACCEPTED;
}

void AutotuneSafe_SetCandidate(const AutotuneSafe_Gains_t *candidate)
{
    if ((candidate != 0) &&
        (AutotuneSafe_ValidateCandidate(candidate) == AUTOTUNE_SAFE_GAIN_ACCEPTED)) {
        g_candidate = *candidate;
        g_candidate_accepted = 1U;
    } else {
        g_candidate_accepted = 0U;
    }
}

void AutotuneSafe_RecordCompletePass(void)
{
    if (g_candidate_accepted == 0U) {
        return;
    }
    g_status.best_known_safe = g_candidate;
    g_status.bootstrap_active = 0U;
    if (g_complete_passes < 255U) {
        ++g_complete_passes;
    }
    g_status.promotion_eligible =
        (g_complete_passes >= AUTOTUNE_SAFE_PROMOTION_PASS_COUNT) &&
        (AutotuneSafe_NextLevelConfigured() != 0U);
    g_candidate_accepted = 0U;
    g_experiment_evidence_valid = 0U;
}

void AutotuneSafe_RecordLocalCandidatePass(
    const AutotuneSafe_Gains_t *candidate)
{
    if ((candidate == 0) ||
        (AutotuneSafe_GainWithin(candidate->kp, AUTOTUNE_SAFE_KP_MIN,
                                 AUTOTUNE_SAFE_KP_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->ki, AUTOTUNE_SAFE_KI_MIN,
                                 AUTOTUNE_SAFE_KI_MAX) == 0U) ||
        (AutotuneSafe_GainWithin(candidate->kd, AUTOTUNE_SAFE_KD_MIN,
                                 AUTOTUNE_SAFE_KD_MAX) == 0U)) {
        return;
    }
    /* The local executor has already applied its phase and per-wheel step
       validator. This core-side API only records a complete four-point local
       pass, never an individual point or an aborted experiment. */
    g_candidate = *candidate;
    g_candidate_accepted = 1U;
    AutotuneSafe_RecordCompletePass();
}

uint8_t AutotuneSafe_RequestPromote(void)
{
    if ((g_status.promotion_eligible == 0U) ||
        (AutotuneSafe_NextLevelConfigured() == 0U)) {
        return 0U;
    }
    if (g_status.level < AUTOTUNE_SAFE_LEVEL_L4) {
        ++g_status.level;
    }
    g_complete_passes = 0U;
    g_status.promotion_eligible = 0U;
    return 1U;
}

void AutotuneSafe_RecordAbort(AutotuneSafe_AbortReason_t reason)
{
    g_status.state = (reason == AUTOTUNE_SAFE_ABORT_THERMAL) ?
                     AUTOTUNE_SAFE_STATE_COOLING : AUTOTUNE_SAFE_STATE_ABORT;
    g_status.abort_reason = (uint8_t)reason;
    g_status.promotion_eligible = 0U;
    g_complete_passes = 0U;
    g_candidate_accepted = 0U;
    g_experiment_evidence_valid = 0U;
    g_status.abort_flag = 1U;
    g_drive_authorized = 0U;
    g_requested_target_mmps = 0;
    g_effective_target_mmps = 0;
    g_last_pwm[0] = 0;
    g_last_pwm[1] = 0;
    g_status.pwm_left = 0;
    g_status.pwm_right = 0;
    if (reason == AUTOTUNE_SAFE_ABORT_THERMAL) {
        g_cooling_until_ms = g_last_process_ms + AUTOTUNE_SAFE_COOLING_WINDOW_MS;
    }
}

uint8_t AutotuneSafe_BeginExperiment(uint32_t session_id,
                                     uint32_t experiment_id,
                                     int16_t requested_target_mmps)
{
    if ((g_status.state != AUTOTUNE_SAFE_STATE_READY) ||
        (requested_target_mmps <= 0) ||
        ((uint16_t)requested_target_mmps > g_status.target_limit_mmps)) {
        return 0U;
    }
    g_requested_target_mmps = requested_target_mmps;
    g_effective_target_mmps = AUTOTUNE_SAFE_TARGET_RAMP_STEP_MMPS;
    if (g_effective_target_mmps > g_requested_target_mmps) {
        g_effective_target_mmps = g_requested_target_mmps;
    }
    g_last_pwm[0] = 0;
    g_last_pwm[1] = 0;
    g_status.pwm_left = 0;
    g_status.pwm_right = 0;
    g_session_id = session_id;
    g_experiment_id = experiment_id;
    g_last_command_sequence = 0U;
    g_heartbeat_seen = 0U;
    g_last_valid_command_ms = 0U;
    g_stall_tracking = 0U;
    g_saturation_tracking = 0U;
    g_overspeed_samples = 0U;
    g_last_error_sign = 0;
    g_stillness_tracking = 0U;
    g_stillness_start_ms = 0U;
    g_process_time_valid = 0U;
    g_last_process_ms = 0U;
    g_session_runtime_ms = 0U;
    g_high_pwm_runtime_ms = 0U;
    g_experiment_evidence_valid = 0U;
    g_status.abort_flag = 0U;
    g_status.abort_reason = AUTOTUNE_SAFE_ABORT_NONE;
    g_status.stall = 0U;
    g_status.saturation = 0U;
    g_status.overspeed = 0U;
    g_status.oscillation = 0U;
    g_status.speed_limit_hit = 0U;
    g_status.stall_time_ms = 0U;
    g_status.saturation_time_ms = 0U;
    g_status.oscillation_count = 0U;
    g_status.last_valid_command_age_ms = 0U;
    g_status.session_id = session_id;
    g_status.experiment_id = experiment_id;
    g_status.requested_target = requested_target_mmps;
    g_status.effective_target = g_effective_target_mmps;
    g_drive_authorized = 1U;
    g_status.state = AUTOTUNE_SAFE_STATE_RAMP;
    return 1U;
}

int16_t AutotuneSafe_GetEffectiveTarget(void)
{
    return g_effective_target_mmps;
}

void AutotuneSafe_RequestStop(void)
{
    if (g_drive_authorized == 0U) {
        return;
    }
    g_drive_authorized = 0U;
    g_requested_target_mmps = 0;
    g_effective_target_mmps = 0;
    g_last_pwm[0] = 0;
    g_last_pwm[1] = 0;
    g_status.requested_target = 0;
    g_status.effective_target = 0;
    g_status.pwm_left = 0;
    g_status.pwm_right = 0;
    g_status.state = AUTOTUNE_SAFE_STATE_STOPPING;
}

uint8_t AutotuneSafe_Heartbeat(uint32_t session_id,
                               uint32_t experiment_id,
                               uint16_t sequence,
                               uint32_t now_ms)
{
    if ((g_drive_authorized == 0U) || (session_id != g_session_id) ||
        (experiment_id != g_experiment_id) ||
        ((g_heartbeat_seen != 0U) &&
         (AutotuneSafe_SequenceNewer(sequence, g_last_command_sequence) == 0U))) {
        return 0U;
    }
    g_heartbeat_seen = 1U;
    g_last_command_sequence = sequence;
    g_last_valid_command_ms = now_ms;
    g_status.last_valid_command_age_ms = 0U;
    return 1U;
}

void AutotuneSafe_RecordSample(uint32_t now_ms,
                               int16_t target_mmps,
                               int16_t actual_mmps,
                               int16_t pwm,
                               uint8_t encoder_valid,
                               uint8_t safety_ok,
                               uint8_t estop_clear)
{
    int16_t error;
    int16_t target_abs;
    int16_t actual_abs;
    int8_t error_sign;
    uint8_t overspeed;

    if (g_status.abort_flag != 0U) {
        return;
    }
    if (estop_clear == 0U || safety_ok == 0U) {
        AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_SAFETY);
        return;
    }
    if (encoder_valid == 0U) {
        AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_ENCODER);
        return;
    }

    target_abs = (target_mmps < 0) ? (int16_t)-target_mmps : target_mmps;
    actual_abs = (actual_mmps < 0) ? (int16_t)-actual_mmps : actual_mmps;
    error = (int16_t)(target_mmps - actual_mmps);
    error_sign = (error > 0) ? 1 : (error < 0) ? -1 : 0;
    if ((error_sign != 0) && (g_last_error_sign != 0) &&
        (error_sign != g_last_error_sign)) {
        if (g_status.oscillation_count < 65535U) {
            ++g_status.oscillation_count;
        }
        if (g_status.oscillation_count >= AUTOTUNE_SAFE_OSCILLATION_LIMIT) {
            g_status.oscillation = 1U;
            AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_OSCILLATION);
            return;
        }
    }
    if (error_sign != 0) {
        g_last_error_sign = error_sign;
    }

    if ((pwm >= AUTOTUNE_SAFE_STALL_PWM_THRESHOLD) &&
        (actual_abs <= AUTOTUNE_SAFE_STALL_SPEED_THRESHOLD)) {
        if (g_stall_tracking == 0U) {
            g_stall_tracking = 1U;
            g_stall_start_ms = now_ms;
        }
        g_status.stall_time_ms = AutotuneSafe_Elapsed(now_ms, g_stall_start_ms);
        g_status.stall = 1U;
        if (g_status.stall_time_ms >= AUTOTUNE_SAFE_STALL_TIMEOUT_MS) {
            AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_STALL);
            return;
        }
    } else {
        g_stall_tracking = 0U;
        g_status.stall = 0U;
        g_status.stall_time_ms = 0U;
    }

    if ((pwm >= AUTOTUNE_SAFE_SATURATION_PWM_THRESHOLD) &&
        (AutotuneSafe_Abs((float)error) >=
         AUTOTUNE_SAFE_SATURATION_ERROR_THRESHOLD)) {
        if (g_saturation_tracking == 0U) {
            g_saturation_tracking = 1U;
            g_saturation_start_ms = now_ms;
        }
        g_status.saturation_time_ms =
            AutotuneSafe_Elapsed(now_ms, g_saturation_start_ms);
        g_status.saturation = 1U;
        if (g_status.saturation_time_ms >= AUTOTUNE_SAFE_SATURATION_TIMEOUT_MS) {
            AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_SATURATION);
            return;
        }
    } else {
        g_saturation_tracking = 0U;
        g_status.saturation = 0U;
        g_status.saturation_time_ms = 0U;
    }

    overspeed = ((actual_abs * 2) > (target_abs * 3)) ||
                (actual_abs > (target_abs + 20));
    if (overspeed != 0U) {
        if (g_overspeed_samples < 255U) {
            ++g_overspeed_samples;
        }
        g_status.overspeed = 1U;
        g_status.speed_limit_hit = 1U;
        if (g_overspeed_samples >= AUTOTUNE_SAFE_OVERSPEED_SAMPLES) {
            AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_OVERSPEED);
        }
    } else {
        g_overspeed_samples = 0U;
        g_status.overspeed = 0U;
    }
    if (g_status.abort_flag == 0U) {
        g_experiment_evidence_valid = 1U;
    }
}

void AutotuneSafe_RecordStillness(uint32_t now_ms,
                                  int16_t actual_left_mmps,
                                  int16_t actual_right_mmps)
{
    int16_t left_abs = (actual_left_mmps < 0) ?
                       (int16_t)-actual_left_mmps : actual_left_mmps;
    int16_t right_abs = (actual_right_mmps < 0) ?
                        (int16_t)-actual_right_mmps : actual_right_mmps;

    if ((g_status.state != AUTOTUNE_SAFE_STATE_STOPPING) &&
        (g_status.state != AUTOTUNE_SAFE_STATE_ABORT)) {
        g_stillness_tracking = 0U;
        return;
    }
    if ((left_abs > AUTOTUNE_SAFE_STILLNESS_THRESHOLD_MMPS) ||
        (right_abs > AUTOTUNE_SAFE_STILLNESS_THRESHOLD_MMPS)) {
        g_stillness_tracking = 0U;
        return;
    }
    if (g_stillness_tracking == 0U) {
        g_stillness_tracking = 1U;
        g_stillness_start_ms = now_ms;
        return;
    }
    if (AutotuneSafe_Elapsed(now_ms, g_stillness_start_ms) <
        AUTOTUNE_SAFE_STILLNESS_CONFIRM_MS) {
        return;
    }
    g_stillness_tracking = 0U;
    if (g_status.state == AUTOTUNE_SAFE_STATE_STOPPING) {
        /* A normal STOP that reaches local stillness is the only implicit
           completion point. Aborted sessions never enter this branch. */
        if (g_experiment_evidence_valid != 0U) {
            AutotuneSafe_RecordCompletePass();
        }
        g_status.state = AUTOTUNE_SAFE_STATE_READY;
    } else {
        /* An abort must pass through a fresh local preflight before it can
           authorize another experiment. */
        g_status.state = AUTOTUNE_SAFE_STATE_LOCKED;
        g_status.level = AUTOTUNE_SAFE_LEVEL_L0_LOCKED;
        g_status.target_limit_mmps = 0U;
        g_status.pwm_limit = 0U;
    }
}

void AutotuneSafe_RecordWheelTelemetry(uint8_t axis,
                                       int16_t actual_mmps,
                                       int16_t pwm,
                                       float pid_p,
                                       float pid_i,
                                       float pid_d,
                                       float pid_output)
{
    if (axis == 1U) {
        g_status.actual_left = actual_mmps;
        g_status.pwm_left = pwm;
        g_status.left_pid_p = pid_p;
        g_status.left_pid_i = pid_i;
        g_status.left_pid_d = pid_d;
        g_status.left_pid_output = pid_output;
    } else if (axis == 2U) {
        g_status.actual_right = actual_mmps;
        g_status.pwm_right = pwm;
        g_status.right_pid_p = pid_p;
        g_status.right_pid_i = pid_i;
        g_status.right_pid_d = pid_d;
        g_status.right_pid_output = pid_output;
    }
}

void AutotuneSafe_SetSafetyTelemetry(uint8_t safety_state,
                                     uint16_t fault_code)
{
    g_status.safety_state = safety_state;
    g_status.fault_code = fault_code;
}

void AutotuneSafe_SetSafetyInput(uint8_t safety_ok,
                                 uint8_t estop_clear)
{
    g_safety_input_ok = safety_ok != 0U ? 1U : 0U;
    g_estop_clear = estop_clear != 0U ? 1U : 0U;
    if ((g_safety_input_ok == 0U || g_estop_clear == 0U) &&
        (g_drive_authorized != 0U)) {
        AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_SAFETY);
    }
}

uint8_t AutotuneSafe_BuildTelemetryFrame(uint16_t std_id,
                                         uint8_t snapshot_seq,
                                         uint8_t axis,
                                         uint8_t data[8])
{
    int16_t q8_8_p;
    int16_t q8_8_i;
    int16_t q8_8_d;
    int16_t q8_8_output;
    int16_t actual;
    int16_t pwm;

    if (data == 0) {
        return 0U;
    }
    data[0] = 1U;
    data[1] = snapshot_seq;
    data[2] = 0U;
    data[3] = 0U;
    data[4] = 0U;
    data[5] = 0U;
    data[6] = 0U;
    data[7] = 0U;

    switch (std_id) {
    case AUTOTUNE_SAFE_ID_FB_IDENTITY:
        data[1] = AUTOTUNE_SAFE_PROFILE_ID;
        data[2] = snapshot_seq;
        AutotuneSafe_WriteU16(&data[3], (uint16_t)g_status.session_id);
        AutotuneSafe_WriteU16(&data[5], (uint16_t)g_status.experiment_id);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_STATE:
        data[2] = g_status.state;
        data[3] = g_status.level;
        data[4] = g_status.safety_state;
        data[5] = g_status.abort_reason;
        AutotuneSafe_WriteU16(&data[6], g_status.fault_code);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_LIMITS:
        AutotuneSafe_WriteI16(&data[2], g_status.requested_target);
        AutotuneSafe_WriteI16(&data[4], g_status.effective_target);
        AutotuneSafe_WriteU16(&data[6], g_status.pwm_limit);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_WHEEL:
        if (axis == 1U) {
            actual = g_status.actual_left;
            pwm = g_status.pwm_left;
        } else if (axis == 2U) {
            actual = g_status.actual_right;
            pwm = g_status.pwm_right;
        } else {
            return 0U;
        }
        data[2] = axis;
        AutotuneSafe_WriteI16(&data[3], actual);
        AutotuneSafe_WriteI16(&data[5], pwm);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_PID_PI:
    case AUTOTUNE_SAFE_ID_FB_PID_DO:
        if (axis == 1U) {
            q8_8_p = AutotuneSafe_EncodeQ8_8(g_status.left_pid_p);
            q8_8_i = AutotuneSafe_EncodeQ8_8(g_status.left_pid_i);
            q8_8_d = AutotuneSafe_EncodeQ8_8(g_status.left_pid_d);
            q8_8_output = AutotuneSafe_EncodeQ8_8(g_status.left_pid_output);
        } else if (axis == 2U) {
            q8_8_p = AutotuneSafe_EncodeQ8_8(g_status.right_pid_p);
            q8_8_i = AutotuneSafe_EncodeQ8_8(g_status.right_pid_i);
            q8_8_d = AutotuneSafe_EncodeQ8_8(g_status.right_pid_d);
            q8_8_output = AutotuneSafe_EncodeQ8_8(g_status.right_pid_output);
        } else {
            return 0U;
        }
        data[2] = axis;
        AutotuneSafe_WriteI16(&data[3],
                             (std_id == AUTOTUNE_SAFE_ID_FB_PID_PI) ? q8_8_p : q8_8_d);
        AutotuneSafe_WriteI16(&data[5],
                             (std_id == AUTOTUNE_SAFE_ID_FB_PID_PI) ? q8_8_i : q8_8_output);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_SAFETY:
        data[2] = (uint8_t)((g_status.stall != 0U) ? 1U : 0U) |
                  (uint8_t)((g_status.saturation != 0U) ? 2U : 0U) |
                  (uint8_t)((g_status.overspeed != 0U) ? 4U : 0U) |
                  (uint8_t)((g_status.oscillation != 0U) ? 8U : 0U) |
                  (uint8_t)((g_status.speed_limit_hit != 0U) ? 16U : 0U);
        data[3] = g_status.abort_reason;
        AutotuneSafe_WriteU16(&data[4], g_status.stall_time_ms);
        AutotuneSafe_WriteU16(&data[6], g_status.saturation_time_ms);
        return 1U;

    case AUTOTUNE_SAFE_ID_FB_COUNTERS:
        AutotuneSafe_WriteU16(&data[2], g_status.oscillation_count);
        AutotuneSafe_WriteU16(&data[4], g_status.last_valid_command_age_ms);
        return 1U;

    default:
        break;
    }

    return 0U;
}

void AutotuneSafe_OnCanFrame(uint8_t dlc,
                             uint8_t ide,
                             uint8_t rtr,
                             const uint8_t data[8],
                             uint32_t now_ms)
{
    uint8_t opcode;
    uint16_t session_id;
    uint16_t experiment_id;
    uint16_t argument;

    if ((data == 0) || (dlc != 8U) || (ide != 0U) || (rtr != 0U) ||
        (data[0] != 1U)) {
        return;
    }
    opcode = data[1];
    session_id = AutotuneSafe_ReadU16(&data[2]);
    experiment_id = AutotuneSafe_ReadU16(&data[4]);
    argument = AutotuneSafe_ReadU16(&data[6]);

    if (opcode == AUTOTUNE_SAFE_OPCODE_START) {
        (void)AutotuneSafe_BeginExperiment(session_id, experiment_id,
                                            AutotuneSafe_ReadI16(&data[6]));
    } else if ((session_id == (uint16_t)g_session_id) &&
               (experiment_id == (uint16_t)g_experiment_id)) {
        if (opcode == AUTOTUNE_SAFE_OPCODE_STOP) {
            AutotuneSafe_RequestStop();
        } else if (opcode == AUTOTUNE_SAFE_OPCODE_HEARTBEAT) {
            (void)AutotuneSafe_Heartbeat(session_id, experiment_id,
                                         argument, now_ms);
        } else if (opcode == AUTOTUNE_SAFE_OPCODE_REQUEST_PROMOTE) {
            (void)AutotuneSafe_RequestPromote();
        }
    }
}

#ifdef AUTOTUNE_SAFE_PROFILE
int16_t AutotuneSafe_ActuatorGate(uint8_t motor_num, int16_t requested_pwm)
{
    uint8_t index;
    int16_t limited_pwm;
    int16_t delta;

    if ((motor_num < 1U) || (motor_num > 2U) ||
        (g_drive_authorized == 0U) ||
        (g_heartbeat_seen == 0U) ||
        (g_safety_input_ok == 0U) ||
        (g_estop_clear == 0U) ||
        ((g_status.state != AUTOTUNE_SAFE_STATE_RAMP) &&
         (g_status.state != AUTOTUNE_SAFE_STATE_RUNNING)) ||
        (requested_pwm <= 0)) {
        return 0;
    }
    index = (uint8_t)(motor_num - 1U);
    limited_pwm = requested_pwm;
    if (limited_pwm > (int16_t)g_status.pwm_limit) {
        limited_pwm = (int16_t)g_status.pwm_limit;
    }
    delta = limited_pwm - g_last_pwm[index];
    if (delta > AUTOTUNE_SAFE_PWM_SLEW_PER_CYCLE) {
        limited_pwm = (int16_t)(g_last_pwm[index] + AUTOTUNE_SAFE_PWM_SLEW_PER_CYCLE);
    } else if (delta < -AUTOTUNE_SAFE_PWM_SLEW_PER_CYCLE) {
        limited_pwm = (int16_t)(g_last_pwm[index] - AUTOTUNE_SAFE_PWM_SLEW_PER_CYCLE);
    }
    g_last_pwm[index] = limited_pwm;
    if (motor_num == 1U) {
        g_status.pwm_left = limited_pwm;
    } else {
        g_status.pwm_right = limited_pwm;
    }
    return limited_pwm;
}
#endif
