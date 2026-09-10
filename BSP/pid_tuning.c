#include "pid_tuning.h"

#ifdef AUTOTUNE_SAFE_PROFILE
#include "autotune_safe.h"
#endif

typedef struct {
    uint8_t present;
    uint16_t sequence;
    uint8_t axis_mask;
    uint16_t kp;
    uint16_t ki;
    uint32_t received_time_ms;
} PID_Tuning_Pending_t;

static PID_Tuning_Gains_t g_left_gains;
static PID_Tuning_Gains_t g_right_gains;
static PID_Tuning_Pending_t g_pending;
static PID_Tuning_Feedback_t g_feedback;

static uint16_t PID_Tuning_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void PID_Tuning_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static float PID_Tuning_DecodeGain(uint16_t raw)
{
    return ((float)raw) / 256.0f;
}

static uint8_t PID_Tuning_HeaderValid(uint16_t std_id,
                                      uint8_t dlc,
                                      uint8_t ide,
                                      uint8_t rtr,
                                      const uint8_t data[8])
{
    if ((std_id != PID_TUNING_ID_CMD_GAINS) && (std_id != PID_TUNING_ID_CMD_D)) {
        return 0U;
    }
    if ((data == 0) || (dlc != 8U) || (ide != 0U) || (rtr != 0U) ||
        (data[0] != PID_TUNING_PROTOCOL_VERSION)) {
        return 0U;
    }
    if ((data[3] != PID_TUNING_AXIS_LEFT) &&
        (data[3] != PID_TUNING_AXIS_RIGHT) &&
        (data[3] != PID_TUNING_AXIS_BOTH)) {
        return 0U;
    }
    return 1U;
}

static uint8_t PID_Tuning_IsSafe(uint8_t command_fresh,
                                 uint8_t command_is_stop,
                                 uint8_t command_is_zero,
                                 uint8_t estop_active,
                                 uint8_t fault_active)
{
    return (command_fresh != 0U) && (command_is_stop != 0U) &&
           (command_is_zero != 0U) && (estop_active == 0U) &&
           (fault_active == 0U);
}

static uint8_t PID_Tuning_GainsValid(uint16_t kp, uint16_t ki, uint16_t kd)
{
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_Gains_t candidate;

    candidate.kp = PID_Tuning_DecodeGain(kp);
    candidate.ki = PID_Tuning_DecodeGain(ki);
    candidate.kd = PID_Tuning_DecodeGain(kd);
    return (AutotuneSafe_ValidateCandidate(&candidate) ==
            AUTOTUNE_SAFE_GAIN_ACCEPTED);
#else
    return (PID_Tuning_DecodeGain(kp) <= PID_TUNING_MAX_GAIN) &&
           (PID_Tuning_DecodeGain(ki) <= PID_TUNING_MAX_GAIN) &&
           (PID_Tuning_DecodeGain(kd) <= PID_TUNING_MAX_GAIN);
#endif
}

void PID_Tuning_Init(void)
{
    g_left_gains.kp = 0.20f;
    g_left_gains.ki = 0.60f;
    g_left_gains.kd = 0.0f;
    g_right_gains = g_left_gains;
    g_pending.present = 0U;
    g_feedback.transaction_seq = 0U;
    g_feedback.status = PID_TUNING_STATUS_NONE;
    g_feedback.axis_mask = 0U;
}

void PID_Tuning_OnFrame(uint16_t std_id,
                        uint8_t dlc,
                        uint8_t ide,
                        uint8_t rtr,
                        const uint8_t data[8],
                        uint32_t now_ms,
                        uint8_t command_fresh,
                        uint8_t command_is_stop,
                        uint8_t command_is_zero,
                        uint8_t estop_active,
                        uint8_t fault_active)
{
    uint16_t sequence;
    uint8_t axis_mask;
    PID_Tuning_Gains_t new_gains;

    if (PID_Tuning_HeaderValid(std_id, dlc, ide, rtr, data) == 0U) {
        g_feedback.status = PID_TUNING_STATUS_REJECTED_FORMAT;
        return;
    }
    sequence = PID_Tuning_ReadU16LE(&data[1]);
    axis_mask = data[3];
    g_feedback.transaction_seq = sequence;
    g_feedback.axis_mask = axis_mask;

    if (PID_Tuning_IsSafe(command_fresh, command_is_stop, command_is_zero,
                          estop_active, fault_active) == 0U) {
        g_pending.present = 0U;
        g_feedback.status = PID_TUNING_STATUS_REJECTED_UNSAFE;
        return;
    }

    if (std_id == PID_TUNING_ID_CMD_GAINS) {
        g_pending.present = 1U;
        g_pending.sequence = sequence;
        g_pending.axis_mask = axis_mask;
        g_pending.kp = PID_Tuning_ReadU16LE(&data[4]);
        g_pending.ki = PID_Tuning_ReadU16LE(&data[6]);
        g_pending.received_time_ms = now_ms;
        g_feedback.status = PID_TUNING_STATUS_NONE;
        return;
    }

    if ((g_pending.present == 0U) ||
        (g_pending.sequence != sequence) ||
        (g_pending.axis_mask != axis_mask)) {
        g_pending.present = 0U;
        g_feedback.status = PID_TUNING_STATUS_REJECTED_FORMAT;
        return;
    }
    if ((uint32_t)(now_ms - g_pending.received_time_ms) > 10U) {
        g_pending.present = 0U;
        g_feedback.status = PID_TUNING_STATUS_REJECTED_TIMEOUT;
        return;
    }
    if (PID_Tuning_ReadU16LE(&data[6]) != PID_TUNING_COMMIT_COOKIE) {
        g_pending.present = 0U;
        g_feedback.status = PID_TUNING_STATUS_REJECTED_FORMAT;
        return;
    }
    if (PID_Tuning_GainsValid(g_pending.kp, g_pending.ki,
                              PID_Tuning_ReadU16LE(&data[4])) == 0U) {
        g_pending.present = 0U;
        g_feedback.status = PID_TUNING_STATUS_REJECTED_FORMAT;
        return;
    }

    new_gains.kp = PID_Tuning_DecodeGain(g_pending.kp);
    new_gains.ki = PID_Tuning_DecodeGain(g_pending.ki);
    new_gains.kd = PID_Tuning_DecodeGain(PID_Tuning_ReadU16LE(&data[4]));
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_SetCandidate((const AutotuneSafe_Gains_t *)&new_gains);
#endif
    if ((axis_mask & PID_TUNING_AXIS_LEFT) != 0U) {
        g_left_gains = new_gains;
    }
    if ((axis_mask & PID_TUNING_AXIS_RIGHT) != 0U) {
        g_right_gains = new_gains;
    }
    g_pending.present = 0U;
    g_feedback.status = PID_TUNING_STATUS_ACCEPTED;
}

void PID_Tuning_GetGains(PID_Tuning_Gains_t *left, PID_Tuning_Gains_t *right)
{
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_Status_t status;
    AutotuneSafe_Gains_t recovery;

    AutotuneSafe_GetStatus(&status);
    if (status.abort_flag != 0U) {
        AutotuneSafe_GetRecoveryGains(&recovery);
        if (left != 0) {
            left->kp = recovery.kp;
            left->ki = recovery.ki;
            left->kd = recovery.kd;
        }
        if (right != 0) {
            right->kp = recovery.kp;
            right->ki = recovery.ki;
            right->kd = recovery.kd;
        }
        return;
    }
#endif
    if (left != 0) {
        *left = g_left_gains;
    }
    if (right != 0) {
        *right = g_right_gains;
    }
}

uint8_t PID_Tuning_GetFeedback(PID_Tuning_Feedback_t *feedback)
{
    if (feedback == 0) {
        return 0U;
    }
    *feedback = g_feedback;
    return 1U;
}

void PID_Tuning_BuildFeedbackFrame(uint8_t data[8])
{
    if (data == 0) {
        return;
    }
    data[0] = PID_TUNING_PROTOCOL_VERSION;
    PID_Tuning_WriteU16LE(&data[1], g_feedback.transaction_seq);
    data[3] = g_feedback.status;
    data[4] = g_feedback.axis_mask;
    data[5] = 0U;
    data[6] = 0U;
    data[7] = 0U;
}
