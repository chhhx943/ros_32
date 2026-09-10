#include "wheel_calibration_service.h"
#include "wheel_calibration.h"

static Wheel_Calibration_TransactionState_t g_transaction_state;
static Wheel_Calibration_Stage_t g_stage;
static Wheel_Calibration_ExitReason_t g_exit_reason;
static Wheel_Calibration_ServiceRequest_t g_active_request;
static uint8_t g_has_active_request;
static uint8_t g_has_last_start_seq;
static uint16_t g_last_start_seq;
static uint8_t g_has_feedback_seq;
static uint16_t g_last_feedback_seq;
static uint32_t g_transaction_start_ms;
static uint32_t g_stage_start_ms;
static uint32_t g_still_start_ms;
static uint8_t g_still_started;
static uint32_t g_last_feedback_ms;
static uint8_t g_feedback_pending;
static uint8_t g_response_pending;
static int8_t g_left_measured_polarity;
static int8_t g_right_measured_polarity;
static uint16_t g_left_start_pwm;
static uint16_t g_right_start_pwm;
static uint16_t g_stage_pwm;
static uint32_t g_last_pwm_step_ms;
static uint8_t g_stage_response_detected;
static int64_t g_stage_left_start_counts;
static int64_t g_stage_right_start_counts;
static Wheel_Calibration_StageDiagnostics_t g_stage_diagnostics;
static Wheel_Calibration_TerminalSample_t g_terminal_sample;

static uint16_t Wheel_Calibration_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void Wheel_Calibration_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint8_t Wheel_Calibration_IsZeroStop(const BSP_BXCAN_Command_t *command)
{
    return (command != 0) &&
           ((command->mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_STOP) &&
           (command->equivalent_steering_mrad == 0) &&
           (command->rear_left_velocity_mmps == 0) &&
           (command->rear_right_velocity_mmps == 0);
}

static uint8_t Wheel_Calibration_IsSequenceFresh(uint16_t sequence)
{
    int16_t distance;

    if (g_has_last_start_seq == 0U) {
        return 1U;
    }
    distance = (int16_t)(sequence - g_last_start_seq);
    return (distance > 0) && (distance != (int16_t)0x8000);
}

static void Wheel_Calibration_SetState(Wheel_Calibration_TransactionState_t state,
                                       Wheel_Calibration_Stage_t stage,
                                       Wheel_Calibration_ExitReason_t reason)
{
    g_transaction_state = state;
    g_stage = stage;
    g_exit_reason = reason;
    g_feedback_pending = 1U;
}

static void Wheel_Calibration_SetTerminal(Wheel_Calibration_TransactionState_t state,
                                          Wheel_Calibration_ExitReason_t reason)
{
    Wheel_Calibration_SetState(state, WHEEL_CALIBRATION_STAGE_NONE, reason);
    g_has_active_request = 0U;
    g_still_start_ms = 0U;
    g_still_started = 0U;
}

static void Wheel_Calibration_LatchTerminalSample(
    uint32_t now_ms,
    Wheel_Calibration_TransactionState_t state,
    Wheel_Calibration_ExitReason_t reason,
    int64_t stage_delta_counts,
    const EncoderSample_t *left_sample)
{
    g_terminal_sample = (Wheel_Calibration_TerminalSample_t){0};
    g_terminal_sample.valid = (left_sample != 0) ? 1U : 0U;
    g_terminal_sample.terminal_state = state;
    g_terminal_sample.stage = g_stage;
    g_terminal_sample.exit_reason = reason;
    g_terminal_sample.timestamp_ms = now_ms;
    if (left_sample != 0) {
        g_terminal_sample.left_sample = *left_sample;
    }
    g_terminal_sample.stage_delta_counts = stage_delta_counts;
    g_terminal_sample.stage_pwm_permille = g_stage_pwm;
    g_terminal_sample.response_detected = g_stage_response_detected;
}

static void Wheel_Calibration_SetProcessTerminal(
    uint32_t now_ms,
    Wheel_Calibration_TransactionState_t state,
    Wheel_Calibration_ExitReason_t reason,
    int64_t stage_delta_counts,
    const EncoderSample_t *left_sample)
{
    /* Capture before SetTerminal clears the active stage.  The caller's
       sample is the only observation that belongs to this decision cycle. */
    Wheel_Calibration_LatchTerminalSample(now_ms,
                                          state,
                                          reason,
                                          stage_delta_counts,
                                          left_sample);
    Wheel_Calibration_SetTerminal(state, reason);
}

static uint8_t Wheel_Calibration_IsActive(void)
{
    return (g_transaction_state == WHEEL_CALIBRATION_TX_PRECHECK) ||
           (g_transaction_state == WHEEL_CALIBRATION_TX_RUNNING);
}

static uint8_t Wheel_Calibration_AbsVelocityOk(const EncoderSample_t *sample)
{
    int32_t velocity;

    if ((sample == 0) || (sample->trusted == 0U)) {
        return 0U;
    }
    velocity = sample->velocity_mmps;
    if (velocity < 0) {
        velocity = -velocity;
    }
    return (velocity <= WHEEL_CALIBRATION_STILL_SPEED_MAX_MMPS) ? 1U : 0U;
}

static uint8_t Wheel_Calibration_StillHeld(uint32_t now_ms,
                                           const EncoderSample_t *left_sample,
                                           const EncoderSample_t *right_sample)
{
    if ((Wheel_Calibration_AbsVelocityOk(left_sample) == 0U) ||
        (Wheel_Calibration_AbsVelocityOk(right_sample) == 0U)) {
        g_still_start_ms = 0U;
        g_still_started = 0U;
        return 0U;
    }

    if (g_still_started == 0U) {
        g_still_start_ms = now_ms;
        g_still_started = 1U;
    }
    return ((uint32_t)(now_ms - g_still_start_ms) >= WHEEL_CALIBRATION_STOP_HOLD_MS) ? 1U : 0U;
}

static void Wheel_Calibration_StartStage(Wheel_Calibration_Stage_t stage,
                                         uint32_t now_ms,
                                         const EncoderSample_t *left_sample,
                                         const EncoderSample_t *right_sample)
{
    g_stage = stage;
    g_stage_start_ms = now_ms;
    g_still_start_ms = 0U;
    g_still_started = 0U;
    g_stage_left_start_counts = (left_sample != 0) ? left_sample->accumulated_counts : 0;
    g_stage_right_start_counts = (right_sample != 0) ? right_sample->accumulated_counts : 0;
    g_stage_response_detected = 0U;
    g_stage_pwm = 0U;
    g_stage_diagnostics.stage = stage;
    g_stage_diagnostics.left_start_counts = g_stage_left_start_counts;
    g_stage_diagnostics.left_delta_counts = 0;
    g_stage_diagnostics.stage_pwm_permille = 0U;
    g_stage_diagnostics.left_sample_trusted = 0U;
    g_stage_diagnostics.response_detected = 0U;
    if ((stage == WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) ||
        (stage == WHEEL_CALIBRATION_STAGE_LEFT_REVERSE) ||
        (stage == WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD) ||
        (stage == WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE)) {
        g_stage_pwm = WHEEL_CALIBRATION_PWM_INITIAL_PERMILLE;
        g_last_pwm_step_ms = now_ms;
    }
}

static int64_t Wheel_Calibration_StageDelta(const EncoderSample_t *sample, int64_t start_counts)
{
    if (sample == 0) {
        return 0;
    }
    return sample->accumulated_counts - start_counts;
}

static uint8_t Wheel_Calibration_IsMotionStage(void)
{
    return (g_stage == WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) ||
           (g_stage == WHEEL_CALIBRATION_STAGE_LEFT_REVERSE) ||
           (g_stage == WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD) ||
           (g_stage == WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE);
}

static void Wheel_Calibration_UpdatePwmRamp(uint32_t now_ms,
                                            int64_t delta,
                                            int8_t expected_sign)
{
    int64_t response_delta = delta;

    if (expected_sign == 0) {
        response_delta = (delta < 0) ? -delta : delta;
    } else if (expected_sign < 0) {
        response_delta = -delta;
    }

    if (Wheel_Calibration_IsMotionStage() == 0U) {
        return;
    }
    if (response_delta >= WHEEL_CALIBRATION_START_RESPONSE_COUNTS) {
        g_stage_response_detected = 1U;
    }
    if ((g_stage_response_detected == 0U) &&
        ((uint32_t)(now_ms - g_last_pwm_step_ms) >= WHEEL_CALIBRATION_PWM_STEP_HOLD_MS) &&
        (g_stage_pwm < WHEEL_CALIBRATION_PWM_MAX_PERMILLE)) {
        g_stage_pwm = (uint16_t)(g_stage_pwm + WHEEL_CALIBRATION_PWM_STEP_PERMILLE);
        if (g_stage_pwm > WHEEL_CALIBRATION_PWM_MAX_PERMILLE) {
            g_stage_pwm = WHEEL_CALIBRATION_PWM_MAX_PERMILLE;
        }
        g_last_pwm_step_ms = now_ms;
    }
}

static void Wheel_Calibration_FinishSuccess(uint32_t now_ms,
                                            int64_t stage_delta_counts,
                                            const EncoderSample_t *left_sample)
{
    CalibrationData_t data = {1U, g_left_measured_polarity, g_right_measured_polarity,
                              g_left_start_pwm, g_right_start_pwm, 1U, 0U};
    WheelCalibrationParameters_t parameters = {500U, 4U, 28000U, 33250U};
    Wheel_Calibration_SetParameters(&parameters);

    if ((Wheel_Calibration_SetPending(&data) != 0U) &&
        (Wheel_Calibration_CommitPending() != 0U)) {
        Wheel_Calibration_LatchTerminalSample(now_ms,
                                              WHEEL_CALIBRATION_TX_SUCCEEDED,
                                              WHEEL_CALIBRATION_EXIT_SUCCESS,
                                              stage_delta_counts,
                                              left_sample);
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_SUCCEEDED,
                                      WHEEL_CALIBRATION_EXIT_SUCCESS);
    } else {
        Wheel_Calibration_DiscardPending();
        Wheel_Calibration_LatchTerminalSample(
            now_ms,
            WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
            WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION,
            stage_delta_counts,
            left_sample);
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                                      WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION);
    }
}

uint8_t Wheel_Calibration_Service_DecodeRequest(uint8_t dlc,
                                                uint8_t ide,
                                                uint8_t rtr,
                                                const uint8_t data[8],
                                                Wheel_Calibration_ServiceRequest_t *request)
{
    if ((dlc != 8U) || (ide != 0U) || (rtr != 0U) || (data == 0) || (request == 0)) {
        return 0U;
    }
    if (data[0] != BSP_BXCAN_PROTOCOL_VERSION) {
        return 0U;
    }

    request->service_seq = Wheel_Calibration_ReadU16LE(&data[1]);
    request->opcode = data[3];
    request->options = data[4];
    request->service_cookie = Wheel_Calibration_ReadU16LE(&data[5]);
    request->reserved = data[7];
    return 1U;
}

void Wheel_Calibration_Service_EncodeFeedback(const Wheel_Calibration_ServiceFeedback_t *feedback,
                                              uint8_t data[8])
{
    if ((feedback == 0) || (data == 0)) {
        return;
    }
    data[0] = BSP_BXCAN_PROTOCOL_VERSION;
    Wheel_Calibration_WriteU16LE(&data[1], feedback->service_seq);
    data[3] = feedback->transaction_state;
    data[4] = feedback->calibration_stage;
    data[5] = feedback->exit_reason;
    data[6] = feedback->result_flags & 0x07U;
    data[7] = (feedback->progress_percent > 100U) ? 100U : feedback->progress_percent;
}

void Wheel_Calibration_Service_Init(void)
{
    g_transaction_state = WHEEL_CALIBRATION_TX_NONE;
    g_stage = WHEEL_CALIBRATION_STAGE_NONE;
    g_exit_reason = WHEEL_CALIBRATION_EXIT_NONE;
    g_active_request.service_seq = 0U;
    g_active_request.opcode = 0U;
    g_active_request.options = 0U;
    g_active_request.service_cookie = 0U;
    g_active_request.reserved = 0U;
    g_has_active_request = 0U;
    g_has_last_start_seq = 0U;
    g_last_start_seq = 0U;
    g_has_feedback_seq = 0U;
    g_last_feedback_seq = 0U;
    g_transaction_start_ms = 0U;
    g_stage_start_ms = 0U;
    g_still_start_ms = 0U;
    g_still_started = 0U;
    g_stage_left_start_counts = 0;
    g_stage_right_start_counts = 0;
    g_last_feedback_ms = 0U;
    g_feedback_pending = 0U;
    g_response_pending = 0U;
    g_left_measured_polarity = 0;
    g_right_measured_polarity = 0;
    g_left_start_pwm = 0U;
    g_right_start_pwm = 0U;
    g_stage_pwm = 0U;
    g_last_pwm_step_ms = 0U;
    g_stage_response_detected = 0U;
    g_stage_diagnostics = (Wheel_Calibration_StageDiagnostics_t){0};
    g_terminal_sample = (Wheel_Calibration_TerminalSample_t){0};
}

void Wheel_Calibration_Service_OnRequest(const Wheel_Calibration_ServiceRequest_t *request,
                                         uint32_t now_ms,
                                         const BSP_BXCAN_Command_t *command,
                                         uint8_t command_fresh,
                                         uint8_t estop_active,
                                         uint8_t fault_active)
{
    if (request == 0) {
        return;
    }

    g_feedback_pending = 1U;
    g_response_pending = 1U;
    g_has_feedback_seq = 1U;
    g_last_feedback_seq = request->service_seq;

    if ((request->service_cookie != WHEEL_CALIBRATION_SERVICE_COOKIE) ||
        (request->reserved != 0U)) {
        g_active_request = *request;
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                      WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT);
        return;
    }

    if ((request->options & (uint8_t)~WHEEL_CALIBRATION_OPTION_MASK) != 0U) {
        g_active_request = *request;
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                      WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT);
        return;
    }

    if ((request->opcode != WHEEL_CALIBRATION_OPCODE_START) &&
        (request->opcode != WHEEL_CALIBRATION_OPCODE_CANCEL) &&
        (request->opcode != WHEEL_CALIBRATION_OPCODE_QUERY)) {
        g_active_request = *request;
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                      WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT);
        return;
    }

    if (request->opcode == WHEEL_CALIBRATION_OPCODE_START) {
        if ((g_has_last_start_seq != 0U) &&
            (request->service_seq == g_last_start_seq)) {
            return;
        }
        if (Wheel_Calibration_IsActive() != 0U) {
            g_active_request = *request;
            Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                          WHEEL_CALIBRATION_EXIT_REJECTED_BUSY);
            return;
        }
        if (Wheel_Calibration_IsSequenceFresh(request->service_seq) == 0U) {
            g_active_request = *request;
            Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                          WHEEL_CALIBRATION_EXIT_REJECTED_STALE_SEQUENCE);
            return;
        }
        if ((request->options != WHEEL_CALIBRATION_OPTION_MASK) ||
            (command_fresh == 0U) ||
            (Wheel_Calibration_IsZeroStop(command) == 0U) ||
            (estop_active != 0U) || (fault_active != 0U)) {
            g_active_request = *request;
            Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                          WHEEL_CALIBRATION_EXIT_REJECTED_NOT_PERMITTED);
            return;
        }

        g_active_request = *request;
        g_last_start_seq = request->service_seq;
        g_has_last_start_seq = 1U;
        g_terminal_sample.valid = 0U;
        g_has_active_request = 1U;
        g_transaction_start_ms = now_ms;
        g_stage_start_ms = now_ms;
        g_still_start_ms = 0U;
        Wheel_Calibration_SetState(WHEEL_CALIBRATION_TX_PRECHECK,
                                   WHEEL_CALIBRATION_STAGE_PRECHECK,
                                   WHEEL_CALIBRATION_EXIT_NONE);
        return;
    }

    if ((request->opcode == WHEEL_CALIBRATION_OPCODE_QUERY) &&
        (g_has_active_request != 0U) &&
        (request->service_seq == g_active_request.service_seq)) {
        return;
    }

    if ((request->opcode == WHEEL_CALIBRATION_OPCODE_CANCEL) &&
        (g_has_active_request != 0U) &&
        (request->service_seq == g_active_request.service_seq) &&
        ((request->options & WHEEL_CALIBRATION_OPTION_MAINTENANCE_CONFIRM) != 0U)) {
        Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_CANCELED,
                                      WHEEL_CALIBRATION_EXIT_OPERATOR_CANCEL);
        return;
    }

    g_active_request = *request;
    Wheel_Calibration_SetTerminal(WHEEL_CALIBRATION_TX_REJECTED,
                                  (request->opcode == WHEEL_CALIBRATION_OPCODE_CANCEL)
                                      ? WHEEL_CALIBRATION_EXIT_REJECTED_NOT_PERMITTED
                                      : WHEEL_CALIBRATION_EXIT_REJECTED_STALE_SEQUENCE);
}

void Wheel_Calibration_Service_Process(uint32_t now_ms,
                                       const BSP_BXCAN_Command_t *command,
                                       uint8_t command_fresh,
                                       const EncoderSample_t *left_sample,
                                       const EncoderSample_t *right_sample,
                                       uint8_t estop_active,
                                       uint8_t fault_active)
{
    int64_t delta;

    if (Wheel_Calibration_IsActive() == 0U) {
        return;
    }

    if (estop_active != 0U) {
        Wheel_Calibration_SetProcessTerminal(now_ms,
                                             WHEEL_CALIBRATION_TX_ABORTED,
                                             WHEEL_CALIBRATION_EXIT_SAFETY_ESTOP,
                                             0,
                                             left_sample);
        return;
    }
    if (fault_active != 0U) {
        Wheel_Calibration_SetProcessTerminal(now_ms,
                                             WHEEL_CALIBRATION_TX_ABORTED,
                                             WHEEL_CALIBRATION_EXIT_SAFETY_FAULT,
                                             0,
                                             left_sample);
        return;
    }
    if ((command_fresh == 0U) || (Wheel_Calibration_IsZeroStop(command) == 0U)) {
        Wheel_Calibration_SetProcessTerminal(now_ms,
                                             WHEEL_CALIBRATION_TX_ABORTED,
                                             WHEEL_CALIBRATION_EXIT_COMMUNICATION_TIMEOUT,
                                             0,
                                             left_sample);
        return;
    }

    if ((uint32_t)(now_ms - g_transaction_start_ms) >= WHEEL_CALIBRATION_TOTAL_TIMEOUT_MS) {
        Wheel_Calibration_SetProcessTerminal(now_ms,
                                             WHEEL_CALIBRATION_TX_ABORTED,
                                             WHEEL_CALIBRATION_EXIT_TOTAL_TIMEOUT,
                                             0,
                                             left_sample);
        return;
    }

    if (g_transaction_state == WHEEL_CALIBRATION_TX_PRECHECK) {
        if ((uint32_t)(now_ms - g_transaction_start_ms) >= WHEEL_CALIBRATION_PRECHECK_TIMEOUT_MS) {
            Wheel_Calibration_SetProcessTerminal(now_ms,
                                                 WHEEL_CALIBRATION_TX_ABORTED,
                                                 WHEEL_CALIBRATION_EXIT_PRECHECK_TIMEOUT,
                                                 0,
                                                 left_sample);
            return;
        }
        if (Wheel_Calibration_StillHeld(now_ms, left_sample, right_sample) != 0U) {
            g_transaction_state = WHEEL_CALIBRATION_TX_RUNNING;
            Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_LEFT_FORWARD,
                                         now_ms,
                                         left_sample,
                                         right_sample);
        }
        return;
    }

    switch (g_stage) {
    case WHEEL_CALIBRATION_STAGE_LEFT_FORWARD:
        delta = Wheel_Calibration_StageDelta(left_sample, g_stage_left_start_counts);
        Wheel_Calibration_UpdatePwmRamp(now_ms, delta, 0);
        g_stage_diagnostics.left_delta_counts = delta;
        g_stage_diagnostics.stage_pwm_permille = g_stage_pwm;
        g_stage_diagnostics.left_sample_trusted =
            (left_sample != 0) ? left_sample->trusted : 0U;
        g_stage_diagnostics.response_detected = g_stage_response_detected;
        if ((uint32_t)(now_ms - g_stage_start_ms) < WHEEL_CALIBRATION_STAGE_MS) {
            return;
        }
        if ((left_sample == 0) || (left_sample->trusted == 0U) ||
            (((delta < 0) ? -delta : delta) < WHEEL_CALIBRATION_MIN_RESPONSE_COUNTS) ||
            (g_stage_response_detected == 0U)) {
            Wheel_Calibration_SetProcessTerminal(
                now_ms,
                WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION,
                delta,
                left_sample);
            return;
        }
        g_left_measured_polarity = (delta >= 0) ? 1 : -1;
        g_left_start_pwm = g_stage_pwm;
        Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_LEFT_SETTLE,
                                     now_ms,
                                     left_sample,
                                     right_sample);
        return;

    case WHEEL_CALIBRATION_STAGE_LEFT_SETTLE:
        if ((uint32_t)(now_ms - g_stage_start_ms) >= WHEEL_CALIBRATION_SETTLE_TIMEOUT_MS) {
            Wheel_Calibration_SetProcessTerminal(now_ms,
                                                 WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                                                 WHEEL_CALIBRATION_EXIT_SETTLE_TIMEOUT,
                                                 0,
                                                 left_sample);
        } else if (Wheel_Calibration_StillHeld(now_ms, left_sample, right_sample) != 0U) {
            Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_LEFT_REVERSE,
                                         now_ms,
                                         left_sample,
                                         right_sample);
        }
        return;

    case WHEEL_CALIBRATION_STAGE_LEFT_REVERSE:
        delta = Wheel_Calibration_StageDelta(left_sample, g_stage_left_start_counts);
        Wheel_Calibration_UpdatePwmRamp(now_ms, delta, (int8_t)-g_left_measured_polarity);
        if ((uint32_t)(now_ms - g_stage_start_ms) < WHEEL_CALIBRATION_STAGE_MS) {
            return;
        }
        if ((left_sample == 0) || (left_sample->trusted == 0U) ||
            ((delta * (int64_t)g_left_measured_polarity) >
             -WHEEL_CALIBRATION_MIN_RESPONSE_COUNTS) ||
            (g_stage_response_detected == 0U)) {
            Wheel_Calibration_SetProcessTerminal(
                now_ms,
                WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                WHEEL_CALIBRATION_EXIT_REVERSE_INSUFFICIENT_MOTION,
                delta,
                left_sample);
            return;
        }
        Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD,
                                     now_ms,
                                     left_sample,
                                     right_sample);
        return;

    case WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD:
        delta = Wheel_Calibration_StageDelta(right_sample, g_stage_right_start_counts);
        Wheel_Calibration_UpdatePwmRamp(now_ms, delta, 0);
        if ((uint32_t)(now_ms - g_stage_start_ms) < WHEEL_CALIBRATION_STAGE_MS) {
            return;
        }
        if ((right_sample == 0) || (right_sample->trusted == 0U) ||
            (((delta < 0) ? -delta : delta) < WHEEL_CALIBRATION_MIN_RESPONSE_COUNTS) ||
            (g_stage_response_detected == 0U)) {
            Wheel_Calibration_SetProcessTerminal(
                now_ms,
                WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION,
                delta,
                left_sample);
            return;
        }
        g_right_measured_polarity = (delta >= 0) ? 1 : -1;
        g_right_start_pwm = g_stage_pwm;
        Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_RIGHT_SETTLE,
                                     now_ms,
                                     left_sample,
                                     right_sample);
        return;

    case WHEEL_CALIBRATION_STAGE_RIGHT_SETTLE:
        if ((uint32_t)(now_ms - g_stage_start_ms) >= WHEEL_CALIBRATION_SETTLE_TIMEOUT_MS) {
            Wheel_Calibration_SetProcessTerminal(now_ms,
                                                 WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                                                 WHEEL_CALIBRATION_EXIT_SETTLE_TIMEOUT,
                                                 0,
                                                 left_sample);
        } else if (Wheel_Calibration_StillHeld(now_ms, left_sample, right_sample) != 0U) {
            Wheel_Calibration_StartStage(WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE,
                                         now_ms,
                                         left_sample,
                                         right_sample);
        }
        return;

    case WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE:
        delta = Wheel_Calibration_StageDelta(right_sample, g_stage_right_start_counts);
        Wheel_Calibration_UpdatePwmRamp(now_ms, delta, (int8_t)-g_right_measured_polarity);
        if ((uint32_t)(now_ms - g_stage_start_ms) < WHEEL_CALIBRATION_STAGE_MS) {
            return;
        }
        if ((right_sample == 0) || (right_sample->trusted == 0U) ||
            ((delta * (int64_t)g_right_measured_polarity) >
             -WHEEL_CALIBRATION_MIN_RESPONSE_COUNTS) ||
            (g_stage_response_detected == 0U)) {
            Wheel_Calibration_SetProcessTerminal(
                now_ms,
                WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
                WHEEL_CALIBRATION_EXIT_REVERSE_INSUFFICIENT_MOTION,
                delta,
                left_sample);
            return;
        }
        Wheel_Calibration_FinishSuccess(now_ms, delta, left_sample);
        return;

    default:
        Wheel_Calibration_SetProcessTerminal(
            now_ms,
            WHEEL_CALIBRATION_TX_FAILED_VALIDATION,
            WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION,
            0,
            left_sample);
        return;
    }
}

uint8_t Wheel_Calibration_Service_GetRecommendation(Wheel_Calibration_Recommendation_t *recommendation)
{
    if (recommendation == 0) {
        return 0U;
    }
    recommendation->left_pwm = 0;
    recommendation->right_pwm = 0;

    if (g_transaction_state != WHEEL_CALIBRATION_TX_RUNNING) {
        return 0U;
    }

    if (g_stage == WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) {
        recommendation->left_pwm = (int16_t)g_stage_pwm;
    } else if (g_stage == WHEEL_CALIBRATION_STAGE_LEFT_REVERSE) {
        recommendation->left_pwm = -(int16_t)g_stage_pwm;
    } else if (g_stage == WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD) {
        recommendation->right_pwm = (int16_t)g_stage_pwm;
    } else if (g_stage == WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE) {
        recommendation->right_pwm = -(int16_t)g_stage_pwm;
    }
    return 1U;
}

void Wheel_Calibration_Service_GetStageDiagnostics(
    Wheel_Calibration_StageDiagnostics_t *diagnostics)
{
    if (diagnostics != 0) {
        *diagnostics = g_stage_diagnostics;
    }
}

void Wheel_Calibration_Service_GetTerminalSample(
    Wheel_Calibration_TerminalSample_t *sample)
{
    if (sample != 0) {
        *sample = g_terminal_sample;
    }
}

Wheel_Calibration_TransactionState_t Wheel_Calibration_Service_GetState(void)
{
    return g_transaction_state;
}

Wheel_Calibration_Stage_t Wheel_Calibration_Service_GetStage(void)
{
    return g_stage;
}

Wheel_Calibration_ExitReason_t Wheel_Calibration_Service_GetExitReason(void)
{
    return g_exit_reason;
}

uint8_t Wheel_Calibration_Service_GetFeedback(Wheel_Calibration_ServiceFeedback_t *feedback,
                                              uint8_t response_to_request)
{
    uint8_t progress = 0U;

    if ((feedback == 0) ||
        (g_has_active_request == 0U && g_has_feedback_seq == 0U)) {
        return 0U;
    }

    if (g_stage >= WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) {
        progress = (uint8_t)((g_stage - WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) * 14U);
    }
    if (g_transaction_state == WHEEL_CALIBRATION_TX_SUCCEEDED) {
        progress = 100U;
    }
    if (progress > 100U) {
        progress = 100U;
    }

    feedback->service_seq = (g_has_active_request != 0U)
                                ? g_active_request.service_seq
                                : g_last_feedback_seq;
    feedback->transaction_state = (uint8_t)g_transaction_state;
    feedback->calibration_stage = (uint8_t)g_stage;
    feedback->exit_reason = (uint8_t)g_exit_reason;
    feedback->result_flags = 0U;
    if (!Wheel_Calibration_IsActive()) {
        feedback->result_flags |= WHEEL_CALIBRATION_RESULT_TERMINAL;
    }
    if (g_transaction_state == WHEEL_CALIBRATION_TX_SUCCEEDED) {
        feedback->result_flags |= WHEEL_CALIBRATION_RESULT_SUCCESS;
    }
    if (response_to_request != 0U) {
        feedback->result_flags |= WHEEL_CALIBRATION_RESULT_RESPONSE_TO_REQUEST;
    }
    feedback->progress_percent = progress;
    return 1U;
}

uint8_t Wheel_Calibration_Service_ShouldPublish(uint32_t now_ms)
{
    if (g_feedback_pending != 0U) {
        return 1U;
    }
    return Wheel_Calibration_IsActive() &&
           ((uint32_t)(now_ms - g_last_feedback_ms) >= WHEEL_CALIBRATION_SERVICE_PERIOD_MS);
}

void Wheel_Calibration_Service_MarkPublished(uint32_t now_ms)
{
    g_last_feedback_ms = now_ms;
    g_feedback_pending = 0U;
    g_response_pending = 0U;
}

uint8_t Wheel_Calibration_Service_HasResponsePending(void)
{
    return g_response_pending;
}

uint8_t Wheel_Calibration_Service_IsActive(void)
{
    return Wheel_Calibration_IsActive();
}

uint8_t Wheel_Calibration_Service_ClearFailedValidation(void)
{
    if (g_transaction_state != WHEEL_CALIBRATION_TX_FAILED_VALIDATION) {
        return 0U;
    }

    /* A calibration-validation fault is recoverable only after the safety
       manager has accepted a zero STOP + RESET command.  Keep the last exit
       reason for diagnostics, but leave no failed transaction state for the
       next Safety_Manager_Process() pass to re-latch. */
    g_transaction_state = WHEEL_CALIBRATION_TX_NONE;
    g_stage = WHEEL_CALIBRATION_STAGE_NONE;
    g_has_active_request = 0U;
    g_still_start_ms = 0U;
    g_still_started = 0U;
    g_stage_response_detected = 0U;
    g_stage_pwm = 0U;
    g_feedback_pending = 1U;
    return 1U;
}
