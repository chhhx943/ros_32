#include "chassis_control.h"
#include "bsp_bxcan.h"
#include "bsp_motor.h"
#include "encoder.h"
#include "PID.h"
#include "safety_manager.h"
#include "wheel_calibration_service.h"
#ifndef CHASSIS_CONTROL_HOST_TEST
#include "servo.h"
#endif
#include "pid_tuning.h"
#include "autotune_safe.h"
#ifdef AUTOTUNE_SAFE_PROFILE
#include "autotune_safe_local.h"
#endif
#ifndef CHASSIS_CONTROL_HOST_TEST
#include "watchdog.h"
#endif

#define CHASSIS_CONTROL_LEFT_MOTOR  1U
#define CHASSIS_CONTROL_RIGHT_MOTOR 2U
#define CHASSIS_CONTROL_PERIOD_MS   10U
#define CHASSIS_CONTROL_COUNTS_PER_WHEEL_REV 56000LL
#define CHASSIS_CONTROL_WHEEL_REV_MRAD       6283LL

static BSP_BXCAN_Command_t g_target_command;
static uint8_t g_has_target_command;
static uint8_t g_scheduler_started;
static uint8_t g_safety_action_applied;
static uint32_t g_last_control_ms;
static Safety_State_t g_last_safety_state;
static Safety_Action_t g_last_safety_action;
static PID_t g_left_pid;
static PID_t g_right_pid;
static BSP_BXCAN_Feedback_t g_feedback_snapshot;
static uint8_t g_static_steering_active;
static volatile uint8_t g_tick_events;
static volatile uint8_t g_tick_overrun;
static Chassis_ControlTelemetry_t g_telemetry;

static void Chassis_ControlPublishSafetyFeedback(void);
static void Chassis_ControlPublishDiagnostics(uint8_t left_encoder_invalid,
                                              uint8_t right_encoder_invalid);

#ifdef AUTOTUNE_SAFE_PROFILE
static int16_t Chassis_ControlAbsI16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static void Chassis_ControlAutotunePreflight(uint32_t now_ms, uint32_t dt_ms)
{
    EncoderSample_t left_sample = Encoder_Sample(CHASSIS_CONTROL_LEFT_MOTOR, dt_ms);
    EncoderSample_t right_sample = Encoder_Sample(CHASSIS_CONTROL_RIGHT_MOTOR, dt_ms);
    uint8_t safety_ok = (Safety_Manager_GetState() == SAFETY_STATE_STANDBY) &&
                        (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE);
    uint8_t still = (left_sample.trusted != 0U) &&
                    (right_sample.trusted != 0U) &&
                    (Chassis_ControlAbsI16((int16_t)left_sample.velocity_mmps) <= 5) &&
                    (Chassis_ControlAbsI16((int16_t)right_sample.velocity_mmps) <= 5);

    AutotuneSafe_RecordStillness(now_ms,
                                 (int16_t)left_sample.velocity_mmps,
                                 (int16_t)right_sample.velocity_mmps);

    (void)AutotuneSafe_ConfirmPreflight(safety_ok,
                                        (Safety_Manager_IsEstopActive() == 0U) ? 1U : 0U,
                                        still,
                                        1U);
}
#endif

static void Chassis_ControlUpdatePidTelemetry(uint32_t now_ms,
                                              uint32_t dt_ms,
                                              const EncoderSample_t *left_sample,
                                              const EncoderSample_t *right_sample,
                                              int16_t left_pwm,
                                              int16_t right_pwm)
{
    float dt_s = ((float)dt_ms) / 1000.0f;

    g_telemetry.timestamp_ms = now_ms;
    g_telemetry.command_seq = g_target_command.command_seq;
    g_telemetry.target_left_mmps = g_target_command.rear_left_velocity_mmps;
    g_telemetry.target_right_mmps = g_target_command.rear_right_velocity_mmps;
    g_telemetry.actual_left_mmps = (left_sample != 0) ? (int16_t)left_sample->velocity_mmps : 0;
    g_telemetry.actual_right_mmps = (right_sample != 0) ? (int16_t)right_sample->velocity_mmps : 0;
    g_telemetry.encoder_delta_left = (left_sample != 0) ? left_sample->delta_counts : 0;
    g_telemetry.encoder_delta_right = (right_sample != 0) ? right_sample->delta_counts : 0;
    g_telemetry.pwm_left = left_pwm;
    g_telemetry.pwm_right = right_pwm;
    g_telemetry.pid_left_p = g_left_pid.Kp * g_left_pid.Error0;
    g_telemetry.pid_left_i = g_left_pid.Ki * g_left_pid.ErrorInt;
    g_telemetry.pid_left_d = (dt_s > 0.0f) ?
        (g_left_pid.Kd * (g_left_pid.Error0 - g_left_pid.Error1) / dt_s) : 0.0f;
    g_telemetry.pid_left_output = g_left_pid.Out;
    g_telemetry.pid_right_p = g_right_pid.Kp * g_right_pid.Error0;
    g_telemetry.pid_right_i = g_right_pid.Ki * g_right_pid.ErrorInt;
    g_telemetry.pid_right_d = (dt_s > 0.0f) ?
        (g_right_pid.Kd * (g_right_pid.Error0 - g_right_pid.Error1) / dt_s) : 0.0f;
    g_telemetry.pid_right_output = g_right_pid.Out;
    g_telemetry.safety_state = (uint8_t)Safety_Manager_GetState();
    g_telemetry.fault_code = Safety_Manager_GetFault();
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_RecordWheelTelemetry(1U,
                                      g_telemetry.actual_left_mmps,
                                      left_pwm,
                                      g_telemetry.pid_left_p,
                                      g_telemetry.pid_left_i,
                                      g_telemetry.pid_left_d,
                                      g_telemetry.pid_left_output);
    AutotuneSafe_RecordWheelTelemetry(2U,
                                      g_telemetry.actual_right_mmps,
                                      right_pwm,
                                      g_telemetry.pid_right_p,
                                      g_telemetry.pid_right_i,
                                      g_telemetry.pid_right_d,
                                      g_telemetry.pid_right_output);
    AutotuneSafe_SetSafetyTelemetry(g_telemetry.safety_state,
                                    g_telemetry.fault_code);
#endif
}

static int16_t Chassis_ControlClampPwm(float pwm)
{
    if (pwm > 1000) {
        return 1000;
    }
    if (pwm < -1000) {
        return -1000;
    }
    return (int16_t)pwm;
}

static void Chassis_ControlConfigurePid(PID_t *pid)
{
    pid->Target = 0.0f;
    pid->Actual = 0.0f;
    pid->Kp = 0.20f;
    pid->Ki = 0.60f;
    pid->Kd = 0.0f;
    pid->OutMax = 1000.0f;
    pid->OutMin = -1000.0f;
    PID_Reset(pid);
}

static void Chassis_ControlResetWheelControllers(void)
{
    PID_Reset(&g_left_pid);
    PID_Reset(&g_right_pid);
}

static void Chassis_ControlRefreshPidGains(void)
{
#ifndef CHASSIS_CONTROL_HOST_TEST
    PID_Tuning_Gains_t left_gains;
    PID_Tuning_Gains_t right_gains;

    PID_Tuning_GetGains(&left_gains, &right_gains);
    g_left_pid.Kp = left_gains.kp;
    g_left_pid.Ki = left_gains.ki;
    g_left_pid.Kd = left_gains.kd;
    g_right_pid.Kp = right_gains.kp;
    g_right_pid.Ki = right_gains.ki;
    g_right_pid.Kd = right_gains.kd;
#endif
}

static uint8_t Chassis_ControlApplyStaticSteering(void)
{
#ifdef CHASSIS_CONTROL_HOST_TEST
    return 0U;
#else
    if ((g_has_target_command == 0U) ||
        (Safety_Manager_GetState() != SAFETY_STATE_STANDBY) ||
        (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_NONE) ||
        (BSP_BXCAN_IsCommandFresh() == 0U) ||
        ((g_target_command.mode_flags & BSP_BXCAN_MODE_MASK) != BSP_BXCAN_MODE_VELOCITY) ||
        (g_target_command.rear_left_velocity_mmps != 0) ||
        (g_target_command.rear_right_velocity_mmps != 0)) {
        return 0U;
    }

    /* Zero wheel speed with a non-zero steering target is a legal static
       steering/calibration command. Keep the wheels coasted while allowing
       the servo to follow the command instead of forcing it to center. */
    /* Equivalent to the legacy Servo_SetAngleMrad(g_target_command.equivalent_steering_mrad)
       call, but with explicit envelope rejection and rate limiting. */
    if (Servo_SetAngleMradChecked(g_target_command.equivalent_steering_mrad) == 0U) {
        return 0U;
    }
    g_static_steering_active = 1U;
    Motor_CoastAll();
    Chassis_ControlResetWheelControllers();
    return 1U;
#endif
}

static void Chassis_ControlApplySafetyAction(void)
{
    Safety_Action_t action = Safety_Manager_GetAction();
    Safety_State_t state = Safety_Manager_GetState();

    if ((g_safety_action_applied != 0U) &&
        (g_last_safety_state == state) &&
        (g_last_safety_action == action)) {
#ifndef CHASSIS_CONTROL_HOST_TEST
        if ((g_has_target_command != 0U) &&
            ((g_target_command.mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_STOP)) {
            Servo_SetNeutral();
            g_static_steering_active = 0U;
        }
#endif
        Chassis_ControlPublishDiagnostics(0U, 0U);
        return;
    }

    g_safety_action_applied = 1U;
    g_last_safety_state = state;
    g_last_safety_action = action;

    if (action == SAFETY_ACTION_BRAKE) {
#ifndef CHASSIS_CONTROL_HOST_TEST
        Servo_SetNeutral();
#endif
        Motor_EmergencyBrakeAll();
        Chassis_ControlResetWheelControllers();
        Chassis_ControlPublishSafetyFeedback();
        return;
    }

    if (action == SAFETY_ACTION_COAST) {
#ifndef CHASSIS_CONTROL_HOST_TEST
        Servo_SetNeutral();
#endif
        Motor_CoastAll();
        Chassis_ControlResetWheelControllers();
        Chassis_ControlPublishSafetyFeedback();
        return;
    }

    Chassis_ControlPublishDiagnostics(0U, 0U);
}

static int32_t Chassis_ControlCountsToPositionMrad(int64_t accumulated_counts)
{
    int64_t position_mrad = (accumulated_counts * CHASSIS_CONTROL_WHEEL_REV_MRAD) /
                            CHASSIS_CONTROL_COUNTS_PER_WHEEL_REV;

    if (position_mrad > 2147483647LL) {
        return 2147483647L;
    }
    if (position_mrad < -2147483648LL) {
        return (-2147483647L - 1L);
    }
    return (int32_t)position_mrad;
}

static void Chassis_ControlPublishFeedback(const EncoderSample_t *left_sample,
                                           const EncoderSample_t *right_sample)
{
    BSP_BXCAN_Feedback_t feedback = {0};

    if (Safety_Manager_IsEstopActive() != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_ESTOP_ACTIVE;
    }
    if (Safety_Manager_IsSafeStopActive() != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_SAFE_STOP_ACTIVE;
    }

    if (left_sample->trusted != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_LEFT_ENCODER_OK;
        feedback.velocity_flags |= BSP_BXCAN_VELOCITY_LEFT_VALID;
        feedback.rear_left_velocity_mmps = (int16_t)left_sample->velocity_mmps;
        feedback.rear_left_position_valid = 1U;
        feedback.rear_left_position_mrad =
            Chassis_ControlCountsToPositionMrad(left_sample->accumulated_counts);
    }

    feedback.left_control_output = g_feedback_snapshot.left_control_output;
    feedback.right_control_output = g_feedback_snapshot.right_control_output;

    if (right_sample->trusted != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_RIGHT_ENCODER_OK;
        feedback.velocity_flags |= BSP_BXCAN_VELOCITY_RIGHT_VALID;
        feedback.rear_right_velocity_mmps = (int16_t)right_sample->velocity_mmps;
        feedback.rear_right_position_valid = 1U;
        feedback.rear_right_position_mrad =
            Chassis_ControlCountsToPositionMrad(right_sample->accumulated_counts);
    }

    if ((Safety_Manager_IsEstopActive() != 0U) ||
        (Safety_Manager_IsSafeStopActive() != 0U)) {
        feedback.left_control_output = 0;
        feedback.right_control_output = 0;
    }

    g_feedback_snapshot = feedback;
    BSP_BXCAN_SetFeedback(&feedback);
    Chassis_ControlPublishDiagnostics((left_sample->trusted == 0U) ? 1U : 0U,
                                      (right_sample->trusted == 0U) ? 1U : 0U);
}

static void Chassis_ControlPublishSafetyFeedback(void)
{
    BSP_BXCAN_Feedback_t feedback = g_feedback_snapshot;

    if (Safety_Manager_IsEstopActive() != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_ESTOP_ACTIVE;
    }
    if (Safety_Manager_IsSafeStopActive() != 0U) {
        feedback.status_flags |= BSP_BXCAN_STATUS_SAFE_STOP_ACTIVE;
    }

    if ((Safety_Manager_IsEstopActive() != 0U) ||
        (Safety_Manager_IsSafeStopActive() != 0U)) {
        feedback.left_control_output = 0;
        feedback.right_control_output = 0;
        g_feedback_snapshot = feedback;
        BSP_BXCAN_SetFeedback(&feedback);
    }
    Chassis_ControlPublishDiagnostics(
        ((feedback.status_flags & BSP_BXCAN_STATUS_LEFT_ENCODER_OK) == 0U) ? 1U : 0U,
        ((feedback.status_flags & BSP_BXCAN_STATUS_RIGHT_ENCODER_OK) == 0U) ? 1U : 0U);
}

static void Chassis_ControlPublishDiagnostics(uint8_t left_encoder_invalid,
                                              uint8_t right_encoder_invalid)
{
    BSP_BXCAN_Diagnostics_t diagnostics;
    uint16_t fault;

    BSP_BXCAN_GetDiagnostics(&diagnostics);
    diagnostics.safety_state = (uint8_t)Safety_Manager_GetState();
    diagnostics.safety_action = (uint8_t)Safety_Manager_GetAction();
    diagnostics.flags &= BSP_BXCAN_DIAG_CAN_ERROR;
    if (Safety_Manager_IsCalibrationRequired() != 0U) {
        diagnostics.flags |= BSP_BXCAN_DIAG_CALIBRATION_REQUIRED;
    }
    if (Safety_Manager_DriveAllowed() != 0U) {
        diagnostics.flags |= BSP_BXCAN_DIAG_DRIVE_ALLOWED;
    }
    if (BSP_BXCAN_IsCommandFresh() != 0U) {
        diagnostics.flags |= BSP_BXCAN_DIAG_COMMAND_FRESH;
    }
    if (left_encoder_invalid != 0U) {
        diagnostics.flags |= BSP_BXCAN_DIAG_LEFT_ENCODER_INVALID;
    }
    if (right_encoder_invalid != 0U) {
        diagnostics.flags |= BSP_BXCAN_DIAG_RIGHT_ENCODER_INVALID;
    }
    fault = Safety_Manager_GetFault();
    if (fault == BSP_BXCAN_FAULT_CONTROL_OVERRUN) {
        diagnostics.flags |= BSP_BXCAN_DIAG_CONTROL_OVERRUN;
    }
    if (fault == BSP_BXCAN_FAULT_MOTOR_STALL) {
        diagnostics.flags |= BSP_BXCAN_DIAG_MOTOR_STALL;
    }
    BSP_BXCAN_SetDiagnostics(&diagnostics);
}

static void Chassis_ControlApplyClosedLoop(uint32_t now_ms, uint32_t dt_ms)
{
    EncoderSample_t left_sample;
    EncoderSample_t right_sample;
    Wheel_Calibration_Recommendation_t calibration_recommendation;
    float dt_s = ((float)dt_ms) / 1000.0f;

    /* Encoder sampling is read-only and must continue in every safety state.
       Otherwise feedback freezes at the last DRIVE/calibration sample, which
       makes the CAN position and velocity fields stale while the controller
       is correctly coasting. */
    left_sample = Encoder_Sample(CHASSIS_CONTROL_LEFT_MOTOR, dt_ms);
    right_sample = Encoder_Sample(CHASSIS_CONTROL_RIGHT_MOTOR, dt_ms);
    Safety_Manager_ReportEncoderSample(CHASSIS_CONTROL_LEFT_MOTOR,
                                       left_sample.trusted,
                                       left_sample.velocity_mmps,
                                       dt_ms);
    Safety_Manager_ReportEncoderSample(CHASSIS_CONTROL_RIGHT_MOTOR,
                                       right_sample.trusted,
                                       right_sample.velocity_mmps,
                                       dt_ms);
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_RecordStillness(now_ms,
                                 (int16_t)left_sample.velocity_mmps,
                                 (int16_t)right_sample.velocity_mmps);
#endif

    if (Wheel_Calibration_Service_IsActive() != 0U) {
        Wheel_Calibration_Service_Process(now_ms,
                                           &g_target_command,
                                           BSP_BXCAN_IsCommandFresh(),
                                           &left_sample,
                                           &right_sample,
                                            Safety_Manager_IsEstopActive(),
                                            (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_NONE) ? 1U : 0U);
        Safety_Manager_Process(now_ms);
        Chassis_ControlPublishFeedback(&left_sample, &right_sample);

        if (Safety_Manager_GetAction() != SAFETY_ACTION_CALIBRATION) {
            Chassis_ControlApplySafetyAction();
            return;
        }

        if ((Wheel_Calibration_Service_GetRecommendation(&calibration_recommendation) == 0U) ||
            ((calibration_recommendation.left_pwm == 0) &&
             (calibration_recommendation.right_pwm == 0))) {
            Motor_CoastAll();
            Chassis_ControlResetWheelControllers();
            return;
        }

        Chassis_ControlResetWheelControllers();
        if (calibration_recommendation.left_pwm != 0) {
            Motor_Drive(CHASSIS_CONTROL_LEFT_MOTOR, calibration_recommendation.left_pwm);
        } else {
            Motor_Coast(CHASSIS_CONTROL_LEFT_MOTOR);
        }
        if (calibration_recommendation.right_pwm != 0) {
            Motor_Drive(CHASSIS_CONTROL_RIGHT_MOTOR, calibration_recommendation.right_pwm);
        } else {
            Motor_Coast(CHASSIS_CONTROL_RIGHT_MOTOR);
        }
        return;
    }

    if ((g_has_target_command == 0U) ||
        (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE)) {
        Chassis_ControlPublishFeedback(&left_sample, &right_sample);
        if (Chassis_ControlApplyStaticSteering() != 0U) {
            return;
        }
        Chassis_ControlApplySafetyAction();
        return;
    }

    if (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE) {
        Chassis_ControlPublishFeedback(&left_sample, &right_sample);
        Chassis_ControlApplySafetyAction();
        return;
    }
    if ((left_sample.trusted == 0U) || (right_sample.trusted == 0U)) {
#ifndef CHASSIS_CONTROL_HOST_TEST
        Servo_SetNeutral();
#endif
        g_feedback_snapshot.left_control_output = 0;
        g_feedback_snapshot.right_control_output = 0;
        Chassis_ControlPublishFeedback(&left_sample, &right_sample);
        Motor_CoastAll();
        Chassis_ControlResetWheelControllers();
        return;
    }

    Chassis_ControlRefreshPidGains();

#ifdef AUTOTUNE_SAFE_PROFILE
    g_left_pid.Target = (float)AutotuneSafe_GetEffectiveTarget();
#else
    g_left_pid.Target = (float)g_target_command.rear_left_velocity_mmps;
#endif
    g_left_pid.Actual = (float)left_sample.velocity_mmps;
    PID_UpdateDt(&g_left_pid, dt_s);

#ifdef AUTOTUNE_SAFE_PROFILE
    g_right_pid.Target = (float)AutotuneSafe_GetEffectiveTarget();
#else
    g_right_pid.Target = (float)g_target_command.rear_right_velocity_mmps;
#endif
    g_right_pid.Actual = (float)right_sample.velocity_mmps;
    PID_UpdateDt(&g_right_pid, dt_s);

    {
        int16_t left_pwm = Chassis_ControlClampPwm(g_left_pid.Out);
        int16_t right_pwm = Chassis_ControlClampPwm(g_right_pid.Out);

        Safety_Manager_ReportDriveObservation(CHASSIS_CONTROL_LEFT_MOTOR,
                                              g_target_command.rear_left_velocity_mmps,
                                              (int16_t)left_sample.velocity_mmps,
                                              left_pwm,
                                              now_ms);
        Safety_Manager_ReportDriveObservation(CHASSIS_CONTROL_RIGHT_MOTOR,
                                              g_target_command.rear_right_velocity_mmps,
                                              (int16_t)right_sample.velocity_mmps,
                                              right_pwm,
                                              now_ms);
        if (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE) {
            Chassis_ControlApplySafetyAction();
            return;
        }
#ifdef AUTOTUNE_SAFE_PROFILE
        AutotuneSafe_RecordSample(now_ms,
                                  (int16_t)g_left_pid.Target,
                                  (int16_t)left_sample.velocity_mmps,
                                  left_pwm,
                                  (left_sample.trusted != 0U) ? 1U : 0U,
                                  (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE) ? 1U : 0U,
                                  (Safety_Manager_IsEstopActive() == 0U) ? 1U : 0U);
        AutotuneSafe_RecordSample(now_ms,
                                  (int16_t)g_right_pid.Target,
                                  (int16_t)right_sample.velocity_mmps,
                                  right_pwm,
                                  (right_sample.trusted != 0U) ? 1U : 0U,
                                  (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE) ? 1U : 0U,
                                  (Safety_Manager_IsEstopActive() == 0U) ? 1U : 0U);
#endif
#ifndef CHASSIS_CONTROL_HOST_TEST
        if (Servo_SetAngleMradChecked(g_target_command.equivalent_steering_mrad) == 0U) {
            Safety_Manager_ReportFault(BSP_BXCAN_FAULT_STEERING_REJECTED);
            Chassis_ControlApplySafetyAction();
            return;
        }
#endif
        Motor_Drive(CHASSIS_CONTROL_LEFT_MOTOR, left_pwm);
        Motor_Drive(CHASSIS_CONTROL_RIGHT_MOTOR, right_pwm);
        g_feedback_snapshot.left_control_output = left_pwm;
        g_feedback_snapshot.right_control_output = right_pwm;
        Chassis_ControlUpdatePidTelemetry(now_ms,
                                          dt_ms,
                                          &left_sample,
                                          &right_sample,
                                          left_pwm,
                                          right_pwm);
        Chassis_ControlPublishFeedback(&left_sample, &right_sample);
    }
}

void Chassis_ControlInit(void)
{
    BSP_BXCAN_Init();
    Safety_Manager_Init();
    Motor_Init();
#ifndef CHASSIS_CONTROL_HOST_TEST
    Servo_Init();
#endif
    Encoder_Init();
#ifndef CHASSIS_CONTROL_HOST_TEST
    PID_Tuning_Init();
#endif
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_LocalExperiment_Init();
#endif
    Chassis_ControlConfigurePid(&g_left_pid);
    Chassis_ControlConfigurePid(&g_right_pid);
    Chassis_ControlRefreshPidGains();
    g_has_target_command = 0U;
    g_scheduler_started = 0U;
    g_safety_action_applied = 0U;
    g_last_control_ms = 0U;
    g_last_safety_state = SAFETY_STATE_BOOT;
    g_last_safety_action = SAFETY_ACTION_COAST;
    g_feedback_snapshot = (BSP_BXCAN_Feedback_t){0};
    g_static_steering_active = 0U;
    g_tick_events = 0U;
    g_tick_overrun = 0U;
    g_telemetry = (Chassis_ControlTelemetry_t){0};
    Motor_CoastAll();
}

void Chassis_ControlOnTick(void)
{
    if (g_tick_events < 20U) {
        g_tick_events++;
    } else {
        g_tick_overrun = 1U;
    }
}

void Chassis_ControlProcessEvents(uint32_t now_ms)
{
    uint8_t events;

    events = g_tick_events;
    g_tick_events = 0U;
    if (g_tick_overrun != 0U) {
        /* Never integrate PID with a backlog-sized dt.  Latch the safety
           fault and let the normal arbitration coast/brake the outputs. */
        g_tick_overrun = 0U;
        Safety_Manager_ReportFault(BSP_BXCAN_FAULT_CONTROL_OVERRUN);
        Chassis_ControlProcess(now_ms);
        return;
    }
    if (events != 0U) {
        Chassis_ControlProcess(now_ms);
#ifndef CHASSIS_CONTROL_HOST_TEST
        BSP_Watchdog_Feed();
#endif
    }
}

void Chassis_ControlProcess(uint32_t now_ms)
{
    BSP_BXCAN_Command_t command;

#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_Process(now_ms);
    {
        AutotuneSafe_Status_t autotune_status;
        AutotuneSafe_GetStatus(&autotune_status);
        if ((autotune_status.state == AUTOTUNE_SAFE_STATE_STOPPING) ||
            (autotune_status.state == AUTOTUNE_SAFE_STATE_ABORT) ||
            (autotune_status.state == AUTOTUNE_SAFE_STATE_COOLING)) {
            Chassis_ControlResetWheelControllers();
        }
    }
#endif
#ifndef CHASSIS_CONTROL_HOST_TEST
    Motor_Process(now_ms);
#endif
    Safety_Manager_Process(now_ms);
#ifdef AUTOTUNE_SAFE_PROFILE
    AutotuneSafe_SetSafetyInput(
        (Safety_Manager_GetAction() == SAFETY_ACTION_DRIVE) &&
        (Safety_Manager_GetFault() == BSP_BXCAN_FAULT_NONE),
        (Safety_Manager_IsEstopActive() == 0U) ? 1U : 0U);
#endif
    Chassis_ControlApplySafetyAction();
    BSP_BXCAN_Process(now_ms);
    while (BSP_BXCAN_GetCommand(&command) != 0U) {
        g_target_command = command;
        g_has_target_command = 1U;
        Safety_Manager_AcceptCommand(&g_target_command);
        Chassis_ControlApplySafetyAction();
    }

#ifdef AUTOTUNE_SAFE_PROFILE
    if (AutotuneSafe_LocalExperiment_IsEngaged() != 0U) {
        AutotuneSafe_LocalExperiment_Process(now_ms);
        return;
    }
#endif

    if (g_scheduler_started == 0U) {
        g_last_control_ms = now_ms;
        g_scheduler_started = 1U;
        return;
    }

    if ((uint32_t)(now_ms - g_last_control_ms) >= CHASSIS_CONTROL_PERIOD_MS) {
        uint32_t dt_ms = now_ms - g_last_control_ms;
        g_last_control_ms = now_ms;
        Safety_Manager_ReportControlTiming(dt_ms);
#ifdef AUTOTUNE_SAFE_PROFILE
        if ((Wheel_Calibration_Service_IsActive() == 0U) &&
            (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE)) {
            Chassis_ControlAutotunePreflight(now_ms, dt_ms);
        }
#endif
        Chassis_ControlApplyClosedLoop(now_ms, dt_ms);
    }
}

void Chassis_Control_GetTelemetry(Chassis_ControlTelemetry_t *out)
{
    if (out != 0) {
        *out = g_telemetry;
    }
}
