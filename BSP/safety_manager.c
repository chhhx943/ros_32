#include "safety_manager.h"
#include "physical_estop.h"
#include "wheel_calibration.h"
#include "wheel_calibration_service.h"

static Safety_State_t g_safety_state;
static Safety_Action_t g_safety_action;
static uint8_t g_estop_latched;
static uint8_t g_has_command;
static BSP_BXCAN_Command_t g_last_command;
static uint16_t g_latched_fault_code;
static uint8_t g_encoder_invalid_samples[2];
static uint32_t g_encoder_invalid_ms[2];
static uint32_t g_encoder_valid_ms[2];
static uint8_t g_encoder_recovery_ready[2];
static uint8_t g_encoder_fault_wheel[2];
static uint8_t g_control_overrun_evidence;
static uint8_t g_watchdog_reset_pending;
static uint8_t g_estop_release_tracking;
static uint32_t g_estop_release_start_ms;
static uint8_t g_estop_release_ready;
static uint8_t g_drive_observation_valid[2];
static uint32_t g_drive_start_ms[2];
static int8_t g_target_direction[2];
static uint32_t g_direction_bad_start_ms[2];
static uint8_t g_direction_bad_tracking[2];
static uint32_t g_stall_bad_start_ms[2];
static uint8_t g_stall_bad_tracking[2];

static uint8_t Safety_Manager_TimeElapsed(uint32_t now_ms,
                                          uint32_t then_ms,
                                          uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - then_ms) >= duration_ms) ? 1U : 0U;
}

static uint8_t Safety_Manager_IsZeroStop(const BSP_BXCAN_Command_t *command)
{
    return (command != 0) &&
           ((command->mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_STOP) &&
           (command->rear_left_velocity_mmps == 0) &&
           (command->rear_right_velocity_mmps == 0) &&
           (command->equivalent_steering_mrad == 0);
}

static uint8_t Safety_Manager_IsLatchedFault(uint16_t fault_code)
{
    return (fault_code == BSP_BXCAN_FAULT_ESTOP_ACTIVE) ||
           (fault_code == BSP_BXCAN_FAULT_REAR_ENCODER) ||
           (fault_code == BSP_BXCAN_FAULT_MCU_WATCHDOG_RESET) ||
           (fault_code == BSP_BXCAN_FAULT_MOTOR_DRIVER) ||
           (fault_code == BSP_BXCAN_FAULT_CALIBRATION_INVALID) ||
           (fault_code == BSP_BXCAN_FAULT_MOTOR_STALL) ||
           (fault_code == BSP_BXCAN_FAULT_CONTROL_OVERRUN) ||
           (fault_code == BSP_BXCAN_FAULT_UNKNOWN_DEVICE);
}

static uint8_t Safety_Manager_FaultPriority(uint16_t fault_code)
{
    if (fault_code == BSP_BXCAN_FAULT_ESTOP_ACTIVE) {
        return 5U;
    }
    if (Safety_Manager_IsLatchedFault(fault_code) != 0U) {
        return 4U;
    }
    if (fault_code == BSP_BXCAN_FAULT_COMMAND_TIMEOUT) {
        return 3U;
    }
    if (fault_code != BSP_BXCAN_FAULT_NONE) {
        return 2U;
    }
    return 0U;
}

static void Safety_ManagerSet(Safety_State_t state, Safety_Action_t action)
{
    g_safety_state = state;
    g_safety_action = action;
}

static void Safety_ManagerSetReadyState(void)
{
    if (Wheel_Calibration_IsValid() != 0U) {
        Safety_ManagerSet(SAFETY_STATE_STANDBY, SAFETY_ACTION_COAST);
    } else {
        Safety_ManagerSet(SAFETY_STATE_CALIBRATION_REQUIRED, SAFETY_ACTION_COAST);
    }
}

static Safety_Action_t Safety_ManagerFaultAction(uint16_t fault_code)
{
    return (fault_code == BSP_BXCAN_FAULT_CONTROL_OVERRUN)
               ? SAFETY_ACTION_BRAKE
               : SAFETY_ACTION_COAST;
}

static void Safety_ManagerApplyLatchedFault(void)
{
    if (g_latched_fault_code == BSP_BXCAN_FAULT_ESTOP_ACTIVE) {
        Safety_ManagerSet(SAFETY_STATE_ESTOP, SAFETY_ACTION_BRAKE);
    } else if (g_latched_fault_code != BSP_BXCAN_FAULT_NONE) {
        Safety_ManagerSet(SAFETY_STATE_FAULT,
                          Safety_ManagerFaultAction(g_latched_fault_code));
    }
}

static void Safety_ManagerLatchEstop(void)
{
    g_estop_latched = 1U;
    g_estop_release_tracking = 0U;
    g_estop_release_ready = 0U;
    Safety_Manager_ReportFault(BSP_BXCAN_FAULT_ESTOP_ACTIVE);
    Safety_ManagerApplyLatchedFault();
}

void Safety_Manager_ReportFault(uint16_t fault_code)
{
    uint8_t candidate_priority;
    uint8_t current_priority;

    if ((fault_code == BSP_BXCAN_FAULT_NONE) ||
        (Safety_Manager_IsLatchedFault(fault_code) == 0U)) {
        return;
    }

    candidate_priority = Safety_Manager_FaultPriority(fault_code);
    current_priority = Safety_Manager_FaultPriority(g_latched_fault_code);
    if ((g_latched_fault_code == BSP_BXCAN_FAULT_NONE) ||
        (candidate_priority > current_priority)) {
        g_latched_fault_code = fault_code;
        BSP_BXCAN_SetFault(g_latched_fault_code, 1U);
    }
    if (fault_code == BSP_BXCAN_FAULT_ESTOP_ACTIVE) {
        g_estop_latched = 1U;
    }
    Safety_ManagerApplyLatchedFault();
}

void Safety_Manager_ReportControlTiming(uint32_t dt_ms)
{
    if (dt_ms > SAFETY_MANAGER_NOMINAL_CONTROL_PERIOD_MS) {
        g_control_overrun_evidence = 1U;
    }
    if (dt_ms > SAFETY_MANAGER_CONTROL_MAX_DT_MS) {
        Safety_Manager_ReportFault(BSP_BXCAN_FAULT_CONTROL_OVERRUN);
    }
}

void Safety_Manager_ReportEncoderSample(uint8_t wheel,
                                        uint8_t trusted,
                                        int32_t velocity_mmps,
                                        uint32_t dt_ms)
{
    uint8_t index;

    if ((wheel < 1U) || (wheel > 2U)) {
        return;
    }
    index = (uint8_t)(wheel - 1U);
    if (trusted != 0U) {
        g_encoder_invalid_samples[index] = 0U;
        g_encoder_invalid_ms[index] = 0U;
        if (g_encoder_valid_ms[index] < SAFETY_MANAGER_ENCODER_VALID_RECOVERY_MS) {
            g_encoder_valid_ms[index] += dt_ms;
            if (g_encoder_valid_ms[index] >= SAFETY_MANAGER_ENCODER_VALID_RECOVERY_MS) {
                g_encoder_recovery_ready[index] = 1U;
            }
        }
        return;
    }

    (void)velocity_mmps;
    g_encoder_valid_ms[index] = 0U;
    g_encoder_recovery_ready[index] = 0U;
    if (g_encoder_invalid_samples[index] < 0xFFU) {
        g_encoder_invalid_samples[index]++;
    }
    g_encoder_invalid_ms[index] += dt_ms;
    if ((g_encoder_invalid_samples[index] >= SAFETY_MANAGER_ENCODER_INVALID_SAMPLES_MAX) ||
        (g_encoder_invalid_ms[index] >= SAFETY_MANAGER_ENCODER_INVALID_MAX_MS)) {
        g_encoder_fault_wheel[index] = 1U;
        Safety_Manager_ReportFault(BSP_BXCAN_FAULT_REAR_ENCODER);
    }
}

void Safety_Manager_ReportDriveObservation(uint8_t wheel,
                                           int16_t target_mmps,
                                           int16_t measured_mmps,
                                           int16_t pwm,
                                           uint32_t now_ms)
{
    uint8_t index;
    int8_t direction;
    int32_t target_abs;
    int32_t measured_abs;
    int32_t pwm_abs;

    if ((wheel < 1U) || (wheel > 2U)) {
        return;
    }
    index = (uint8_t)(wheel - 1U);
    target_abs = target_mmps;
    if (target_abs < 0) {
        target_abs = -target_abs;
    }
    measured_abs = measured_mmps;
    if (measured_abs < 0) {
        measured_abs = -measured_abs;
    }
    pwm_abs = pwm;
    if (pwm_abs < 0) {
        pwm_abs = -pwm_abs;
    }

    if (target_abs < SAFETY_MANAGER_DIRECTION_TARGET_MIN_MMPS) {
        g_drive_observation_valid[index] = 0U;
        g_direction_bad_tracking[index] = 0U;
        g_stall_bad_tracking[index] = 0U;
        return;
    }

    direction = (target_mmps > 0) ? 1 : -1;
    if ((g_drive_observation_valid[index] == 0U) ||
        (g_target_direction[index] != direction)) {
        g_drive_observation_valid[index] = 1U;
        g_drive_start_ms[index] = now_ms;
        g_target_direction[index] = direction;
        g_direction_bad_tracking[index] = 0U;
        g_stall_bad_tracking[index] = 0U;
    }

    if (((direction > 0) && (measured_mmps <= -SAFETY_MANAGER_DIRECTION_OPPOSITE_MIN_MMPS)) ||
        ((direction < 0) && (measured_mmps >= SAFETY_MANAGER_DIRECTION_OPPOSITE_MIN_MMPS))) {
        if (g_direction_bad_tracking[index] == 0U) {
            g_direction_bad_tracking[index] = 1U;
            g_direction_bad_start_ms[index] = now_ms;
        } else if (Safety_Manager_TimeElapsed(now_ms,
                                               g_direction_bad_start_ms[index],
                                               SAFETY_MANAGER_DIRECTION_DEBOUNCE_MS) != 0U) {
            Safety_Manager_ReportFault(BSP_BXCAN_FAULT_REAR_ENCODER);
            return;
        }
    } else {
        g_direction_bad_tracking[index] = 0U;
    }

    if ((Safety_Manager_TimeElapsed(now_ms,
                                    g_drive_start_ms[index],
                                    SAFETY_MANAGER_STALL_ARM_DELAY_MS) != 0U) &&
        (target_abs >= SAFETY_MANAGER_STALL_TARGET_MIN_MMPS) &&
        (pwm_abs >= SAFETY_MANAGER_STALL_PWM_MIN) &&
        (measured_abs <= SAFETY_MANAGER_STALL_SPEED_MAX_MMPS)) {
        if (g_stall_bad_tracking[index] == 0U) {
            g_stall_bad_tracking[index] = 1U;
            g_stall_bad_start_ms[index] = now_ms;
        } else if (Safety_Manager_TimeElapsed(now_ms,
                                               g_stall_bad_start_ms[index],
                                               SAFETY_MANAGER_STALL_DEBOUNCE_MS) != 0U) {
            Safety_Manager_ReportFault(BSP_BXCAN_FAULT_MOTOR_STALL);
        }
    } else {
        g_stall_bad_tracking[index] = 0U;
    }
}

void Safety_Manager_NotifyWatchdogReset(void)
{
    g_watchdog_reset_pending = 1U;
}

void Safety_Manager_Init(void)
{
    uint8_t index;

    Physical_EStop_Init();
    Wheel_Calibration_Init();
    g_safety_state = SAFETY_STATE_BOOT;
    g_safety_action = SAFETY_ACTION_COAST;
    g_estop_latched = 0U;
    g_has_command = 0U;
    g_last_command = (BSP_BXCAN_Command_t){0};
    g_latched_fault_code = BSP_BXCAN_FAULT_NONE;
    g_control_overrun_evidence = 0U;
    g_watchdog_reset_pending = 0U;
    g_estop_release_tracking = 0U;
    g_estop_release_start_ms = 0U;
    g_estop_release_ready = 0U;
    for (index = 0U; index < 2U; ++index) {
        g_encoder_invalid_samples[index] = 0U;
        g_encoder_invalid_ms[index] = 0U;
        g_encoder_valid_ms[index] = 0U;
        g_encoder_recovery_ready[index] = 0U;
        g_encoder_fault_wheel[index] = 0U;
        g_drive_observation_valid[index] = 0U;
        g_drive_start_ms[index] = 0U;
        g_target_direction[index] = 0;
        g_direction_bad_start_ms[index] = 0U;
        g_direction_bad_tracking[index] = 0U;
        g_stall_bad_start_ms[index] = 0U;
        g_stall_bad_tracking[index] = 0U;
    }
#ifndef SAFETY_MANAGER_HOST_TEST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        Safety_Manager_NotifyWatchdogReset();
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
#endif
}

void Safety_Manager_Process(uint32_t now_ms)
{
    uint8_t physical_estop_active;
    uint8_t physical_estop_event;
    uint16_t fault;

    Physical_EStop_Process();
    physical_estop_active = Physical_EStop_IsAsserted();
    physical_estop_event = Physical_EStop_ConsumeAssertEvent();
    if ((physical_estop_active != 0U) || (physical_estop_event != 0U)) {
        Safety_ManagerLatchEstop();
        if (physical_estop_active == 0U) {
            g_estop_release_tracking = 1U;
            g_estop_release_start_ms = now_ms;
        }
        return;
    }

    if ((g_estop_latched != 0U) && (g_estop_release_tracking == 0U)) {
        g_estop_release_tracking = 1U;
        g_estop_release_start_ms = now_ms;
    }
    if ((g_estop_latched != 0U) &&
        (Safety_Manager_TimeElapsed(now_ms,
                                    g_estop_release_start_ms,
                                    SAFETY_MANAGER_ESTOP_RELEASE_HOLD_MS) != 0U)) {
        g_estop_release_ready = 1U;
    }

    if (g_watchdog_reset_pending != 0U) {
        g_watchdog_reset_pending = 0U;
        Safety_Manager_ReportFault(BSP_BXCAN_FAULT_MCU_WATCHDOG_RESET);
    }

    fault = BSP_BXCAN_GetFault();
    if (fault == BSP_BXCAN_FAULT_ESTOP_ACTIVE) {
        if (g_estop_latched == 0U) {
            Safety_ManagerLatchEstop();
        } else {
            Safety_ManagerApplyLatchedFault();
        }
        return;
    }
    if (g_latched_fault_code != BSP_BXCAN_FAULT_NONE) {
        Safety_ManagerApplyLatchedFault();
        return;
    }
    if (fault == BSP_BXCAN_FAULT_COMMAND_TIMEOUT) {
        Safety_ManagerSet(SAFETY_STATE_SAFE_STOP, SAFETY_ACTION_COAST);
        return;
    }
    if ((fault != BSP_BXCAN_FAULT_NONE) &&
        (Safety_Manager_IsLatchedFault(fault) != 0U)) {
        Safety_Manager_ReportFault(fault);
        return;
    }
    if ((fault != BSP_BXCAN_FAULT_NONE) &&
        (g_safety_state != SAFETY_STATE_SAFE_STOP)) {
        Safety_ManagerSet(SAFETY_STATE_FAULT, SAFETY_ACTION_COAST);
        return;
    }
    if (Wheel_Calibration_Service_GetState() == WHEEL_CALIBRATION_TX_FAILED_VALIDATION) {
        Safety_Manager_ReportFault(BSP_BXCAN_FAULT_CALIBRATION_INVALID);
        return;
    }
    if (Wheel_Calibration_Service_IsActive() != 0U) {
        Safety_ManagerSet(SAFETY_STATE_CALIBRATION, SAFETY_ACTION_CALIBRATION);
        return;
    }

    if ((g_safety_state == SAFETY_STATE_BOOT) ||
        ((g_safety_state == SAFETY_STATE_CALIBRATION_REQUIRED) &&
         (Wheel_Calibration_IsValid() != 0U))) {
        Safety_ManagerSetReadyState();
    } else if ((Wheel_Calibration_IsValid() == 0U) &&
               ((g_safety_state == SAFETY_STATE_STANDBY) ||
                (g_safety_state == SAFETY_STATE_DRIVE))) {
        Safety_ManagerSet(SAFETY_STATE_CALIBRATION_REQUIRED, SAFETY_ACTION_COAST);
    }
}

static uint8_t Safety_ManagerResetCauseCleared(void)
{
    uint8_t index;

    if (g_latched_fault_code == BSP_BXCAN_FAULT_REAR_ENCODER) {
        for (index = 0U; index < 2U; ++index) {
            if ((g_encoder_fault_wheel[index] != 0U) &&
                (g_encoder_recovery_ready[index] == 0U)) {
                return 0U;
            }
        }
    }
    return 1U;
}

void Safety_Manager_AcceptCommand(const BSP_BXCAN_Command_t *command)
{
    uint8_t mode;
    uint16_t fault;

    if (command == 0) {
        return;
    }
    g_last_command = *command;
    g_has_command = 1U;
    mode = command->mode_flags & BSP_BXCAN_MODE_MASK;
    fault = BSP_BXCAN_GetFault();

    if ((command->mode_flags & BSP_BXCAN_FLAG_ESTOP) != 0U) {
        Safety_ManagerLatchEstop();
        return;
    }

    if ((g_estop_latched != 0U) ||
        (g_latched_fault_code == BSP_BXCAN_FAULT_ESTOP_ACTIVE) ||
        (fault == BSP_BXCAN_FAULT_ESTOP_ACTIVE)) {
        if ((command->mode_flags & BSP_BXCAN_FLAG_RESET_FAULT) != 0U &&
            Safety_Manager_IsZeroStop(command) &&
            (Physical_EStop_IsAsserted() == 0U) &&
            (g_estop_release_ready != 0U)) {
            g_estop_latched = 0U;
            g_latched_fault_code = BSP_BXCAN_FAULT_NONE;
            BSP_BXCAN_SetFault(BSP_BXCAN_FAULT_NONE, 0U);
            Safety_ManagerSetReadyState();
        } else {
            Safety_ManagerApplyLatchedFault();
        }
        return;
    }

    if (g_latched_fault_code != BSP_BXCAN_FAULT_NONE) {
        if ((command->mode_flags & BSP_BXCAN_FLAG_RESET_FAULT) != 0U &&
            Safety_Manager_IsZeroStop(command) &&
            (Safety_ManagerResetCauseCleared() != 0U)) {
            g_latched_fault_code = BSP_BXCAN_FAULT_NONE;
            BSP_BXCAN_SetFault(BSP_BXCAN_FAULT_NONE, 0U);
            Safety_ManagerSetReadyState();
        } else {
            Safety_ManagerApplyLatchedFault();
        }
        return;
    }

    if ((g_safety_state == SAFETY_STATE_SAFE_STOP) ||
        (fault == BSP_BXCAN_FAULT_COMMAND_TIMEOUT)) {
        if (Safety_Manager_IsZeroStop(command) != 0U) {
            BSP_BXCAN_SetFault(BSP_BXCAN_FAULT_NONE, 0U);
            Safety_ManagerSetReadyState();
        } else {
            Safety_ManagerSet(SAFETY_STATE_SAFE_STOP, SAFETY_ACTION_COAST);
        }
        return;
    }

    if ((fault != BSP_BXCAN_FAULT_NONE) ||
        ((command->mode_flags & BSP_BXCAN_FLAG_SAFE_STOP) != 0U)) {
        Safety_ManagerSet(SAFETY_STATE_FAULT, SAFETY_ACTION_COAST);
        return;
    }
    if (Wheel_Calibration_Service_IsActive() != 0U) {
        Safety_ManagerSet(SAFETY_STATE_CALIBRATION, SAFETY_ACTION_CALIBRATION);
        return;
    }
    if (Wheel_Calibration_IsValid() == 0U) {
        Safety_ManagerSet(SAFETY_STATE_CALIBRATION_REQUIRED, SAFETY_ACTION_COAST);
        return;
    }
    if ((mode == BSP_BXCAN_MODE_STOP) ||
        ((mode == BSP_BXCAN_MODE_VELOCITY) &&
         (command->rear_left_velocity_mmps == 0) &&
         (command->rear_right_velocity_mmps == 0))) {
        Safety_ManagerSet(SAFETY_STATE_STANDBY, SAFETY_ACTION_COAST);
        return;
    }
    if (mode == BSP_BXCAN_MODE_VELOCITY) {
        Safety_ManagerSet(SAFETY_STATE_DRIVE, SAFETY_ACTION_DRIVE);
        return;
    }
    Safety_ManagerSet(SAFETY_STATE_STANDBY, SAFETY_ACTION_COAST);
}

Safety_State_t Safety_Manager_GetState(void)
{
    return g_safety_state;
}

Safety_Action_t Safety_Manager_GetAction(void)
{
    return g_safety_action;
}

uint8_t Safety_Manager_DriveAllowed(void)
{
    return (g_safety_action == SAFETY_ACTION_DRIVE) ? 1U : 0U;
}

uint16_t Safety_Manager_GetFault(void)
{
    uint16_t selected_fault = BSP_BXCAN_GetFault();

    if (Safety_Manager_FaultPriority(g_latched_fault_code) >
        Safety_Manager_FaultPriority(selected_fault)) {
        selected_fault = g_latched_fault_code;
    }
    return selected_fault;
}

uint8_t Safety_Manager_IsEstopActive(void)
{
    return (g_safety_state == SAFETY_STATE_ESTOP) ? 1U : 0U;
}

uint8_t Safety_Manager_IsSafeStopActive(void)
{
    return (g_safety_state == SAFETY_STATE_SAFE_STOP) ? 1U : 0U;
}

uint8_t Safety_Manager_IsCalibrationRequired(void)
{
    return (g_safety_state == SAFETY_STATE_CALIBRATION_REQUIRED) ? 1U : 0U;
}
