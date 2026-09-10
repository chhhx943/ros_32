#include "bsp_bxcan.h"
#include "wheel_calibration_service.h"
#include "pid_tuning.h"
#include "autotune_safe.h"

#define BXCAN_DLC_V1 8U
#define BXCAN_STANDARD_FRAME 0U
#define BXCAN_DATA_FRAME 0U

typedef struct {
    uint8_t present;
    uint8_t range_ok;
    uint16_t command_seq;
    uint8_t mode_flags;
    uint32_t received_time_ms;
    int16_t steering_mrad;
} BXCAN_PendingSteering_t;

typedef struct {
    uint8_t present;
    uint8_t range_ok;
    uint16_t command_seq;
    uint8_t mode_flags;
    uint32_t received_time_ms;
    int16_t left_velocity_mmps;
    int16_t right_velocity_mmps;
} BXCAN_PendingWheels_t;

static BXCAN_PendingSteering_t g_pending_steering;
static BXCAN_PendingWheels_t g_pending_wheels;
static BSP_BXCAN_Command_t g_current_command;
static BSP_BXCAN_Feedback_t g_feedback;
static uint8_t g_command_available;
static uint8_t g_command_group_accepted;
static uint8_t g_steering_command_accepted;
static uint8_t g_estop_active;
static uint8_t g_safe_stop_active;
static uint8_t g_fault_latched;
static uint16_t g_fault_code;
static uint16_t g_applied_command_seq;
static uint8_t g_applied_command_seq_valid;
static uint16_t g_feedback_seq;
static uint8_t g_heartbeat_counter;
static uint32_t g_last_feedback_ms;
static uint8_t g_tx_active;
static uint8_t g_tx_index;
static uint16_t g_tx_feedback_seq;
static uint8_t g_tx_heartbeat_counter;
static uint16_t g_tx_applied_command_seq;
static BSP_BXCAN_Feedback_t g_tx_feedback;
static BSP_BXCAN_Diagnostics_t g_diagnostics;
static BSP_BXCAN_Diagnostics_t g_tx_diagnostics;
static uint32_t g_tx_device_time_ms;
#ifdef AUTOTUNE_SAFE_PROFILE
static uint8_t g_autotune_tx_active;
static uint8_t g_autotune_tx_index;
static uint8_t g_autotune_tx_snapshot_seq;
static uint32_t g_autotune_last_feedback_ms;
#endif
#ifndef BSP_BXCAN_HOST_TEST
static uint32_t g_last_hal_can_error;
#endif

static uint16_t BXCAN_ReadU16LE(const uint8_t *data);
static int16_t BXCAN_ReadI16LE(const uint8_t *data);
static void BXCAN_WriteU16LE(uint8_t *data, uint16_t value);
static void BXCAN_WriteI16LE(uint8_t *data, int16_t value);
static void BXCAN_WriteI32LE(uint8_t *data, int32_t value);
static uint8_t BXCAN_AbsI16InRange(int16_t value, int16_t limit);
static uint8_t BXCAN_FrameHeaderValid(uint8_t dlc, uint8_t ide, uint8_t rtr, const uint8_t data[8]);
static uint8_t BXCAN_ModeCanApply(uint8_t mode_flags);
static uint8_t BXCAN_CommandSequenceIsFresh(uint16_t candidate);
static uint8_t BXCAN_TimeElapsed(uint32_t now_ms, uint32_t then_ms, uint32_t limit_ms);
static uint16_t BXCAN_SaturatingIncrement(uint16_t value);
static void BXCAN_UpdateCommandAge(uint32_t now_ms);
#ifndef BSP_BXCAN_HOST_TEST
static void BXCAN_UpdateHalCanDiagnostics(void);
#endif
static void BXCAN_ClearPending(void);
static void BXCAN_ClearRecoverableFault(void);
static void BXCAN_SetProtocolFault(uint16_t fault_code, uint8_t latched);
static void BXCAN_AcceptCommand(uint16_t command_seq,
                                uint8_t mode_flags,
                                int16_t steering_mrad,
                                int16_t left_velocity_mmps,
                                int16_t right_velocity_mmps,
                                uint32_t now_ms);
static void BXCAN_ApplyEstop(uint16_t command_seq, uint8_t mode_flags, uint32_t now_ms);
static void BXCAN_TryCompleteCommandGroup(void);
static uint8_t BXCAN_BuildFeedbackFrame(uint16_t std_id,
                                        uint16_t feedback_seq,
                                        uint8_t heartbeat_counter,
                                        uint16_t applied_command_seq,
                                        uint16_t fault_code,
                                        uint8_t fault_latched,
                                        uint8_t estop_active,
                                        uint8_t safe_stop_active,
                                        const BSP_BXCAN_Feedback_t *feedback,
                                        const BSP_BXCAN_Diagnostics_t *diagnostics,
                                        uint32_t device_time_ms,
                                        uint8_t data[8]);
#if !defined(BSP_BXCAN_ENABLE_TEST_HOOKS)
static void BSP_BXCAN_ResetForTest(void);
#endif

static const uint16_t g_feedback_ids[8] = {
    BSP_BXCAN_ID_FB_STATUS,
    BSP_BXCAN_ID_FB_HEALTH,
    BSP_BXCAN_ID_FB_REAR_VELOCITY,
    BSP_BXCAN_ID_FB_REAR_LEFT_POSITION,
    BSP_BXCAN_ID_FB_REAR_RIGHT_POSITION,
    BSP_BXCAN_ID_FB_DIAGNOSTICS,
    PID_TUNING_ID_FB_GAINS,
    BSP_BXCAN_ID_FB_CONTROL_OUTPUT,
};

#ifdef AUTOTUNE_SAFE_PROFILE
static const uint16_t g_autotune_feedback_ids[11] = {
    AUTOTUNE_SAFE_ID_FB_IDENTITY,
    AUTOTUNE_SAFE_ID_FB_STATE,
    AUTOTUNE_SAFE_ID_FB_LIMITS,
    AUTOTUNE_SAFE_ID_FB_WHEEL,
    AUTOTUNE_SAFE_ID_FB_WHEEL,
    AUTOTUNE_SAFE_ID_FB_PID_PI,
    AUTOTUNE_SAFE_ID_FB_PID_DO,
    AUTOTUNE_SAFE_ID_FB_PID_PI,
    AUTOTUNE_SAFE_ID_FB_PID_DO,
    AUTOTUNE_SAFE_ID_FB_SAFETY,
    AUTOTUNE_SAFE_ID_FB_COUNTERS,
};
#endif

#ifndef BSP_BXCAN_HOST_TEST
void CAN1_Filter_Config(void)
{
    uint32_t fmr;

    /* CAN1/CAN2 share the filter block on STM32F407.  Configure bank 0
       directly while the filter block is in initialization mode.  This
       avoids the HAL filter-state path that can leave FINIT asserted on this
       target and prevent the application RX path from becoming usable. */
    __HAL_RCC_CAN2_CLK_ENABLE();
    fmr = CAN1->FMR;
    fmr &= ~CAN_FMR_CAN2SB;
    fmr |= (14U << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT;
    CAN1->FMR = fmr;

    CAN1->FA1R &= ~CAN_FA1R_FACT0;
    CAN1->FS1R |= CAN_FS1R_FSC0;
    CAN1->FM1R &= ~CAN_FM1R_FBM0;
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;
    /* Temporary A/B test: accept every standard/extended frame.  Once RX0
       is confirmed, restore the protocol mask for 0x120..0x127. */
    CAN1->sFilterRegister[0].FR1 = 0x00000000U;
    CAN1->sFilterRegister[0].FR2 = 0x00000000U;
    CAN1->FA1R |= CAN_FA1R_FACT0;
    CAN1->FMR = fmr & ~CAN_FMR_FINIT;
    __DSB();
}

void BSP_BXCAN_Init(void)
{
    BSP_BXCAN_ResetForTest();
    PID_Tuning_Init();
    Wheel_Calibration_Service_Init();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    /* CAN1/CAN2 share the acceptance-filter block on STM32F407.  Keep CAN2's
       clock enabled and apply the filter after CAN1 has entered Normal mode;
       this prevents the shared block from remaining in FINIT with bank 0
       inactive on boards where the CAN peripheral was reset during startup. */
    __HAL_RCC_CAN2_CLK_ENABLE();
    CAN1_Filter_Config();

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}
#else
void CAN1_Filter_Config(void)
{
}

void BSP_BXCAN_Init(void)
{
    BSP_BXCAN_ResetForTest();
}
#endif

void BSP_BXCAN_Process(uint32_t now_ms)
{
#ifndef BSP_BXCAN_HOST_TEST
    /* The CAN1/CAN2 filter block is shared on STM32F407.  Recover the
       acceptance path if a peripheral reset or clock transition leaves the
       shared bank in filter-init mode or deactivates bank 0. */
    if (((RCC->APB1ENR & RCC_APB1ENR_CAN2EN) == 0U) ||
        ((CAN1->FMR & CAN_FMR_FINIT) != 0U) ||
        ((CAN1->FA1R & CAN_FA1R_FACT0) == 0U)) {
        __HAL_RCC_CAN2_CLK_ENABLE();
        CAN1_Filter_Config();
    }
#endif
    BXCAN_UpdateCommandAge(now_ms);
#ifndef BSP_BXCAN_HOST_TEST
    BXCAN_UpdateHalCanDiagnostics();
#endif
    if (g_pending_steering.present &&
        ((uint32_t)(now_ms - g_pending_steering.received_time_ms) > BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS)) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCOMPLETE, 0U);
        BXCAN_ClearPending();
    }

    if (g_pending_wheels.present &&
        ((uint32_t)(now_ms - g_pending_wheels.received_time_ms) > BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS)) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCOMPLETE, 0U);
        BXCAN_ClearPending();
    }

    if (g_command_group_accepted &&
        BXCAN_TimeElapsed(now_ms, g_current_command.accepted_time_ms, BSP_BXCAN_COMMAND_TIMEOUT_MS)) {
        g_current_command.rear_left_velocity_mmps = 0;
        g_current_command.rear_right_velocity_mmps = 0;
        g_command_available = 1U;
        g_command_group_accepted = 0U;
        g_safe_stop_active = 1U;
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_COMMAND_TIMEOUT, 0U);
        BXCAN_UpdateCommandAge(now_ms);
    }

#ifndef BSP_BXCAN_HOST_TEST
    if (!g_tx_active && BXCAN_TimeElapsed(now_ms, g_last_feedback_ms, BSP_BXCAN_FEEDBACK_PERIOD_MS)) {
        uint32_t primask = __get_PRIMASK();

        /* Freeze all correlated fields as one immutable snapshot. CAN RX and
           error callbacks can update diagnostics concurrently with the main
           control context, so do not allow a torn sequence/state pair. */
        __disable_irq();
        g_feedback_seq++;
        g_heartbeat_counter++;
        g_tx_feedback_seq = g_feedback_seq;
        g_tx_heartbeat_counter = g_heartbeat_counter;
        g_tx_applied_command_seq = g_applied_command_seq;
        g_tx_feedback = g_feedback;
        g_tx_diagnostics = g_diagnostics;
        g_tx_device_time_ms = now_ms;
        g_tx_index = 0U;
        g_tx_active = 1U;
        g_last_feedback_ms = now_ms;
        if (primask == 0U) {
            __enable_irq();
        }
    }

    while (g_tx_active && (g_tx_index < 8U) && (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)) {
        CAN_TxHeaderTypeDef tx_header = {0};
        uint8_t tx_data[8] = {0};
        uint32_t mailbox = 0U;

        tx_header.StdId = g_feedback_ids[g_tx_index];
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = BXCAN_DLC_V1;
        tx_header.TransmitGlobalTime = DISABLE;

        if (tx_header.StdId == PID_TUNING_ID_FB_GAINS) {
            PID_Tuning_BuildFeedbackFrame(tx_data);
        } else {
            (void)BXCAN_BuildFeedbackFrame(tx_header.StdId,
                                           g_tx_feedback_seq,
                                           g_tx_heartbeat_counter,
                                           g_tx_applied_command_seq,
                                           g_fault_code,
                                           g_fault_latched,
                                           g_estop_active,
                                           g_safe_stop_active,
                                           &g_tx_feedback,
                                           &g_tx_diagnostics,
                                           g_tx_device_time_ms,
                                           tx_data);
        }

        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) != HAL_OK) {
            BSP_BXCAN_ReportTxError();
            break;
        }

        g_tx_index++;
        if (g_tx_index >= 8U) {
            g_tx_active = 0U;
        }
    }

#ifdef AUTOTUNE_SAFE_PROFILE
    if (!g_tx_active && !g_autotune_tx_active &&
        BXCAN_TimeElapsed(now_ms, g_autotune_last_feedback_ms,
                          BSP_BXCAN_FEEDBACK_PERIOD_MS)) {
        g_autotune_tx_snapshot_seq++;
        g_autotune_tx_index = 0U;
        g_autotune_tx_active = 1U;
        g_autotune_last_feedback_ms = now_ms;
    }

    while (g_autotune_tx_active && (g_autotune_tx_index < 11U) &&
           (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)) {
        CAN_TxHeaderTypeDef tx_header = {0};
        uint8_t tx_data[8] = {0};
        uint32_t mailbox = 0U;
        uint8_t axis = 0U;

        tx_header.StdId = g_autotune_feedback_ids[g_autotune_tx_index];
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = BXCAN_DLC_V1;
        tx_header.TransmitGlobalTime = DISABLE;
        if ((g_autotune_tx_index == 3U) || (g_autotune_tx_index == 5U) ||
            (g_autotune_tx_index == 6U)) {
            axis = 1U;
        } else if ((g_autotune_tx_index == 4U) ||
                   (g_autotune_tx_index == 7U) ||
                   (g_autotune_tx_index == 8U)) {
            axis = 2U;
        }
        if (AutotuneSafe_BuildTelemetryFrame(tx_header.StdId,
                                              g_autotune_tx_snapshot_seq,
                                              axis, tx_data) == 0U) {
            g_autotune_tx_index = 11U;
            g_autotune_tx_active = 0U;
            break;
        }
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) != HAL_OK) {
            BSP_BXCAN_ReportTxError();
            break;
        }
        g_autotune_tx_index++;
        if (g_autotune_tx_index >= 11U) {
            g_autotune_tx_active = 0U;
        }
    }
#endif

    if (!g_tx_active && Wheel_Calibration_Service_ShouldPublish(now_ms) &&
        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)) {
        CAN_TxHeaderTypeDef tx_header = {0};
        uint8_t tx_data[8] = {0};
        uint32_t mailbox = 0U;
        Wheel_Calibration_ServiceFeedback_t calibration_feedback;

        tx_header.StdId = BSP_BXCAN_ID_FB_CALIBRATION;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = BXCAN_DLC_V1;
        tx_header.TransmitGlobalTime = DISABLE;

        if (Wheel_Calibration_Service_GetFeedback(&calibration_feedback,
                                                  Wheel_Calibration_Service_HasResponsePending()) != 0U) {
            Wheel_Calibration_Service_EncodeFeedback(&calibration_feedback, tx_data);
            if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) == HAL_OK) {
                Wheel_Calibration_Service_MarkPublished(now_ms);
            }
        }
    }
#else
    (void)now_ms;
#endif
}

#ifdef AUTOTUNE_SAFE_PROFILE
void BSP_BXCAN_SetLocalVelocityCommand(int16_t left_velocity_mmps,
                                       int16_t right_velocity_mmps,
                                       uint32_t now_ms)
{
    uint16_t sequence = g_applied_command_seq_valid != 0U
                            ? (uint16_t)(g_applied_command_seq + 1U)
                            : 1U;

    BXCAN_AcceptCommand(sequence,
                        ((left_velocity_mmps == 0) && (right_velocity_mmps == 0))
                            ? BSP_BXCAN_MODE_STOP : BSP_BXCAN_MODE_VELOCITY,
                        0, left_velocity_mmps, right_velocity_mmps, now_ms);
}
#endif

void BSP_BXCAN_OnRxFrame(uint16_t std_id,
                         uint8_t dlc,
                         uint8_t ide,
                         uint8_t rtr,
                         const uint8_t data[8],
                         uint32_t now_ms)
{
    uint16_t command_seq;
    uint8_t mode_flags;

#ifdef AUTOTUNE_SAFE_PROFILE
    if (std_id == AUTOTUNE_SAFE_ID_CMD_CONTROL) {
        AutotuneSafe_OnCanFrame(dlc, ide, rtr, data, now_ms);
        return;
    }
#endif

#ifndef BSP_BXCAN_HOST_TEST
    if ((std_id == PID_TUNING_ID_CMD_GAINS) || (std_id == PID_TUNING_ID_CMD_D)) {
        uint8_t command_is_stop = ((g_current_command.mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_STOP);
        uint8_t command_is_zero = (g_current_command.equivalent_steering_mrad == 0) &&
                                  (g_current_command.rear_left_velocity_mmps == 0) &&
                                  (g_current_command.rear_right_velocity_mmps == 0);

        PID_Tuning_OnFrame(std_id, dlc, ide, rtr, data, now_ms,
                           g_command_group_accepted, command_is_stop,
                           command_is_zero, g_estop_active,
                           (g_fault_code != BSP_BXCAN_FAULT_NONE) ? 1U : 0U);
        return;
    }
#endif

    if (std_id == BSP_BXCAN_ID_CMD_CALIBRATION) {
        Wheel_Calibration_ServiceRequest_t request;

        if (Wheel_Calibration_Service_DecodeRequest(dlc, ide, rtr, data, &request) == 0U) {
            BSP_BXCAN_ReportRxError();
            BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCONSISTENT, 0U);
            return;
        }

        Wheel_Calibration_Service_OnRequest(&request,
                                            now_ms,
                                            &g_current_command,
                                            g_command_group_accepted,
                                            g_estop_active,
                                            (g_fault_code != BSP_BXCAN_FAULT_NONE) ? 1U : 0U);
        return;
    }

    if ((std_id != BSP_BXCAN_ID_CMD_STEERING) && (std_id != BSP_BXCAN_ID_CMD_REAR_WHEELS)) {
        return;
    }

    if (!BXCAN_FrameHeaderValid(dlc, ide, rtr, data)) {
        BSP_BXCAN_ReportRxError();
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCONSISTENT, 0U);
        return;
    }

    command_seq = BXCAN_ReadU16LE(&data[1]);
    mode_flags = data[3];

    if ((mode_flags & BSP_BXCAN_FLAG_ESTOP) != 0U) {
        BXCAN_ApplyEstop(command_seq, mode_flags, now_ms);
        return;
    }

    if (std_id == BSP_BXCAN_ID_CMD_STEERING) {
        if (BXCAN_ReadU16LE(&data[6]) != 0U) {
            BSP_BXCAN_ReportRxError();
            BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCONSISTENT, 0U);
            return;
        }

        g_pending_steering.present = 1U;
        g_pending_steering.command_seq = command_seq;
        g_pending_steering.mode_flags = mode_flags;
        g_pending_steering.received_time_ms = now_ms;
        g_pending_steering.steering_mrad = BXCAN_ReadI16LE(&data[4]);
        g_pending_steering.range_ok = BXCAN_AbsI16InRange(g_pending_steering.steering_mrad,
                                                          BSP_BXCAN_MAX_STEERING_MRAD);
    } else {
        g_pending_wheels.present = 1U;
        g_pending_wheels.command_seq = command_seq;
        g_pending_wheels.mode_flags = mode_flags;
        g_pending_wheels.received_time_ms = now_ms;
        g_pending_wheels.left_velocity_mmps = BXCAN_ReadI16LE(&data[4]);
        g_pending_wheels.right_velocity_mmps = BXCAN_ReadI16LE(&data[6]);
        g_pending_wheels.range_ok =
            BXCAN_AbsI16InRange(g_pending_wheels.left_velocity_mmps, BSP_BXCAN_MAX_REAR_VELOCITY_MMPS) &&
            BXCAN_AbsI16InRange(g_pending_wheels.right_velocity_mmps, BSP_BXCAN_MAX_REAR_VELOCITY_MMPS);
    }

    BXCAN_TryCompleteCommandGroup();
}

uint8_t BSP_BXCAN_GetCommand(BSP_BXCAN_Command_t *out_command)
{
    if (out_command == 0) {
        return 0U;
    }

    if (g_command_available == 0U) {
        return 0U;
    }

    *out_command = g_current_command;
    g_command_available = 0U;
    return 1U;
}

uint8_t BSP_BXCAN_GetCurrentCommand(BSP_BXCAN_Command_t *out_command)
{
    if (out_command == 0) {
        return 0U;
    }
    *out_command = g_current_command;
    return 1U;
}

uint8_t BSP_BXCAN_IsCommandFresh(void)
{
    return g_command_group_accepted;
}

void BSP_BXCAN_SetFeedback(const BSP_BXCAN_Feedback_t *feedback)
{
    if (feedback == 0) {
        return;
    }

    g_feedback = *feedback;
}

void BSP_BXCAN_SetDiagnostics(const BSP_BXCAN_Diagnostics_t *diagnostics)
{
    uint16_t rx_error_count;
    uint16_t tx_error_count;
    uint8_t can_error_class;

    if (diagnostics == 0) {
        return;
    }

    rx_error_count = g_diagnostics.rx_error_count;
    tx_error_count = g_diagnostics.tx_error_count;
    can_error_class = g_diagnostics.can_error_class;
    g_diagnostics = *diagnostics;
    g_diagnostics.rx_error_count = rx_error_count;
    g_diagnostics.tx_error_count = tx_error_count;
    g_diagnostics.can_error_class = can_error_class;
    if (can_error_class != BSP_BXCAN_CAN_ERROR_NONE) {
        g_diagnostics.flags |= BSP_BXCAN_DIAG_CAN_ERROR;
    }
}

void BSP_BXCAN_GetDiagnostics(BSP_BXCAN_Diagnostics_t *out_diagnostics)
{
    if (out_diagnostics != 0) {
        *out_diagnostics = g_diagnostics;
    }
}

uint16_t BSP_BXCAN_GetCommandAgeMs(uint32_t now_ms)
{
    if (g_command_group_accepted == 0U) {
        return 0xFFFFU;
    }
    if ((uint32_t)(now_ms - g_current_command.accepted_time_ms) > 0xFFFFU) {
        return 0xFFFFU;
    }
    return (uint16_t)(now_ms - g_current_command.accepted_time_ms);
}

void BSP_BXCAN_ReportRxError(void)
{
    g_diagnostics.rx_error_count = BXCAN_SaturatingIncrement(g_diagnostics.rx_error_count);
}

void BSP_BXCAN_ReportTxError(void)
{
    g_diagnostics.tx_error_count = BXCAN_SaturatingIncrement(g_diagnostics.tx_error_count);
    BSP_BXCAN_ReportCanError(BSP_BXCAN_CAN_ERROR_TX_FAILURE);
}

void BSP_BXCAN_ReportCanError(uint8_t error_class)
{
    if (error_class == BSP_BXCAN_CAN_ERROR_NONE) {
        return;
    }
    g_diagnostics.can_error_class = error_class;
    g_diagnostics.flags |= BSP_BXCAN_DIAG_CAN_ERROR;
}

void BSP_BXCAN_SetFault(uint16_t fault_code, uint8_t latched)
{
    BXCAN_SetProtocolFault(fault_code, latched);
}

uint16_t BSP_BXCAN_GetFault(void)
{
    return g_fault_code;
}

uint16_t BSP_BXCAN_GetAppliedCommandSeq(void)
{
    return g_applied_command_seq;
}

#if defined(BSP_BXCAN_ENABLE_TEST_HOOKS)
void BSP_BXCAN_ResetForTest(void)
#else
static void BSP_BXCAN_ResetForTest(void)
#endif
{
    g_pending_steering.present = 0U;
    g_pending_wheels.present = 0U;
    g_current_command.command_seq = 0U;
    g_current_command.mode_flags = BSP_BXCAN_MODE_STOP;
    g_current_command.equivalent_steering_mrad = 0;
    g_current_command.rear_left_velocity_mmps = 0;
    g_current_command.rear_right_velocity_mmps = 0;
    g_current_command.accepted_time_ms = 0U;
    g_feedback.status_flags = 0U;
    g_feedback.rear_left_velocity_mmps = 0;
    g_feedback.rear_right_velocity_mmps = 0;
    g_feedback.rear_left_position_mrad = 0;
    g_feedback.rear_right_position_mrad = 0;
    g_feedback.velocity_flags = 0U;
    g_feedback.rear_left_position_valid = 0U;
    g_feedback.rear_right_position_valid = 0U;
    g_feedback.left_control_output = 0;
    g_feedback.right_control_output = 0;
    g_diagnostics.flags = 0U;
    g_diagnostics.safety_state = 0U;
    g_diagnostics.safety_action = 0U;
    g_diagnostics.command_age_ms = 0xFFFFU;
    g_diagnostics.rx_error_count = 0U;
    g_diagnostics.tx_error_count = 0U;
    g_diagnostics.can_error_class = BSP_BXCAN_CAN_ERROR_NONE;
    g_command_available = 0U;
    g_command_group_accepted = 0U;
    g_steering_command_accepted = 0U;
    g_estop_active = 0U;
    g_safe_stop_active = 0U;
    g_fault_latched = 0U;
    g_fault_code = BSP_BXCAN_FAULT_NONE;
    g_applied_command_seq = 0U;
    g_applied_command_seq_valid = 0U;
    g_feedback_seq = 0U;
    g_heartbeat_counter = 0U;
    g_last_feedback_ms = 0U;
    g_tx_active = 0U;
    g_tx_index = 0U;
    g_tx_feedback_seq = 0U;
    g_tx_heartbeat_counter = 0U;
    g_tx_applied_command_seq = 0U;
    g_tx_feedback = g_feedback;
    g_tx_diagnostics = g_diagnostics;
    g_tx_device_time_ms = 0U;
#ifdef AUTOTUNE_SAFE_PROFILE
    g_autotune_tx_active = 0U;
    g_autotune_tx_index = 0U;
    g_autotune_tx_snapshot_seq = 0U;
    g_autotune_last_feedback_ms = 0U;
#endif
#ifndef BSP_BXCAN_HOST_TEST
    g_last_hal_can_error = HAL_CAN_ERROR_NONE;
#endif
    Wheel_Calibration_Service_Init();
}

#if defined(BSP_BXCAN_ENABLE_TEST_HOOKS)
void BSP_BXCAN_TestBuildFrame(uint16_t std_id,
                              uint16_t feedback_seq,
                              uint8_t heartbeat_counter,
                              uint16_t applied_command_seq,
                              const BSP_BXCAN_Feedback_t *feedback,
                              uint32_t device_time_ms,
                              uint8_t data[8])
{
#ifndef BSP_BXCAN_HOST_TEST
    if (std_id == PID_TUNING_ID_FB_GAINS) {
        PID_Tuning_BuildFeedbackFrame(data);
        return;
    }
#endif
    (void)BXCAN_BuildFeedbackFrame(std_id,
                                   feedback_seq,
                                   heartbeat_counter,
                                   applied_command_seq,
                                   g_fault_code,
                                   g_fault_latched,
                                   g_estop_active,
                                   g_safe_stop_active,
                                   feedback,
                                   &g_diagnostics,
                                   device_time_ms,
                                   data);
}

void BSP_BXCAN_TestBuildCalibrationFrame(uint8_t data[8])
{
    Wheel_Calibration_ServiceFeedback_t feedback;

    if (Wheel_Calibration_Service_GetFeedback(&feedback, 1U) == 0U) {
        return;
    }
    Wheel_Calibration_Service_EncodeFeedback(&feedback, data);
}
#endif

#ifndef BSP_BXCAN_HOST_TEST
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if (hcan->Instance != CAN1) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }

        BSP_BXCAN_OnRxFrame((uint16_t)rx_header.StdId,
                            (uint8_t)rx_header.DLC,
                            (rx_header.IDE == CAN_ID_STD) ? 0U : 1U,
                            (rx_header.RTR == CAN_RTR_DATA) ? 0U : 1U,
                            rx_data,
                            HAL_GetTick());
    }
}
#endif

static uint16_t BXCAN_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t BXCAN_ReadI16LE(const uint8_t *data)
{
    return (int16_t)BXCAN_ReadU16LE(data);
}

static void BXCAN_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void BXCAN_WriteI16LE(uint8_t *data, int16_t value)
{
    BXCAN_WriteU16LE(data, (uint16_t)value);
}

static void BXCAN_WriteI32LE(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static uint8_t BXCAN_AbsI16InRange(int16_t value, int16_t limit)
{
    if (value > limit) {
        return 0U;
    }

    if (value < (int16_t)-limit) {
        return 0U;
    }

    return 1U;
}

static uint8_t BXCAN_FrameHeaderValid(uint8_t dlc, uint8_t ide, uint8_t rtr, const uint8_t data[8])
{
    if (data == 0) {
        return 0U;
    }

    if ((dlc != BXCAN_DLC_V1) || (ide != BXCAN_STANDARD_FRAME) || (rtr != BXCAN_DATA_FRAME)) {
        return 0U;
    }

    if (data[0] != BSP_BXCAN_PROTOCOL_VERSION) {
        return 0U;
    }

    if ((data[3] & BSP_BXCAN_FLAG_RESERVED_MASK) != 0U) {
        return 0U;
    }

    return 1U;
}

static uint8_t BXCAN_ModeCanApply(uint8_t mode_flags)
{
    uint8_t mode = mode_flags & BSP_BXCAN_MODE_MASK;

    if (mode == BSP_BXCAN_MODE_MAINTENANCE) {
        return 0U;
    }

    if (mode == BSP_BXCAN_MODE_RESERVED) {
        return 0U;
    }

    return 1U;
}

static uint8_t BXCAN_TimeElapsed(uint32_t now_ms, uint32_t then_ms, uint32_t limit_ms)
{
    return ((uint32_t)(now_ms - then_ms) >= limit_ms) ? 1U : 0U;
}

static void BXCAN_ClearPending(void)
{
    g_pending_steering.present = 0U;
    g_pending_wheels.present = 0U;
}

static void BXCAN_ClearRecoverableFault(void)
{
    if (g_fault_latched == 0U) {
        g_fault_code = BSP_BXCAN_FAULT_NONE;
    }
}

static void BXCAN_SetProtocolFault(uint16_t fault_code, uint8_t latched)
{
    if ((g_fault_latched != 0U) && (latched == 0U) && (fault_code != BSP_BXCAN_FAULT_NONE)) {
        return;
    }

    g_fault_code = fault_code;
    g_fault_latched = (latched != 0U) ? 1U : 0U;
}

static void BXCAN_AcceptCommand(uint16_t command_seq,
                                uint8_t mode_flags,
                                int16_t steering_mrad,
                                int16_t left_velocity_mmps,
                                int16_t right_velocity_mmps,
                                uint32_t now_ms)
{
    uint8_t mode = mode_flags & BSP_BXCAN_MODE_MASK;

    g_current_command.command_seq = command_seq;
    g_current_command.mode_flags = mode_flags;
    g_current_command.equivalent_steering_mrad = steering_mrad;
    g_current_command.accepted_time_ms = now_ms;

    g_estop_active = 0U;
    g_safe_stop_active = ((mode_flags & BSP_BXCAN_FLAG_SAFE_STOP) != 0U) ? 1U : 0U;

    if ((mode == BSP_BXCAN_MODE_STOP) || (g_safe_stop_active != 0U)) {
        g_current_command.rear_left_velocity_mmps = 0;
        g_current_command.rear_right_velocity_mmps = 0;
    } else {
        g_current_command.rear_left_velocity_mmps = left_velocity_mmps;
        g_current_command.rear_right_velocity_mmps = right_velocity_mmps;
    }

    if (((mode_flags & BSP_BXCAN_FLAG_RESET_FAULT) != 0U) &&
        (mode == BSP_BXCAN_MODE_STOP) &&
        (steering_mrad == 0) &&
        (left_velocity_mmps == 0) &&
        (right_velocity_mmps == 0) &&
        (g_fault_code == BSP_BXCAN_FAULT_MCU_WATCHDOG_RESET)) {
        g_fault_code = BSP_BXCAN_FAULT_NONE;
        g_fault_latched = 0U;
    }

    BXCAN_ClearRecoverableFault();
    g_command_available = 1U;
    g_command_group_accepted = 1U;
    g_steering_command_accepted = 1U;
    g_applied_command_seq = command_seq;
    g_applied_command_seq_valid = 1U;
}

static void BXCAN_ApplyEstop(uint16_t command_seq, uint8_t mode_flags, uint32_t now_ms)
{
    g_current_command.command_seq = command_seq;
    g_current_command.mode_flags = mode_flags;
    g_current_command.equivalent_steering_mrad = 0;
    g_current_command.rear_left_velocity_mmps = 0;
    g_current_command.rear_right_velocity_mmps = 0;
    g_current_command.accepted_time_ms = now_ms;
    g_command_available = 1U;
    g_command_group_accepted = 0U;
    g_steering_command_accepted = 0U;
    g_estop_active = 1U;
    g_safe_stop_active = 0U;
    /* Protocol §6: a single-frame E-stop must not update applied_command_seq
     * because no complete command group was committed. */
    BXCAN_ClearPending();
    BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_ESTOP_ACTIVE, 1U);
}

static void BXCAN_TryCompleteCommandGroup(void)
{
    uint32_t first_time;
    uint32_t second_time;
    uint8_t mode_flags;

    if ((g_pending_steering.present == 0U) || (g_pending_wheels.present == 0U)) {
        return;
    }

    if ((g_pending_steering.command_seq != g_pending_wheels.command_seq) ||
        (g_pending_steering.mode_flags != g_pending_wheels.mode_flags)) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCONSISTENT, 0U);
        BXCAN_ClearPending();
        return;
    }

    first_time = g_pending_steering.received_time_ms;
    second_time = g_pending_wheels.received_time_ms;
    if ((uint32_t)(first_time - second_time) > BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS &&
        (uint32_t)(second_time - first_time) > BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCOMPLETE, 0U);
        BXCAN_ClearPending();
        return;
    }

    if ((g_pending_steering.range_ok == 0U) || (g_pending_wheels.range_ok == 0U)) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_RANGE_INVALID, 0U);
        BXCAN_ClearPending();
        return;
    }

    mode_flags = g_pending_steering.mode_flags;
    if (BXCAN_ModeCanApply(mode_flags) == 0U) {
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_GROUP_INCONSISTENT, 0U);
        BXCAN_ClearPending();
        return;
    }

    /* A complete but duplicate/out-of-order group is not fresh. */
    if (BXCAN_CommandSequenceIsFresh(g_pending_steering.command_seq) == 0U) {
        /* Duplicate/out-of-order traffic is stale, not a malformed frame. */
        BXCAN_ClearPending();
        return;
    }

    BXCAN_AcceptCommand(g_pending_steering.command_seq,
                        mode_flags,
                        g_pending_steering.steering_mrad,
                        g_pending_wheels.left_velocity_mmps,
                        g_pending_wheels.right_velocity_mmps,
                        (g_pending_steering.received_time_ms > g_pending_wheels.received_time_ms)
                            ? g_pending_steering.received_time_ms
                            : g_pending_wheels.received_time_ms);
    BXCAN_ClearPending();
}

static uint8_t BXCAN_CommandSequenceIsFresh(uint16_t candidate)
{
    uint16_t distance;

    if (g_applied_command_seq_valid == 0U) {
        return 1U;
    }
    distance = (uint16_t)(candidate - g_applied_command_seq);
    return (distance != 0U) && (distance < 0x8000U);
}

static uint8_t BXCAN_BuildFeedbackFrame(uint16_t std_id,
                                        uint16_t feedback_seq,
                                        uint8_t heartbeat_counter,
                                        uint16_t applied_command_seq,
                                        uint16_t fault_code,
                                        uint8_t fault_latched,
                                        uint8_t estop_active,
                                        uint8_t safe_stop_active,
                                        const BSP_BXCAN_Feedback_t *feedback,
                                        const BSP_BXCAN_Diagnostics_t *diagnostics,
                                        uint32_t device_time_ms,
                                        uint8_t data[8])
{
    uint8_t status_flags;

    if ((feedback == 0) || (diagnostics == 0) || (data == 0)) {
        return 0U;
    }

    data[0] = BSP_BXCAN_PROTOCOL_VERSION;
    BXCAN_WriteU16LE(&data[1], feedback_seq);

    switch (std_id) {
    case BSP_BXCAN_ID_FB_STATUS:
        data[3] = heartbeat_counter;
        BXCAN_WriteU16LE(&data[4], applied_command_seq);
        BXCAN_WriteU16LE(&data[6], fault_code);
        return 1U;

    case BSP_BXCAN_ID_FB_HEALTH:
        status_flags = feedback->status_flags;
        status_flags &= (uint8_t)~BSP_BXCAN_STATUS_STEERING_FB_AVAILABLE;
        if (g_command_group_accepted != 0U) {
            status_flags |= BSP_BXCAN_STATUS_COMMAND_ACCEPTED;
        }
        if (g_steering_command_accepted != 0U) {
            status_flags |= BSP_BXCAN_STATUS_STEERING_ACCEPTED;
        }
        if (estop_active != 0U) {
            status_flags |= BSP_BXCAN_STATUS_ESTOP_ACTIVE;
        }
        if (safe_stop_active != 0U) {
            status_flags |= BSP_BXCAN_STATUS_SAFE_STOP_ACTIVE;
        }
        if (fault_latched != 0U) {
            status_flags |= BSP_BXCAN_STATUS_FAULT_LATCHED;
        }
        data[3] = status_flags;
        data[4] = (uint8_t)(device_time_ms & 0xFFU);
        data[5] = (uint8_t)((device_time_ms >> 8) & 0xFFU);
        data[6] = (uint8_t)((device_time_ms >> 16) & 0xFFU);
        data[7] = (uint8_t)((device_time_ms >> 24) & 0xFFU);
        return 1U;

    case BSP_BXCAN_ID_FB_REAR_VELOCITY:
        data[3] = feedback->velocity_flags & (BSP_BXCAN_VELOCITY_LEFT_VALID | BSP_BXCAN_VELOCITY_RIGHT_VALID);
        BXCAN_WriteI16LE(&data[4], feedback->rear_left_velocity_mmps);
        BXCAN_WriteI16LE(&data[6], feedback->rear_right_velocity_mmps);
        return 1U;

    case BSP_BXCAN_ID_FB_REAR_LEFT_POSITION:
        data[3] = (feedback->rear_left_position_valid != 0U) ? 1U : 0U;
        BXCAN_WriteI32LE(&data[4], feedback->rear_left_position_mrad);
        return 1U;

    case BSP_BXCAN_ID_FB_REAR_RIGHT_POSITION:
        data[3] = (feedback->rear_right_position_valid != 0U) ? 1U : 0U;
        BXCAN_WriteI32LE(&data[4], feedback->rear_right_position_mrad);
        return 1U;

    case BSP_BXCAN_ID_FB_CONTROL_OUTPUT:
        data[3] = 0x03U;
        BXCAN_WriteI16LE(&data[4], feedback->left_control_output);
        BXCAN_WriteI16LE(&data[6], feedback->right_control_output);
        return 1U;

    case BSP_BXCAN_ID_FB_DIAGNOSTICS:
        data[3] = diagnostics->flags;
        data[4] = diagnostics->safety_state;
        data[5] = diagnostics->safety_action;
        data[6] = (diagnostics->command_age_ms == 0xFFFFU)
                      ? 0xFFU
                      : (uint8_t)(diagnostics->command_age_ms / 10U);
        data[7] = diagnostics->can_error_class;
        return 1U;

    default:
        return 0U;
    }
}

static uint16_t BXCAN_SaturatingIncrement(uint16_t value)
{
    return (value == 0xFFFFU) ? value : (uint16_t)(value + 1U);
}

static void BXCAN_UpdateCommandAge(uint32_t now_ms)
{
    g_diagnostics.command_age_ms = BSP_BXCAN_GetCommandAgeMs(now_ms);
    if (g_diagnostics.command_age_ms == 0xFFFFU) {
        g_diagnostics.flags &= (uint8_t)~BSP_BXCAN_DIAG_COMMAND_FRESH;
    } else {
        g_diagnostics.flags |= BSP_BXCAN_DIAG_COMMAND_FRESH;
    }
}

#ifndef BSP_BXCAN_HOST_TEST
static void BXCAN_UpdateHalCanDiagnostics(void)
{
    uint32_t error_code = HAL_CAN_GetError(&hcan1);

    if (error_code == g_last_hal_can_error) {
        return;
    }
    g_last_hal_can_error = error_code;

    if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
        BSP_BXCAN_ReportCanError(BSP_BXCAN_CAN_ERROR_BUS_OFF);
        /* A bus-off controller cannot receive the fresh STOP required by the
           watchdog. Force the same zero-output SAFE_STOP path immediately. */
        g_current_command.rear_left_velocity_mmps = 0;
        g_current_command.rear_right_velocity_mmps = 0;
        g_command_available = 1U;
        g_command_group_accepted = 0U;
        g_safe_stop_active = 1U;
        BXCAN_SetProtocolFault(BSP_BXCAN_FAULT_COMMAND_TIMEOUT, 0U);
    } else if ((error_code & HAL_CAN_ERROR_EPV) != 0U) {
        BSP_BXCAN_ReportCanError(BSP_BXCAN_CAN_ERROR_PASSIVE);
    } else if (error_code != HAL_CAN_ERROR_NONE) {
        BSP_BXCAN_ReportCanError(BSP_BXCAN_CAN_ERROR_WARNING);
    }
}
#endif
