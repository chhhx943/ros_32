#ifndef __BSP_BXCAN_H
#define __BSP_BXCAN_H

#ifdef BSP_BXCAN_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#include "can.h"
#endif

#include "pid_tuning.h"

#define BSP_BXCAN_ID_CMD_PID_GAINS             PID_TUNING_ID_CMD_GAINS
#define BSP_BXCAN_ID_CMD_PID_D                 PID_TUNING_ID_CMD_D
#define BSP_BXCAN_ID_FB_PID_GAINS              PID_TUNING_ID_FB_GAINS
#define BSP_BXCAN_ID_FB_CONTROL_OUTPUT         0x188U

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BXCAN_PROTOCOL_VERSION             1U

#define BSP_BXCAN_ID_CMD_STEERING              0x120U
#define BSP_BXCAN_ID_CMD_REAR_WHEELS           0x121U
#define BSP_BXCAN_ID_CMD_CALIBRATION           0x122U
#define BSP_BXCAN_ID_FB_STATUS                 0x180U
#define BSP_BXCAN_ID_FB_HEALTH                 0x181U
#define BSP_BXCAN_ID_FB_REAR_VELOCITY          0x182U
#define BSP_BXCAN_ID_FB_REAR_LEFT_POSITION     0x183U
#define BSP_BXCAN_ID_FB_REAR_RIGHT_POSITION    0x184U
#define BSP_BXCAN_ID_FB_CALIBRATION             0x185U
#define BSP_BXCAN_ID_FB_DIAGNOSTICS             0x186U

#define BSP_BXCAN_MODE_STOP                    0x00U
#define BSP_BXCAN_MODE_VELOCITY                0x01U
#define BSP_BXCAN_MODE_MAINTENANCE             0x02U
#define BSP_BXCAN_MODE_RESERVED                0x03U
#define BSP_BXCAN_MODE_MASK                    0x03U
#define BSP_BXCAN_FLAG_ESTOP                   0x04U
#define BSP_BXCAN_FLAG_SAFE_STOP               0x08U
#define BSP_BXCAN_FLAG_RESET_FAULT             0x10U
#define BSP_BXCAN_FLAG_RESERVED_MASK           0xE0U

#define BSP_BXCAN_STATUS_COMMAND_ACCEPTED      0x01U
#define BSP_BXCAN_STATUS_LEFT_ENCODER_OK       0x02U
#define BSP_BXCAN_STATUS_RIGHT_ENCODER_OK      0x04U
#define BSP_BXCAN_STATUS_STEERING_ACCEPTED     0x08U
#define BSP_BXCAN_STATUS_STEERING_FB_AVAILABLE 0x10U
#define BSP_BXCAN_STATUS_ESTOP_ACTIVE          0x20U
#define BSP_BXCAN_STATUS_SAFE_STOP_ACTIVE      0x40U
#define BSP_BXCAN_STATUS_FAULT_LATCHED         0x80U

#define BSP_BXCAN_VELOCITY_LEFT_VALID          0x01U
#define BSP_BXCAN_VELOCITY_RIGHT_VALID         0x02U

#define BSP_BXCAN_DIAG_CALIBRATION_REQUIRED    0x01U
#define BSP_BXCAN_DIAG_DRIVE_ALLOWED           0x02U
#define BSP_BXCAN_DIAG_COMMAND_FRESH           0x04U
#define BSP_BXCAN_DIAG_LEFT_ENCODER_INVALID    0x08U
#define BSP_BXCAN_DIAG_RIGHT_ENCODER_INVALID   0x10U
#define BSP_BXCAN_DIAG_CONTROL_OVERRUN         0x20U
#define BSP_BXCAN_DIAG_MOTOR_STALL             0x40U
#define BSP_BXCAN_DIAG_CAN_ERROR               0x80U

#define BSP_BXCAN_CAN_ERROR_NONE               0x00U
#define BSP_BXCAN_CAN_ERROR_WARNING            0x01U
#define BSP_BXCAN_CAN_ERROR_PASSIVE            0x02U
#define BSP_BXCAN_CAN_ERROR_BUS_OFF            0x03U
#define BSP_BXCAN_CAN_ERROR_TX_FAILURE         0x04U

#define BSP_BXCAN_FAULT_NONE                   0x0000U
#define BSP_BXCAN_FAULT_ESTOP_ACTIVE           0x0001U
#define BSP_BXCAN_FAULT_REAR_ENCODER           0x0002U
#define BSP_BXCAN_FAULT_STEERING_REJECTED      0x0003U
#define BSP_BXCAN_FAULT_COMMAND_TIMEOUT        0x0004U
#define BSP_BXCAN_FAULT_GROUP_INCOMPLETE       0x0005U
#define BSP_BXCAN_FAULT_GROUP_INCONSISTENT     0x0006U
#define BSP_BXCAN_FAULT_RANGE_INVALID          0x0007U
#define BSP_BXCAN_FAULT_MCU_WATCHDOG_RESET     0x0008U
#define BSP_BXCAN_FAULT_MOTOR_DRIVER           0x0009U
#define BSP_BXCAN_FAULT_CALIBRATION_INVALID    0x000AU
#define BSP_BXCAN_FAULT_MOTOR_STALL           0x000BU
#define BSP_BXCAN_FAULT_CONTROL_OVERRUN       0x000CU
#define BSP_BXCAN_FAULT_UNKNOWN_DEVICE         0xFFFFU

#ifndef BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS
#define BSP_BXCAN_COMPLETE_COMMAND_WINDOW_MS   10U
#endif

#ifndef BSP_BXCAN_COMMAND_TIMEOUT_MS
#define BSP_BXCAN_COMMAND_TIMEOUT_MS           100U
#endif

#ifndef BSP_BXCAN_FEEDBACK_PERIOD_MS
#define BSP_BXCAN_FEEDBACK_PERIOD_MS           20U
#endif

#ifndef BSP_BXCAN_MAX_STEERING_MRAD
#define BSP_BXCAN_MAX_STEERING_MRAD            1000
#endif

#ifndef BSP_BXCAN_MAX_REAR_VELOCITY_MMPS
#define BSP_BXCAN_MAX_REAR_VELOCITY_MMPS       3000
#endif

typedef struct {
    uint16_t command_seq;
    uint8_t mode_flags;
    int16_t equivalent_steering_mrad;
    int16_t rear_left_velocity_mmps;
    int16_t rear_right_velocity_mmps;
    uint32_t accepted_time_ms;
} BSP_BXCAN_Command_t;

typedef struct {
    uint8_t status_flags;
    int16_t rear_left_velocity_mmps;
    int16_t rear_right_velocity_mmps;
    int32_t rear_left_position_mrad;
    int32_t rear_right_position_mrad;
    uint8_t velocity_flags;
    uint8_t rear_left_position_valid;
    uint8_t rear_right_position_valid;
    int16_t left_control_output;
    int16_t right_control_output;
} BSP_BXCAN_Feedback_t;

typedef struct {
    uint8_t flags;
    uint8_t safety_state;
    uint8_t safety_action;
    uint16_t command_age_ms;
    uint16_t rx_error_count;
    uint16_t tx_error_count;
    uint8_t can_error_class;
} BSP_BXCAN_Diagnostics_t;

void CAN1_Filter_Config(void);
void BSP_BXCAN_Init(void);
void BSP_BXCAN_Process(uint32_t now_ms);
void BSP_BXCAN_OnRxFrame(uint16_t std_id,
                         uint8_t dlc,
                         uint8_t ide,
                         uint8_t rtr,
                         const uint8_t data[8],
                         uint32_t now_ms);
uint8_t BSP_BXCAN_GetCommand(BSP_BXCAN_Command_t *out_command);
uint8_t BSP_BXCAN_GetCurrentCommand(BSP_BXCAN_Command_t *out_command);
uint8_t BSP_BXCAN_IsCommandFresh(void);
void BSP_BXCAN_SetFeedback(const BSP_BXCAN_Feedback_t *feedback);
void BSP_BXCAN_SetDiagnostics(const BSP_BXCAN_Diagnostics_t *diagnostics);
void BSP_BXCAN_GetDiagnostics(BSP_BXCAN_Diagnostics_t *out_diagnostics);
uint16_t BSP_BXCAN_GetCommandAgeMs(uint32_t now_ms);
void BSP_BXCAN_ReportRxError(void);
void BSP_BXCAN_ReportTxError(void);
void BSP_BXCAN_ReportCanError(uint8_t error_class);
void BSP_BXCAN_SetFault(uint16_t fault_code, uint8_t latched);
uint16_t BSP_BXCAN_GetFault(void);
uint16_t BSP_BXCAN_GetAppliedCommandSeq(void);

#if defined(BSP_BXCAN_ENABLE_TEST_HOOKS)
void BSP_BXCAN_ResetForTest(void);
void BSP_BXCAN_TestBuildFrame(uint16_t std_id,
                              uint16_t feedback_seq,
                              uint8_t heartbeat_counter,
                              uint16_t applied_command_seq,
                              const BSP_BXCAN_Feedback_t *feedback,
                              uint32_t device_time_ms,
                              uint8_t data[8]);
void BSP_BXCAN_TestBuildCalibrationFrame(uint8_t data[8]);
#endif

#ifdef __cplusplus
}
#endif

#endif
