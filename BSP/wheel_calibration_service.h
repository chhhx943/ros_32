#ifndef __WHEEL_CALIBRATION_SERVICE_H
#define __WHEEL_CALIBRATION_SERVICE_H

#ifdef WHEEL_CALIBRATION_SERVICE_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#include "bsp_bxcan.h"
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_CALIBRATION_SERVICE_COOKIE              0xC35AU
#define WHEEL_CALIBRATION_SERVICE_PERIOD_MS           20U
#define WHEEL_CALIBRATION_OPCODE_START               0x01U
#define WHEEL_CALIBRATION_OPCODE_CANCEL              0x02U
#define WHEEL_CALIBRATION_OPCODE_QUERY               0x03U
#define WHEEL_CALIBRATION_OPTION_MAINTENANCE_CONFIRM 0x01U
#define WHEEL_CALIBRATION_OPTION_WHEELS_LIFTED_CONFIRM 0x02U
#define WHEEL_CALIBRATION_OPTION_MASK                0x03U
#define WHEEL_CALIBRATION_PWM_INITIAL_PERMILLE       50U
#define WHEEL_CALIBRATION_PWM_STEP_PERMILLE          25U
#define WHEEL_CALIBRATION_PWM_STEP_HOLD_MS           200U
#define WHEEL_CALIBRATION_MIN_RESPONSE_COUNTS        560
#define WHEEL_CALIBRATION_START_RESPONSE_COUNTS      280
#define WHEEL_CALIBRATION_PWM_MAX_PERMILLE            250
#define WHEEL_CALIBRATION_STOP_HOLD_MS               300U
#define WHEEL_CALIBRATION_STILL_SPEED_MAX_MMPS       10
#define WHEEL_CALIBRATION_STAGE_MS                   1500U
#define WHEEL_CALIBRATION_FORWARD_TIMEOUT_MS         2000U
#define WHEEL_CALIBRATION_SETTLE_HOLD_MS             300U
#define WHEEL_CALIBRATION_SETTLE_TIMEOUT_MS          1000U
#define WHEEL_CALIBRATION_PRECHECK_TIMEOUT_MS        5000U
#define WHEEL_CALIBRATION_TOTAL_TIMEOUT_MS           20000U

#define WHEEL_CALIBRATION_RESULT_TERMINAL            0x01U
#define WHEEL_CALIBRATION_RESULT_SUCCESS             0x02U
#define WHEEL_CALIBRATION_RESULT_RESPONSE_TO_REQUEST 0x04U

typedef enum {
    WHEEL_CALIBRATION_TX_NONE = 0,
    WHEEL_CALIBRATION_TX_PRECHECK = 1,
    WHEEL_CALIBRATION_TX_RUNNING = 2,
    WHEEL_CALIBRATION_TX_SUCCEEDED = 3,
    WHEEL_CALIBRATION_TX_CANCELED = 4,
    WHEEL_CALIBRATION_TX_ABORTED = 5,
    WHEEL_CALIBRATION_TX_FAILED_VALIDATION = 6,
    WHEEL_CALIBRATION_TX_REJECTED = 7
} Wheel_Calibration_TransactionState_t;

typedef enum {
    WHEEL_CALIBRATION_STAGE_NONE = 0,
    WHEEL_CALIBRATION_STAGE_PRECHECK = 1,
    WHEEL_CALIBRATION_STAGE_LEFT_FORWARD = 2,
    WHEEL_CALIBRATION_STAGE_LEFT_SETTLE = 3,
    WHEEL_CALIBRATION_STAGE_LEFT_REVERSE = 4,
    WHEEL_CALIBRATION_STAGE_RIGHT_FORWARD = 5,
    WHEEL_CALIBRATION_STAGE_RIGHT_SETTLE = 6,
    WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE = 7,
    WHEEL_CALIBRATION_STAGE_VALIDATE = 8
} Wheel_Calibration_Stage_t;

typedef enum {
    WHEEL_CALIBRATION_EXIT_NONE = 0x00U,
    WHEEL_CALIBRATION_EXIT_SUCCESS = 0x01U,
    WHEEL_CALIBRATION_EXIT_OPERATOR_CANCEL = 0x02U,
    WHEEL_CALIBRATION_EXIT_SAFETY_ESTOP = 0x03U,
    WHEEL_CALIBRATION_EXIT_SAFETY_FAULT = 0x04U,
    WHEEL_CALIBRATION_EXIT_COMMUNICATION_TIMEOUT = 0x05U,
    WHEEL_CALIBRATION_EXIT_ENCODER_INVALID = 0x06U,
    WHEEL_CALIBRATION_EXIT_PRECHECK_TIMEOUT = 0x07U,
    WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION = 0x08U,
    WHEEL_CALIBRATION_EXIT_FORWARD_DIRECTION_MISMATCH = 0x09U,
    WHEEL_CALIBRATION_EXIT_SETTLE_TIMEOUT = 0x0BU,
    WHEEL_CALIBRATION_EXIT_REVERSE_INSUFFICIENT_MOTION = 0x0CU,
    WHEEL_CALIBRATION_EXIT_REVERSE_DIRECTION_MISMATCH = 0x0DU,
    WHEEL_CALIBRATION_EXIT_REVERSE_TIMEOUT = 0x0EU,
    WHEEL_CALIBRATION_EXIT_LEFT_RIGHT_INCONSISTENT = 0x0FU,
    WHEEL_CALIBRATION_EXIT_TOTAL_TIMEOUT = 0x10U,
    WHEEL_CALIBRATION_EXIT_REJECTED_BUSY = 0x11U,
    WHEEL_CALIBRATION_EXIT_REJECTED_STALE_SEQUENCE = 0x12U,
    WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT = 0x13U,
    WHEEL_CALIBRATION_EXIT_REJECTED_NOT_PERMITTED = 0x14U
} Wheel_Calibration_ExitReason_t;

typedef struct {
    uint16_t service_seq;
    uint8_t opcode;
    uint8_t options;
    uint16_t service_cookie;
    uint8_t reserved;
} Wheel_Calibration_ServiceRequest_t;

typedef struct {
    uint16_t service_seq;
    uint8_t transaction_state;
    uint8_t calibration_stage;
    uint8_t exit_reason;
    uint8_t result_flags;
    uint8_t progress_percent;
} Wheel_Calibration_ServiceFeedback_t;

typedef struct {
    int16_t left_pwm;
    int16_t right_pwm;
} Wheel_Calibration_Recommendation_t;

typedef struct {
    Wheel_Calibration_Stage_t stage;
    int64_t left_start_counts;
    int64_t left_delta_counts;
    uint16_t stage_pwm_permille;
    uint8_t left_sample_trusted;
    uint8_t response_detected;
} Wheel_Calibration_StageDiagnostics_t;

typedef struct {
    uint8_t valid;
    Wheel_Calibration_TransactionState_t terminal_state;
    Wheel_Calibration_Stage_t stage;
    Wheel_Calibration_ExitReason_t exit_reason;
    uint32_t timestamp_ms;
    EncoderSample_t left_sample;
    int64_t stage_delta_counts;
    uint16_t stage_pwm_permille;
    uint8_t response_detected;
} Wheel_Calibration_TerminalSample_t;

uint8_t Wheel_Calibration_Service_DecodeRequest(uint8_t dlc,
                                                uint8_t ide,
                                                uint8_t rtr,
                                                const uint8_t data[8],
                                                Wheel_Calibration_ServiceRequest_t *request);
void Wheel_Calibration_Service_EncodeFeedback(const Wheel_Calibration_ServiceFeedback_t *feedback,
                                              uint8_t data[8]);
void Wheel_Calibration_Service_Init(void);
void Wheel_Calibration_Service_OnRequest(const Wheel_Calibration_ServiceRequest_t *request,
                                         uint32_t now_ms,
                                         const BSP_BXCAN_Command_t *command,
                                         uint8_t command_fresh,
                                         uint8_t estop_active,
                                         uint8_t fault_active);
void Wheel_Calibration_Service_Process(uint32_t now_ms,
                                       const BSP_BXCAN_Command_t *command,
                                       uint8_t command_fresh,
                                       const EncoderSample_t *left_sample,
                                       const EncoderSample_t *right_sample,
                                       uint8_t estop_active,
                                       uint8_t fault_active);
uint8_t Wheel_Calibration_Service_GetRecommendation(Wheel_Calibration_Recommendation_t *recommendation);
void Wheel_Calibration_Service_GetStageDiagnostics(Wheel_Calibration_StageDiagnostics_t *diagnostics);
void Wheel_Calibration_Service_GetTerminalSample(Wheel_Calibration_TerminalSample_t *sample);
Wheel_Calibration_TransactionState_t Wheel_Calibration_Service_GetState(void);
Wheel_Calibration_Stage_t Wheel_Calibration_Service_GetStage(void);
Wheel_Calibration_ExitReason_t Wheel_Calibration_Service_GetExitReason(void);
uint8_t Wheel_Calibration_Service_GetFeedback(Wheel_Calibration_ServiceFeedback_t *feedback,
                                              uint8_t response_to_request);
uint8_t Wheel_Calibration_Service_ShouldPublish(uint32_t now_ms);
void Wheel_Calibration_Service_MarkPublished(uint32_t now_ms);
uint8_t Wheel_Calibration_Service_HasResponsePending(void);
uint8_t Wheel_Calibration_Service_IsActive(void);
uint8_t Wheel_Calibration_Service_ClearFailedValidation(void);

#ifdef __cplusplus
}
#endif

#endif
