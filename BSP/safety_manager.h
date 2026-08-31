#ifndef __SAFETY_MANAGER_H
#define __SAFETY_MANAGER_H

#ifdef SAFETY_MANAGER_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#include "bsp_bxcan.h"

#define SAFETY_MANAGER_NOMINAL_CONTROL_PERIOD_MS       10U
#define SAFETY_MANAGER_ESTOP_RELEASE_HOLD_MS           50U
#define SAFETY_MANAGER_CONTROL_MAX_DT_MS              100U
#define SAFETY_MANAGER_ENCODER_INVALID_SAMPLES_MAX      5U
#define SAFETY_MANAGER_ENCODER_INVALID_MAX_MS         100U
#define SAFETY_MANAGER_ENCODER_VALID_RECOVERY_MS      100U
#define SAFETY_MANAGER_DIRECTION_TARGET_MIN_MMPS      300
#define SAFETY_MANAGER_DIRECTION_OPPOSITE_MIN_MMPS     30
#define SAFETY_MANAGER_DIRECTION_DEBOUNCE_MS          100U
#define SAFETY_MANAGER_STALL_ARM_DELAY_MS            1000U
#define SAFETY_MANAGER_STALL_TARGET_MIN_MMPS          300
#define SAFETY_MANAGER_STALL_PWM_MIN                  700
#define SAFETY_MANAGER_STALL_SPEED_MAX_MMPS            10
#define SAFETY_MANAGER_STALL_DEBOUNCE_MS              500U

typedef enum {
    SAFETY_STATE_BOOT = 0,
    SAFETY_STATE_CALIBRATION_REQUIRED,
    SAFETY_STATE_CALIBRATION,
    SAFETY_STATE_STANDBY,
    SAFETY_STATE_DRIVE,
    SAFETY_STATE_SAFE_STOP,
    SAFETY_STATE_ESTOP,
    SAFETY_STATE_FAULT
} Safety_State_t;

typedef enum {
    SAFETY_ACTION_COAST = 0,
    SAFETY_ACTION_DRIVE,
    SAFETY_ACTION_CALIBRATION,
    SAFETY_ACTION_BRAKE
} Safety_Action_t;

#ifdef __cplusplus
extern "C" {
#endif

void Safety_Manager_Init(void);
void Safety_Manager_Process(uint32_t now_ms);
void Safety_Manager_AcceptCommand(const BSP_BXCAN_Command_t *command);
void Safety_Manager_ReportFault(uint16_t fault_code);
void Safety_Manager_ReportControlTiming(uint32_t dt_ms);
void Safety_Manager_ReportEncoderSample(uint8_t wheel,
                                        uint8_t trusted,
                                        int32_t velocity_mmps,
                                        uint32_t dt_ms);
void Safety_Manager_ReportDriveObservation(uint8_t wheel,
                                           int16_t target_mmps,
                                           int16_t measured_mmps,
                                           int16_t pwm,
                                           uint32_t now_ms);
void Safety_Manager_NotifyWatchdogReset(void);
Safety_State_t Safety_Manager_GetState(void);
Safety_Action_t Safety_Manager_GetAction(void);
uint8_t Safety_Manager_DriveAllowed(void);
uint16_t Safety_Manager_GetFault(void);
uint8_t Safety_Manager_IsEstopActive(void);
uint8_t Safety_Manager_IsSafeStopActive(void);
uint8_t Safety_Manager_IsCalibrationRequired(void);

#ifdef __cplusplus
}
#endif

#endif
