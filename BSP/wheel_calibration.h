#ifndef __WHEEL_CALIBRATION_H
#define __WHEEL_CALIBRATION_H

#if defined(WHEEL_CALIBRATION_HOST_TEST) || defined(ENCODER_HOST_TEST)
#include <stdint.h>
#else
#include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t valid;
    int8_t left_encoder_polarity;
    int8_t right_encoder_polarity;
    uint16_t left_min_start_pwm_permille;
    uint16_t right_min_start_pwm_permille;
    uint16_t version;
    uint16_t reserved;
} CalibrationData_t;

typedef struct {
    uint32_t encoder_ppr;
    uint32_t quadrature_factor;
    uint32_t gear_ratio_x1000;
    uint32_t wheel_radius_mm_x1000;
} WheelCalibrationParameters_t;

void Wheel_Calibration_Init(void);
uint8_t Wheel_Calibration_IsValid(void);
uint8_t Wheel_Calibration_IsDataValid(const CalibrationData_t *data);
const CalibrationData_t *Wheel_Calibration_GetActive(void);
uint8_t Wheel_Calibration_Install(const CalibrationData_t *data);
uint8_t Wheel_Calibration_SetPending(const CalibrationData_t *data);
uint8_t Wheel_Calibration_CommitPending(void);
void Wheel_Calibration_DiscardPending(void);
void Wheel_Calibration_Invalidate(void);
const WheelCalibrationParameters_t *Wheel_Calibration_GetParameters(void);
void Wheel_Calibration_SetParameters(const WheelCalibrationParameters_t *parameters);

#ifdef __cplusplus
}
#endif

#endif
