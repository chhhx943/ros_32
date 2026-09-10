#include "wheel_calibration.h"

#if !defined(WHEEL_CALIBRATION_HOST_TEST) || defined(WHEEL_CALIBRATION_STORAGE_HOST_TEST)
#include "wheel_calibration_storage.h"
#define WHEEL_CALIBRATION_PERSISTENCE_ENABLED
#endif

static CalibrationData_t g_active_calibration;
static CalibrationData_t g_pending_calibration;
static uint8_t g_pending_valid;
static WheelCalibrationParameters_t g_parameters = {500U, 4U, 28000U, 33250U};

uint8_t Wheel_Calibration_IsDataValid(const CalibrationData_t *data)
{
    if (data == 0) {
        return 0U;
    }
    if (data->valid == 0U) {
        return 0U;
    }
    if (((data->left_encoder_polarity != 1) &&
         (data->left_encoder_polarity != -1)) ||
        ((data->right_encoder_polarity != 1) &&
         (data->right_encoder_polarity != -1))) {
        return 0U;
    }
    if ((data->left_min_start_pwm_permille == 0U) ||
        (data->left_min_start_pwm_permille > 1000U) ||
        (data->right_min_start_pwm_permille == 0U) ||
        (data->right_min_start_pwm_permille > 1000U) ||
        (data->version == 0U)) {
        return 0U;
    }
    return 1U;
}

void Wheel_Calibration_Init(void)
{
    g_active_calibration = (CalibrationData_t){0};
    g_pending_calibration = (CalibrationData_t){0};
    g_pending_valid = 0U;
    g_parameters = (WheelCalibrationParameters_t){500U, 4U, 28000U, 33250U};

#ifdef CALIBRATION_BENCH_DEFAULTS
    g_active_calibration.valid = 1U;
    g_active_calibration.left_encoder_polarity = 1;
    g_active_calibration.right_encoder_polarity = -1;
    g_active_calibration.left_min_start_pwm_permille = 100U;
    g_active_calibration.right_min_start_pwm_permille = 100U;
    g_active_calibration.version = 1U;
#endif

#ifdef WHEEL_CALIBRATION_PERSISTENCE_ENABLED
    {
        CalibrationData_t stored_calibration;
        if (Wheel_Calibration_Storage_Load(&stored_calibration) != 0U) {
            g_active_calibration = stored_calibration;
        }
    }
#endif
}

const WheelCalibrationParameters_t *Wheel_Calibration_GetParameters(void)
{
    return &g_parameters;
}

void Wheel_Calibration_SetParameters(const WheelCalibrationParameters_t *parameters)
{
    if ((parameters != 0) && (parameters->encoder_ppr != 0U) &&
        (parameters->quadrature_factor != 0U) &&
        (parameters->gear_ratio_x1000 != 0U) &&
        (parameters->wheel_radius_mm_x1000 != 0U)) {
        g_parameters = *parameters;
    }
}

uint8_t Wheel_Calibration_IsValid(void)
{
    return Wheel_Calibration_IsDataValid(&g_active_calibration);
}

const CalibrationData_t *Wheel_Calibration_GetActive(void)
{
    return &g_active_calibration;
}

uint8_t Wheel_Calibration_Install(const CalibrationData_t *data)
{
    if (Wheel_Calibration_IsDataValid(data) == 0U) {
        return 0U;
    }

    g_active_calibration = *data;
    return 1U;
}

uint8_t Wheel_Calibration_SetPending(const CalibrationData_t *data)
{
    if (Wheel_Calibration_IsDataValid(data) == 0U) {
        return 0U;
    }
    g_pending_calibration = *data;
    g_pending_valid = 1U;
    return 1U;
}

uint8_t Wheel_Calibration_CommitPending(void)
{
    if (g_pending_valid == 0U) {
        return 0U;
    }
#ifdef WHEEL_CALIBRATION_PERSISTENCE_ENABLED
    if (Wheel_Calibration_Storage_Save(&g_pending_calibration) == 0U) {
        return 0U;
    }
#endif
    g_active_calibration = g_pending_calibration;
    g_pending_valid = 0U;
    return 1U;
}

void Wheel_Calibration_DiscardPending(void)
{
    g_pending_calibration = (CalibrationData_t){0};
    g_pending_valid = 0U;
}

void Wheel_Calibration_Invalidate(void)
{
    g_active_calibration.valid = 0U;
}
