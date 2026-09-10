#include "encoder.h"
#include "tim.h"
#include "wheel_calibration.h"

#define ENCODER_LEFT_MOTOR          1U
#define ENCODER_RIGHT_MOTOR         2U
#define ENCODER_LEFT_POLARITY       1
#define ENCODER_RIGHT_POLARITY     -1
#define ENCODER_PPR_PER_CHANNEL     500LL
#define ENCODER_QUADRATURE_FACTOR   4LL
#define ENCODER_GEAR_RATIO          28LL
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR_PER_CHANNEL * ENCODER_QUADRATURE_FACTOR * ENCODER_GEAR_RATIO)
#define ENCODER_WHEEL_RADIUS_MM_X100 3325LL
#define ENCODER_PI_X1000000         3141593LL
#define ENCODER_WHEEL_CIRCUM_MM_X1000 \
    ((2LL * ENCODER_WHEEL_RADIUS_MM_X100 * ENCODER_PI_X1000000 * 10LL + 500000LL) / 1000000LL)
#define ENCODER_MAX_SPEED_MMPS      3000LL
#define ENCODER_TRUST_MARGIN_NUM    2LL
#define ENCODER_TRUST_MARGIN_DEN    1LL

typedef struct {
    int64_t accumulated_counts;
    uint32_t last_counter;
    uint8_t initialized;
} EncoderState_t;

static EncoderState_t g_left_encoder;
static EncoderState_t g_right_encoder;
static EncoderDiagnostics_t g_left_diagnostics;
static EncoderDiagnostics_t g_right_diagnostics;

#ifdef ENCODER_HOST_TEST
static WheelCalibrationParameters_t g_encoder_host_parameters = {500U, 4U, 28000U, 33250U};
#define Encoder_GetParameters() (&g_encoder_host_parameters)
#else
#define Encoder_GetParameters() (Wheel_Calibration_GetParameters())
#endif

static uint32_t Encoder_Ppr(void)
{
    return Encoder_GetParameters()->encoder_ppr;
}
static uint32_t Encoder_Quadrature(void)
{
    return Encoder_GetParameters()->quadrature_factor;
}
static uint32_t Encoder_GearRatioX1000(void)
{
    return Encoder_GetParameters()->gear_ratio_x1000;
}
static int8_t Encoder_Polarity(uint8_t num)
{
#ifdef ENCODER_HOST_TEST
    return (num == ENCODER_RIGHT_MOTOR) ? ENCODER_RIGHT_POLARITY : ENCODER_LEFT_POLARITY;
#else
    const CalibrationData_t *c = Wheel_Calibration_GetActive();
    int8_t polarity = (num == ENCODER_RIGHT_MOTOR) ? ENCODER_RIGHT_POLARITY : ENCODER_LEFT_POLARITY;
    if (c->valid != 0U) {
        polarity = (num == ENCODER_RIGHT_MOTOR) ? c->right_encoder_polarity : c->left_encoder_polarity;
    }
    return polarity;
#endif
}
static int64_t Encoder_CountsPerRev(void)
{
    return ((int64_t)Encoder_Ppr() * (int64_t)Encoder_Quadrature() *
            (int64_t)Encoder_GearRatioX1000()) / 1000LL;
}
static int64_t Encoder_CircumferenceMmX1000(void)
{
    int64_t radius = Encoder_GetParameters()->wheel_radius_mm_x1000;
    return (2LL * radius * ENCODER_PI_X1000000 + 500000LL) / 1000000LL;
}

static EncoderSample_t Encoder_EmptySample(void)
{
    EncoderSample_t sample;

    sample.delta_counts = 0;
    sample.accumulated_counts = 0;
    sample.velocity_mmps = 0;
    sample.trusted = 0U;
    return sample;
}

static TIM_HandleTypeDef *Encoder_GetTimer(uint8_t num)
{
    if (num == ENCODER_LEFT_MOTOR) {
        return &htim1;
    }
    if (num == ENCODER_RIGHT_MOTOR) {
        return &htim2;
    }
    return 0;
}

static EncoderState_t *Encoder_GetState(uint8_t num)
{
    if (num == ENCODER_LEFT_MOTOR) {
        return &g_left_encoder;
    }
    if (num == ENCODER_RIGHT_MOTOR) {
        return &g_right_encoder;
    }
    return 0;
}

static EncoderDiagnostics_t *Encoder_GetDiagnosticsState(uint8_t num)
{
    if (num == ENCODER_LEFT_MOTOR) {
        return &g_left_diagnostics;
    }
    if (num == ENCODER_RIGHT_MOTOR) {
        return &g_right_diagnostics;
    }
    return 0;
}

static int32_t Encoder_CalculateDelta(uint32_t current, uint32_t previous, uint32_t period)
{
    uint64_t modulo = (uint64_t)period + 1ULL;
    int64_t delta = (int64_t)current - (int64_t)previous;
    int64_t half = (int64_t)(modulo / 2ULL);

    if (delta > half) {
        delta -= (int64_t)modulo;
    } else if (delta < -half) {
        delta += (int64_t)modulo;
    }

    return (int32_t)delta;
}

static int32_t Encoder_ApplyPolarity(uint8_t num, int32_t raw_delta)
{
    if (Encoder_Polarity(num) < 0) {
        if (raw_delta == INT32_MIN) {
            return INT32_MAX;
        }
        return -raw_delta;
    }

    return raw_delta;
}

static int32_t Encoder_CountsToMmps(int32_t delta_counts, uint32_t dt_ms)
{
    int64_t numerator;

    if (dt_ms == 0U) {
        return 0;
    }

    numerator = (int64_t)delta_counts * Encoder_CircumferenceMmX1000();
    return (int32_t)(numerator / (Encoder_CountsPerRev() * (int64_t)dt_ms));
}

static uint8_t Encoder_IsTrustedDelta(int32_t delta_counts, uint32_t dt_ms)
{
    int64_t abs_delta = delta_counts;
    int64_t max_delta;

    if (dt_ms == 0U) {
        return 0U;
    }

    if (abs_delta < 0) {
        abs_delta = -abs_delta;
    }

    max_delta = ENCODER_MAX_SPEED_MMPS * (int64_t)dt_ms * Encoder_CountsPerRev();
    max_delta *= ENCODER_TRUST_MARGIN_NUM;
    max_delta /= (Encoder_CircumferenceMmX1000() * ENCODER_TRUST_MARGIN_DEN);

    return (abs_delta <= max_delta) ? 1U : 0U;
}

void Encoder_Init(void)
{
    (void)HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    Encoder_Reset();
}

void Encoder_Reset(void)
{
    g_left_encoder.accumulated_counts = 0;
    g_left_encoder.last_counter = __HAL_TIM_GET_COUNTER(&htim1);
    g_left_encoder.initialized = 1U;
    g_left_diagnostics = (EncoderDiagnostics_t){0};

    g_right_encoder.accumulated_counts = 0;
    g_right_encoder.last_counter = __HAL_TIM_GET_COUNTER(&htim2);
    g_right_encoder.initialized = 1U;
    g_right_diagnostics = (EncoderDiagnostics_t){0};
}

EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms)
{
    EncoderSample_t sample = Encoder_EmptySample();
    EncoderState_t *state = Encoder_GetState(num);
    EncoderDiagnostics_t *diagnostics = Encoder_GetDiagnosticsState(num);
    TIM_HandleTypeDef *timer = Encoder_GetTimer(num);
    uint32_t current_counter;
    int32_t raw_delta;
    int32_t delta;

    if ((state == 0) || (diagnostics == 0) || (timer == 0)) {
        return sample;
    }

    current_counter = __HAL_TIM_GET_COUNTER(timer);
    if (state->initialized == 0U) {
        state->last_counter = current_counter;
        state->initialized = 1U;
        sample.accumulated_counts = state->accumulated_counts;
        diagnostics->raw_delta_counts = 0;
        diagnostics->applied_delta_counts = 0;
        diagnostics->dt_ms = dt_ms;
        diagnostics->accumulated_counts = state->accumulated_counts;
        diagnostics->trusted = 0U;
        diagnostics->sample_sequence++;
        diagnostics->untrusted_samples++;
        return sample;
    }

    raw_delta = Encoder_CalculateDelta(current_counter,
                                       state->last_counter,
                                       timer->Init.Period);
    state->last_counter = current_counter;
    delta = Encoder_ApplyPolarity(num, raw_delta);

    sample.delta_counts = delta;
    sample.velocity_mmps = Encoder_CountsToMmps(delta, dt_ms);
    sample.trusted = Encoder_IsTrustedDelta(delta, dt_ms);
    if (sample.trusted != 0U) {
        state->accumulated_counts += delta;
    }
    sample.accumulated_counts = state->accumulated_counts;
    diagnostics->raw_delta_counts = raw_delta;
    diagnostics->applied_delta_counts = delta;
    diagnostics->dt_ms = dt_ms;
    diagnostics->accumulated_counts = state->accumulated_counts;
    diagnostics->trusted = sample.trusted;
    diagnostics->sample_sequence++;
    if (sample.trusted != 0U) {
        diagnostics->trusted_samples++;
    } else {
        diagnostics->untrusted_samples++;
    }
    return sample;
}

int32_t Encoder_RecomputeSpeedMmps(int32_t delta_counts, uint32_t dt_ms)
{
    return Encoder_CountsToMmps(delta_counts, dt_ms);
}

void Encoder_GetSpeedEvidence(const EncoderSample_t *sample,
                              uint32_t dt_ms,
                              EncoderSpeedEvidence_t *evidence)
{
    if ((sample == 0) || (evidence == 0)) {
        return;
    }
    evidence->raw_delta_counts = sample->delta_counts;
    evidence->dt_ms = dt_ms;
    evidence->mcu_speed_mmps = sample->velocity_mmps;
    evidence->recomputed_speed_mmps = Encoder_RecomputeSpeedMmps(
        sample->delta_counts, dt_ms);
    evidence->counts_per_wheel_rev = Encoder_CountsPerRev();
    evidence->circumference_mm_x1000 = Encoder_CircumferenceMmX1000();
}

void Encoder_GetDiagnostics(uint8_t num, EncoderDiagnostics_t *out)
{
    EncoderDiagnostics_t *diagnostics = Encoder_GetDiagnosticsState(num);

    if ((diagnostics != 0) && (out != 0)) {
        *out = *diagnostics;
    }
}

float Encoder_Get(uint8_t num)
{
    EncoderSample_t sample = Encoder_Sample(num, 20U);
    return ((float)sample.velocity_mmps) / 1000.0f;
}
