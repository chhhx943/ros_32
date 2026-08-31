#include "encoder.h"
#include "tim.h"

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
    if ((num == ENCODER_RIGHT_MOTOR) && (ENCODER_RIGHT_POLARITY < 0)) {
        if (raw_delta == INT32_MIN) {
            return INT32_MAX;
        }
        return -raw_delta;
    }

    return (ENCODER_LEFT_POLARITY < 0) ? -raw_delta : raw_delta;
}

static int32_t Encoder_CountsToMmps(int32_t delta_counts, uint32_t dt_ms)
{
    int64_t numerator;

    if (dt_ms == 0U) {
        return 0;
    }

    numerator = (int64_t)delta_counts * ENCODER_WHEEL_CIRCUM_MM_X1000;
    return (int32_t)(numerator / (ENCODER_COUNTS_PER_REV * (int64_t)dt_ms));
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

    max_delta = ENCODER_MAX_SPEED_MMPS * (int64_t)dt_ms * ENCODER_COUNTS_PER_REV;
    max_delta *= ENCODER_TRUST_MARGIN_NUM;
    max_delta /= (ENCODER_WHEEL_CIRCUM_MM_X1000 * ENCODER_TRUST_MARGIN_DEN);

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

    g_right_encoder.accumulated_counts = 0;
    g_right_encoder.last_counter = __HAL_TIM_GET_COUNTER(&htim2);
    g_right_encoder.initialized = 1U;
}

EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms)
{
    EncoderSample_t sample = Encoder_EmptySample();
    EncoderState_t *state = Encoder_GetState(num);
    TIM_HandleTypeDef *timer = Encoder_GetTimer(num);
    uint32_t current_counter;
    int32_t delta;

    if ((state == 0) || (timer == 0)) {
        return sample;
    }

    current_counter = __HAL_TIM_GET_COUNTER(timer);
    if (state->initialized == 0U) {
        state->last_counter = current_counter;
        state->initialized = 1U;
        sample.accumulated_counts = state->accumulated_counts;
        return sample;
    }

    delta = Encoder_CalculateDelta(current_counter, state->last_counter, timer->Init.Period);
    state->last_counter = current_counter;
    delta = Encoder_ApplyPolarity(num, delta);

    sample.delta_counts = delta;
    sample.velocity_mmps = Encoder_CountsToMmps(delta, dt_ms);
    sample.trusted = Encoder_IsTrustedDelta(delta, dt_ms);
    if (sample.trusted != 0U) {
        state->accumulated_counts += delta;
    }
    sample.accumulated_counts = state->accumulated_counts;
    return sample;
}

float Encoder_Get(uint8_t num)
{
    EncoderSample_t sample = Encoder_Sample(num, 20U);
    return ((float)sample.velocity_mmps) / 1000.0f;
}
