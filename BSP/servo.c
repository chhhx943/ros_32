#include "servo.h"
#include "tim.h"

static uint16_t g_servo_pulse_us = SERVO_CENTER_PULSE_US;
#define SERVO_MAX_STEP_US 50U

static uint16_t Servo_ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }
    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }
    return pulse_us;
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    g_servo_pulse_us = Servo_ClampPulse(pulse_us);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, g_servo_pulse_us);
}

void Servo_SetNeutral(void)
{
    Servo_SetPulseUs(SERVO_CENTER_PULSE_US);
}

void Servo_SetAngleMrad(int16_t angle_mrad)
{
    (void)Servo_SetAngleMradChecked(angle_mrad);
}

uint8_t Servo_SetAngleMradChecked(int16_t angle_mrad)
{
    int32_t bounded_angle = angle_mrad;
    int32_t pulse_us;

    /* Vehicle-envelope violations are rejected, never silently clamped. */
    if ((bounded_angle > SERVO_MAX_ANGLE_MRAD) ||
        (bounded_angle < -SERVO_MAX_ANGLE_MRAD)) {
        return 0U;
    }

    pulse_us = (int32_t)SERVO_CENTER_PULSE_US +
               (bounded_angle * ((int32_t)SERVO_MAX_PULSE_US - (int32_t)SERVO_CENTER_PULSE_US)) /
                   SERVO_MAX_ANGLE_MRAD;
    if (pulse_us > (int32_t)g_servo_pulse_us + (int32_t)SERVO_MAX_STEP_US) {
        pulse_us = (int32_t)g_servo_pulse_us + (int32_t)SERVO_MAX_STEP_US;
    } else if (pulse_us + (int32_t)SERVO_MAX_STEP_US < (int32_t)g_servo_pulse_us) {
        pulse_us = (int32_t)g_servo_pulse_us - (int32_t)SERVO_MAX_STEP_US;
    }
    Servo_SetPulseUs((uint16_t)pulse_us);
    return 1U;
}

uint16_t Servo_GetPulseUs(void)
{
    return g_servo_pulse_us;
}

void Servo_Init(void)
{
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_SetNeutral();
}
