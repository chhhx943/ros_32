#include "bsp_motor.h"
#include "tim.h"

static int Motor_IsValid(uint8_t num)
{
    return (num == 1U) || (num == 2U);
}

static uint32_t Motor_GetPwmChannel(uint8_t num)
{
    return (num == 1U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static uint16_t Motor_AbsClampPwm(int16_t pwm)
{
    int32_t value = pwm;

    if (value < 0) {
        value = -value;
    }
    if (value > 1000) {
        value = 1000;
    }
    return (uint16_t)value;
}

static void Motor_SetDuty(uint8_t num, uint16_t duty)
{
    if (!Motor_IsValid(num)) {
        return;
    }
    if (duty > 1000U) {
        duty = 1000U;
    }

    __HAL_TIM_SET_COMPARE(&htim3, Motor_GetPwmChannel(num),
                          (htim3.Init.Period + 1U) * duty / 1000U);
}

static void Motor_SetTb6612Coast(uint8_t num)
{
    if (num == 1U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else if (num == 2U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

static void Motor_SetTb6612Brake(uint8_t num)
{
    if (num == 1U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    } else if (num == 2U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
}

static void Motor_SetTb6612DriveDirection(uint8_t num, int16_t pwm)
{
    if (num == 1U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (pwm >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (pwm >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else if (num == 2U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (pwm >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (pwm >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void Motor_Init(void)
{
    Motor_SetDuty(1U, 0U);
    Motor_SetDuty(2U, 0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Motor_CoastAll();
}

void Motor_Drive(uint8_t num, int16_t pwm)
{
    uint16_t duty;

    if (!Motor_IsValid(num)) {
        return;
    }

    if (pwm == 0) {
        Motor_Coast(num);
        return;
    }

    duty = Motor_AbsClampPwm(pwm);
    Motor_SetDuty(num, 0U);
    Motor_SetTb6612DriveDirection(num, pwm);
    Motor_SetDuty(num, duty);
}

void Motor_Coast(uint8_t num)
{
    if (!Motor_IsValid(num)) {
        return;
    }

    Motor_SetDuty(num, 0U);
    Motor_SetTb6612Coast(num);
}

void Motor_Brake(uint8_t num)
{
    if (!Motor_IsValid(num)) {
        return;
    }

    Motor_SetDuty(num, 0U);
    Motor_SetTb6612Brake(num);
}

void Motor_CoastAll(void)
{
    Motor_Coast(1U);
    Motor_Coast(2U);
}

void Motor_EmergencyBrakeAll(void)
{
    Motor_Brake(1U);
    Motor_Brake(2U);
}

void Motor_SetPWM(uint8_t num, int16_t PWM)
{
    Motor_Drive(num, PWM);
}
