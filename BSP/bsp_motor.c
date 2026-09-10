#include "bsp_motor.h"
#include "autotune_safe.h"
#include "tim.h"

#define MOTOR_DIRECTION_DEAD_TIME_MS 2U

static int8_t g_direction_sign[2];
static int16_t g_pending_pwm[2];
static uint32_t g_deadline_ms[2];
static uint8_t g_deadtime_pending[2];

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

static void Motor_ApplyDrive(uint8_t num, int16_t pwm)
{
    uint8_t index = (uint8_t)(num - 1U);
    uint16_t duty = Motor_AbsClampPwm(pwm);

    /* This helper is called only after the direction interlock has been
       satisfied.  Keep the state update next to the actual GPIO/PWM write so
       a completed reversal cannot be queued again by Motor_Process(). */
    g_deadtime_pending[index] = 0U;
    g_direction_sign[index] = (pwm < 0) ? -1 : 1;
    Motor_SetDuty(num, 0U);
    Motor_SetTb6612DriveDirection(num, pwm);
    Motor_SetDuty(num, duty);
}

void Motor_Init(void)
{
    g_direction_sign[0] = 0; g_direction_sign[1] = 0;
    g_pending_pwm[0] = 0; g_pending_pwm[1] = 0;
    g_deadtime_pending[0] = 0U; g_deadtime_pending[1] = 0U;
    Motor_SetDuty(1U, 0U);
    Motor_SetDuty(2U, 0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Motor_CoastAll();
}

void Motor_Drive(uint8_t num, int16_t pwm)
{
    uint8_t index;
    int8_t sign;

    if (!Motor_IsValid(num)) {
        return;
    }

#ifdef AUTOTUNE_SAFE_PROFILE
    pwm = AutotuneSafe_ActuatorGate(num, pwm);
#endif

    if (pwm == 0) {
        Motor_Coast(num);
        return;
    }

    index = (uint8_t)(num - 1U);
    sign = (pwm < 0) ? -1 : 1;
    if ((g_direction_sign[index] != 0) && (g_direction_sign[index] != sign)) {
        /* Reverse only after PWM=0 and a COAST dead-time interval. */
        Motor_SetDuty(num, 0U);
        Motor_SetTb6612Coast(num);
        g_pending_pwm[index] = pwm;
        g_deadline_ms[index] = HAL_GetTick() + MOTOR_DIRECTION_DEAD_TIME_MS;
        g_deadtime_pending[index] = 1U;
        return;
    }
    Motor_ApplyDrive(num, pwm);
}

void Motor_Process(uint32_t now_ms)
{
    uint8_t index;
    for (index = 0U; index < 2U; ++index) {
        if ((g_deadtime_pending[index] != 0U) &&
            ((int32_t)(now_ms - g_deadline_ms[index]) >= 0)) {
            int16_t pending_pwm = g_pending_pwm[index];

            g_deadtime_pending[index] = 0U;
            /* The output is coasting here.  Reset the remembered direction
               before re-entering Motor_Drive(), otherwise the completed
               reversal is mistaken for a new reversal and is queued forever. */
            g_direction_sign[index] = 0;
            Motor_Drive((uint8_t)(index + 1U), pending_pwm);
        }
    }
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
