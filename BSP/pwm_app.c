#include "pwm_app.h"
/**
 * @brief 注册一个 PWM
 */
BSP_PWM_Status_t PWM_Register(PWM_Handle_t *handle, PWM_Num_t id)
{
    if (!handle) return BSP_PWM_ERR_INVALID_ID;

    BSP_PWM_Status_t status = BSP_PWM_Init(id);
    if (status == BSP_PWM_OK) {
        handle->id = id;
        handle->duty = 0;
        handle->target_duty = 0;
        handle->ramp_interval = 0;
        handle->last_tick = 0;
        handle->running = 0;
    }
    return status;
}

/**
 * @brief 启动 PWM 输出
 */
BSP_PWM_Status_t PWM_Start(PWM_Handle_t *handle)
{
    if (!handle) return BSP_PWM_ERR_INVALID_ID;

    BSP_PWM_Status_t status = BSP_PWM_Start(handle->id);
    if (status == BSP_PWM_OK) {
        handle->running = 1;
    }
    return status;
}

/**
 * @brief 停止 PWM 输出
 */
BSP_PWM_Status_t PWM_Stop(PWM_Handle_t *handle)
{
    if (!handle) return BSP_PWM_ERR_INVALID_ID;

    BSP_PWM_Status_t status = BSP_PWM_Stop(handle->id);
    if (status == BSP_PWM_OK) {
        handle->running = 0;
    }
    return status;
}

/**
 * @brief 设置 PWM 占空比
 */
BSP_PWM_Status_t PWM_SetDuty(PWM_Handle_t *handle, uint16_t duty)
{
    if (!handle) return BSP_PWM_ERR_INVALID_ID;

    BSP_PWM_Status_t status = BSP_PWM_SetDuty(handle->id, duty);
    if (status == BSP_PWM_OK) {
        handle->duty = duty;
        handle->target_duty = duty;
    }
    return status;
}

/**
 * @brief 渐变占空比（类似 LED 闪烁）
 * @param ramp_interval_ms: 每次更新的时间间隔
 * @param target_duty: 目标占空比
 */
void PWM_StartRamp(PWM_Handle_t *handle, uint16_t target_duty, uint32_t ramp_interval_ms)
{
    if (!handle) return;
    handle->target_duty = (target_duty > 1000) ? 1000 : target_duty;
    handle->ramp_interval = ramp_interval_ms;
    handle->last_tick = 0;
}

/**
 * @brief 停止渐变
 */
void PWM_StopRamp(PWM_Handle_t *handle)
{
    if (!handle) return;
    handle->ramp_interval = 0;
}

/**
 * @brief PWM 更新函数，需在循环中定时调用
 */
void PWM_Process(PWM_Handle_t *handle, uint32_t current_tick)
{
    if (!handle || handle->ramp_interval == 0) return;

    if ((current_tick - handle->last_tick) >= handle->ramp_interval) {
        if (handle->duty < handle->target_duty) {
            handle->duty++;
        } else if (handle->duty > handle->target_duty) {
            handle->duty--;
        }

        BSP_PWM_SetDuty(handle->id, handle->duty);
        handle->last_tick = current_tick;
    }
}
