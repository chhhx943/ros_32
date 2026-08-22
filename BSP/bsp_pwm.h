#ifndef  __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_pwm_driver.h"
#include <stdint.h>

/* PWM 高级句柄结构体 */
typedef struct {
    PWM_Num_t id;              /*!< PWM 编号 */
    uint16_t duty;             /*!< 当前占空比 0~1000 */
    uint16_t target_duty;      /*!< 渐变/动画目标占空比 */
    uint32_t ramp_interval;    /*!< 渐变周期（ms） */
    uint32_t last_tick;        /*!< 上次更新时间 */
    uint8_t running;           /*!< 是否正在输出 PWM */
} PWM_Handle_t;

/* 注册 PWM */
BSP_PWM_Status_t PWM_Register(PWM_Handle_t *handle, PWM_Num_t id);

/* 启动 PWM 输出 */
BSP_PWM_Status_t PWM_Start(PWM_Handle_t *handle);

/* 停止 PWM 输出 */
BSP_PWM_Status_t PWM_Stop(PWM_Handle_t *handle);

/* 设置 PWM 占空比 */
BSP_PWM_Status_t PWM_SetDuty(PWM_Handle_t *handle, uint16_t duty);

/* 开始占空比渐变 / 动画 */
void PWM_StartRamp(PWM_Handle_t *handle, uint16_t target_duty, uint32_t ramp_interval_ms);

/* 停止占空比渐变 / 动画 */
void PWM_StopRamp(PWM_Handle_t *handle);

/* PWM 处理函数，需要在循环中定时调用 */
void PWM_Process(PWM_Handle_t *handle, uint32_t current_tick);

#ifdef __cplusplus
}
#endif

#endif