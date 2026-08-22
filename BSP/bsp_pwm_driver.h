#ifndef __BSP_PWM_DRIVER_H__
#define __BSP_PWM_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include <stdint.h>



/* PWM 编号定义 */
typedef enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
	  PWM_COUNT
} PWM_Num_t;

/* PWM 返回状态 */
typedef enum {
    BSP_PWM_OK = 0,
    BSP_PWM_ERR_INVALID_ID,
    BSP_PWM_ERR_NOT_INITIALIZED,
    BSP_PWM_ERR_INVALID_PARAM
} BSP_PWM_Status_t;

/**
 * @brief 初始化指定 PWM
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Init(PWM_Num_t pwm);

/**
 * @brief 启动 PWM 输出
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Start(PWM_Num_t pwm);

/**
 * @brief 停止 PWM 输出
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Stop(PWM_Num_t pwm);

/**
 * @brief 设置 PWM 占空比
 * @param pwm: PWM 编号
 * @param duty: 占空比 0~1000 (0~100%)
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_SetDuty(PWM_Num_t pwm, uint16_t duty);

/**
 * @brief 获取 PWM 占空比
 * @param pwm: PWM 编号
 * @param out_duty: 输出占空比
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_GetDuty(PWM_Num_t pwm, uint16_t *out_duty);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_DRIVER_H__ */
