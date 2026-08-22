/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm_driver.h"
#include "tim.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint8_t initialized;           /*!< 标记 PWM 是否已初始化 */
    uint16_t duty_cycle;           /*!< 当前占空比 0~1000 (0~100%) */
    uint8_t running;               /*!< 是否正在输出 PWM */
} PWM_HW_Map_t;

/* Private variables ---------------------------------------------------------*/
/* PWM 硬件映射表 */
static PWM_HW_Map_t pwm_hw_map[PWM_COUNT] = {
    [PWM1] = {&htim3, TIM_CHANNEL_1, 0, 0, 0},
    [PWM2] = {&htim3, TIM_CHANNEL_2, 0, 0, 0},

};

/* Private function prototypes -----------------------------------------------*/
static int BSP_PWM_IsValid(PWM_Num_t pwm);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化指定 PWM
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Init(PWM_Num_t pwm)
{
    if (!BSP_PWM_IsValid(pwm)) {
        return BSP_PWM_ERR_INVALID_ID;
    }

    /* 如果 HAL 库的 TIM 已经初始化，这里可直接标记 */
    pwm_hw_map[pwm].initialized = 1;
    pwm_hw_map[pwm].duty_cycle = 0;
    pwm_hw_map[pwm].running = 0;

    return BSP_PWM_OK;
}

/**
 * @brief 启动 PWM 输出
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Start(PWM_Num_t pwm)
{
    if (!BSP_PWM_IsValid(pwm)) return BSP_PWM_ERR_INVALID_ID;
    if (!pwm_hw_map[pwm].initialized) return BSP_PWM_ERR_NOT_INITIALIZED;

    HAL_TIM_PWM_Start(pwm_hw_map[pwm].htim, pwm_hw_map[pwm].channel);
    pwm_hw_map[pwm].running = 1;
    return BSP_PWM_OK;
}

/**
 * @brief 停止 PWM 输出
 * @param pwm: PWM 编号
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_Stop(PWM_Num_t pwm)
{
    if (!BSP_PWM_IsValid(pwm)) return BSP_PWM_ERR_INVALID_ID;
    if (!pwm_hw_map[pwm].initialized) return BSP_PWM_ERR_NOT_INITIALIZED;

    HAL_TIM_PWM_Stop(pwm_hw_map[pwm].htim, pwm_hw_map[pwm].channel);
    pwm_hw_map[pwm].running = 0;
    return BSP_PWM_OK;
}

/**
 * @brief 设置 PWM 占空比
 * @param pwm: PWM 编号
 * @param duty: 占空比 0~1000 (对应 0~100%)
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_SetDuty(PWM_Num_t pwm, uint16_t duty)
{
    if (!BSP_PWM_IsValid(pwm)) return BSP_PWM_ERR_INVALID_ID;
    if (!pwm_hw_map[pwm].initialized) return BSP_PWM_ERR_NOT_INITIALIZED;
    if (duty > 1000) duty = 1000;

    __HAL_TIM_SET_COMPARE(pwm_hw_map[pwm].htim, pwm_hw_map[pwm].channel,
                          (pwm_hw_map[pwm].htim->Init.Period + 1) * duty / 1000);
    pwm_hw_map[pwm].duty_cycle = duty;
    return BSP_PWM_OK;
}

/**
 * @brief 获取 PWM 占空比
 * @param pwm: PWM 编号
 * @param out_duty: 输出占空比
 * @retval BSP_PWM_Status_t
 */
BSP_PWM_Status_t BSP_PWM_GetDuty(PWM_Num_t pwm, uint16_t *out_duty)
{
    if (!BSP_PWM_IsValid(pwm)) return BSP_PWM_ERR_INVALID_ID;
    if (!pwm_hw_map[pwm].initialized) return BSP_PWM_ERR_NOT_INITIALIZED;
    if (out_duty == NULL) return BSP_PWM_ERR_INVALID_PARAM;

    *out_duty = pwm_hw_map[pwm].duty_cycle;
    return BSP_PWM_OK;
}

/* Private functions ---------------------------------------------------------*/
static int BSP_PWM_IsValid(PWM_Num_t pwm)
{
    return (pwm >= 0 && pwm < PWM_COUNT);
}

