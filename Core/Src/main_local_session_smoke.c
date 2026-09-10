#include "main.h"

#include "autotune_safe_session.h"

#ifndef LOCAL_SESSION_BUILD_ID
#define LOCAL_SESSION_BUILD_ID 0x4C534D31UL
#endif

extern void AutotuneSession_LocalSmokeProcess(uint32_t now_ms);

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4;
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static void ClearInheritedInterrupts(void)
{
    uint32_t index;

    __disable_irq();
    for (index = 0U; index < 8U; ++index) {
        NVIC->ICER[index] = 0xFFFFFFFFUL;
        NVIC->ICPR[index] = 0xFFFFFFFFUL;
    }
    __enable_irq();
}

int main(void)
{
    ClearInheritedInterrupts();
    HAL_Init();
    SystemClock_Config();
    AutotuneSession_Init(LOCAL_SESSION_BUILD_ID,
                         AUTOTUNE_SESSION_PROFILE_LOCAL_SMOKE,
                         AUTOTUNE_SESSION_BOOT_REASON_UNKNOWN);
    while (1) {
        AutotuneSession_LocalSmokeProcess(HAL_GetTick());
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
