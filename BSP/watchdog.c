#include "watchdog.h"
#ifndef BSP_BXCAN_HOST_TEST
#ifdef CAN_DEBUG_NO_WATCHDOG
void BSP_Watchdog_Init(void) {}
void BSP_Watchdog_Feed(void) {}
#else
void BSP_Watchdog_Init(void)
{
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U) {}
    IWDG->KR = 0x5555U;
    IWDG->PR = 3U;       /* /32 */
    IWDG->RLR = 2000U;   /* approximately two seconds at 32 kHz */
    IWDG->KR = 0xAAAAU;
    IWDG->KR = 0xCCCCU;
}

void BSP_Watchdog_Feed(void)
{
    IWDG->KR = 0xAAAAU;
}
#endif
#else
void BSP_Watchdog_Init(void) {}
void BSP_Watchdog_Feed(void) {}
#endif
