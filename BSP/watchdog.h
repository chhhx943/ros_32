#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H
#include <stdint.h>
#ifndef BSP_BXCAN_HOST_TEST
#include "main.h"
#endif
void BSP_Watchdog_Init(void);
void BSP_Watchdog_Feed(void);
#endif
