#include "main.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* CubeProgrammer -run is a core reset, not a full peripheral power cycle.
   Keep inherited watchdog/peripheral interrupts from entering a weak handler
   before the actuator-free Smoke main has established its own runtime. */
void WWDG_IRQHandler(void)
{
}

void RCC_IRQHandler(void)
{
}
