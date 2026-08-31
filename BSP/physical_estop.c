#include "physical_estop.h"

#ifdef PHYSICAL_ESTOP_HOST_TEST
extern uint8_t g_physical_estop_pin_level;

static uint8_t Physical_EStop_ReadActive(void)
{
    return (g_physical_estop_pin_level == 0U) ? 1U : 0U;
}

#define PHYSICAL_ESTOP_EXTI_PIN PHYSICAL_ESTOP_PIN_NUMBER
#else
#include "gpio.h"

static uint8_t Physical_EStop_ReadActive(void)
{
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1U : 0U;
}

#define PHYSICAL_ESTOP_EXTI_PIN GPIO_PIN_1
#endif

static volatile uint8_t g_physical_estop_asserted;
static volatile uint8_t g_physical_estop_assert_event;

void Physical_EStop_Init(void)
{
    g_physical_estop_asserted = Physical_EStop_ReadActive();
    g_physical_estop_assert_event = g_physical_estop_asserted;
}

void Physical_EStop_Process(void)
{
    uint8_t active = Physical_EStop_ReadActive();

    g_physical_estop_asserted = active;
    if (active != 0U) {
        g_physical_estop_assert_event = 1U;
    }
}

void Physical_EStop_OnExti(uint16_t pin)
{
    if (pin != PHYSICAL_ESTOP_EXTI_PIN) {
        return;
    }

    g_physical_estop_asserted = 1U;
    g_physical_estop_assert_event = 1U;
}

uint8_t Physical_EStop_IsAsserted(void)
{
    return g_physical_estop_asserted;
}

uint8_t Physical_EStop_ConsumeAssertEvent(void)
{
    uint8_t event = g_physical_estop_assert_event;

    g_physical_estop_assert_event = 0U;
    return event;
}
