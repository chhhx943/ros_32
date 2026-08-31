#ifndef __PHYSICAL_ESTOP_H
#define __PHYSICAL_ESTOP_H

#ifdef PHYSICAL_ESTOP_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#define PHYSICAL_ESTOP_PIN_NUMBER 1U

#ifdef __cplusplus
extern "C" {
#endif

void Physical_EStop_Init(void);
void Physical_EStop_Process(void);
void Physical_EStop_OnExti(uint16_t pin);
uint8_t Physical_EStop_IsAsserted(void);
uint8_t Physical_EStop_ConsumeAssertEvent(void);

#ifdef __cplusplus
}
#endif

#endif
