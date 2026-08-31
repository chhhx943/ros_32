#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#ifdef CHASSIS_CONTROL_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void Chassis_ControlInit(void);
void Chassis_ControlProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
