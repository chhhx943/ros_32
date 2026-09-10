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

typedef struct {
    uint32_t timestamp_ms;
    uint16_t command_seq;
    int16_t target_left_mmps;
    int16_t target_right_mmps;
    int16_t actual_left_mmps;
    int16_t actual_right_mmps;
    int32_t encoder_delta_left;
    int32_t encoder_delta_right;
    int16_t pwm_left;
    int16_t pwm_right;
    float pid_left_p;
    float pid_left_i;
    float pid_left_d;
    float pid_left_output;
    float pid_right_p;
    float pid_right_i;
    float pid_right_d;
    float pid_right_output;
    uint8_t safety_state;
    uint16_t fault_code;
} Chassis_ControlTelemetry_t;

void Chassis_ControlInit(void);
void Chassis_ControlProcess(uint32_t now_ms);
/* TIM6 1 kHz ISR/event bridge.  The ISR only posts bounded work; all motor
   and CAN operations remain in the main context. */
void Chassis_ControlOnTick(void);
void Chassis_ControlProcessEvents(uint32_t now_ms);
void Chassis_Control_GetTelemetry(Chassis_ControlTelemetry_t *out);

#ifdef __cplusplus
}
#endif

#endif
