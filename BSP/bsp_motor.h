#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Drive(uint8_t num, int16_t pwm);
void Motor_Coast(uint8_t num);
void Motor_Brake(uint8_t num);
void Motor_CoastAll(void);
void Motor_EmergencyBrakeAll(void);
void Motor_SetPWM(uint8_t num, int16_t PWM);

#endif
