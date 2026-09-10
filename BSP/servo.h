#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

#define SERVO_MIN_PULSE_US       1000U
#define SERVO_CENTER_PULSE_US    1500U
#define SERVO_MAX_PULSE_US       2000U
#define SERVO_MAX_ANGLE_MRAD     600

void Servo_Init(void);
void Servo_SetNeutral(void);
void Servo_SetPulseUs(uint16_t pulse_us);
void Servo_SetAngleMrad(int16_t angle_mrad);
uint8_t Servo_SetAngleMradChecked(int16_t angle_mrad);
uint16_t Servo_GetPulseUs(void);

#endif
