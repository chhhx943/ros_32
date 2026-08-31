#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
} PID_t;

void PID_Update(PID_t *p);
void PID_Reset(PID_t *p);
void PID_UpdateDt(PID_t *p, float dt_s);

#endif
