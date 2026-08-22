#include "bsp_motor.h"
#include "stm32f407xx.h"
#define AIN1_ON   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
#define AIN1_OFF  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
#define AIN2_ON   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
#define AIN2_OFF  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
#define BIN1_ON   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
#define BIN1_OFF  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
#define BIN2_ON   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
#define BIN2_OFF  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

extern PWM_Handle_t pwm1;
extern PWM_Handle_t pwm2;
void Motor_SetPWM(uint8_t num,int16_t PWM)
{
  if(PWM>0)
	{
		if(num == 1)
		{
		  AIN1_ON;
		  AIN2_OFF;	
		  PWM_SetDuty(&pwm1,PWM);
		}
		else if(num==2)
		{
		  BIN1_OFF;
		  BIN2_ON;	
		  PWM_SetDuty(&pwm2,PWM);		  
		}
	}
  else 
	{
		if(num == 1)
		{
		  AIN1_OFF;
		  AIN2_ON;	
			PWM_SetDuty(&pwm1,-PWM);
		}
		else if(num==2)
		{
		  BIN1_ON;
		  BIN2_OFF;	
			PWM_SetDuty(&pwm2,-PWM);		  
		}
	}

	
}