#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"


#define PWMA   TIM1->CCR1  //PA8 	// L
#define AIN2   PCout(1)
#define AIN1   PCout(0)
#define BIN1   PCout(2)
#define BIN2   PCout(3)
#define PWMB   TIM1->CCR4  //PA11 	// R
void PWM_Init(u16 arr,u16 psc);
void Motor_Init(void);
#endif
