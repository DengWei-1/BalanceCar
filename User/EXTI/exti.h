#ifndef __EXTI_H
#define __EXTI_H	 
#include "sys.h"
 
#define MPU_INT PBin(5)   //PB5连接到MPU6050的中断引脚
void EXTI_5_Init(void);	//外部中断初始化	

#endif

