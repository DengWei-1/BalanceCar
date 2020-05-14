#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern int Encoder_Left,Encoder_Right;   

void TIM4_Int_Init(u16 arr,u16 psc);
void Read_Distane(void);
void TIM4_IRQHandler(void);
#endif
