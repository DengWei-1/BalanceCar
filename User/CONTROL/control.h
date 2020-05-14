#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
 
 
#define PI 3.14159265
#define ZHONGZHI 3.3
#define DIFFERENCE 100

int EXTI15_10_IRQHandler(void);
int balance(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pnumber(void);
u8 Turn_Off(float angle);
void Get_Angle(void);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);

#endif

