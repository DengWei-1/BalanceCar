#include "sys.h"
#include "delay.h"
#include "usart.h"	  
#include "led.h"
#include "oled.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor.h"
#include "show.h"
#include "exti.h"
#include "control.h"	
#include "usart2.h"		
#include "DataScope_DP.h"
#include "adc.h"


u8 delay_50,delay_500,delay_flag,Blue_flag,PID_Send;  //延时和调参等变量
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=0,Flag_Show=0;  
int Moto1,Moto2;                            //电机PWM变量
int Temperature,Voltage;                                //电池电压采样相关的变量
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;   
float Balance_Kp=156,Balance_Kd=0.6,Velocity_Kp=2.2,Velocity_Ki=0.011; 	//PID参数 	
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据

int main(void)
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//--设置NVIC中断分组2:2位抢占优先级，2位响应优先级 	
	delay_init();	    	 //--延时函数初始化	  
	LED_Init();			     //--LED端口初始化
	uart_init(115200);
    OLED_Init();			 //--初始化OLED  
	
	Encoder_Init_TIM2();     //--编码器接口
	Encoder_Init_TIM3();
	MPU6050_initialize();    //--MPU6050初始化	
	DMP_Init();              //--初始化DMP   
	LED=1;
	Motor_Init();
	PWM_Init(7199,0);    	 //--初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
	uart2_init(9600);
	EXTI_5_Init();
	Adc_Init();
	oled_oneshow();
	
	while(1)
	{ 	
		
		APP_Show();
		oled_show();          //===显示屏打开
			
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	  //通过MPU6050的INT中断实现的50ms精准延时	
		
		LED = !LED; 	 	  //--运行指示灯
	
			
	} 
 

}

