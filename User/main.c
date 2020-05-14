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


u8 delay_50,delay_500,delay_flag,Blue_flag,PID_Send;  //��ʱ�͵��εȱ���
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=0,Flag_Show=0;  
int Moto1,Moto2;                            //���PWM����
int Temperature,Voltage;                                //��ص�ѹ������صı���
int Encoder_Left,Encoder_Right;             //���ұ��������������
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Acceleration_Z;   
float Balance_Kp=156,Balance_Kd=0.6,Velocity_Kp=2.2,Velocity_Ki=0.011; 	//PID���� 	
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������

int main(void)
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//--����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ� 	
	delay_init();	    	 //--��ʱ������ʼ��	  
	LED_Init();			     //--LED�˿ڳ�ʼ��
	uart_init(115200);
    OLED_Init();			 //--��ʼ��OLED  
	
	Encoder_Init_TIM2();     //--�������ӿ�
	Encoder_Init_TIM3();
	MPU6050_initialize();    //--MPU6050��ʼ��	
	DMP_Init();              //--��ʼ��DMP   
	LED=1;
	Motor_Init();
	PWM_Init(7199,0);    	 //--��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
	uart2_init(9600);
	EXTI_5_Init();
	Adc_Init();
	oled_oneshow();
	
	while(1)
	{ 	
		
		APP_Show();
		oled_show();          //===��ʾ����
			
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	  //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
		
		LED = !LED; 	 	  //--����ָʾ��
	
			
	} 
 

}

