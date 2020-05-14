#include "oled.h"
#include "show.h"
#include "timer.h"
#include "mpu6050.h"
#include "adc.h"
#include "bmp.h" 	
#include "delay.h"
#include "motor.h"

extern u8 delay_flag,show1_flag,show2_flag,show3_flag;
extern u8 PID_Send,Flag_Show,Blue_flag;
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki; 	//PID参数 	
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
	extern float Angle_Balance;
float Vol;
extern int Temperature,Voltage;
int V_n;

/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	static u8 app_flag;
	int En_R,En_L,V_number;
	blueflag(); 	// 更新ADC2采集
	Flag_Show=1; 	// 串口2 printf
	
	V_number=(Voltage-1060)/2;		if(V_number<1)V_number=1;if(V_number>100)V_number=100;   //对电压数据进行处理
	V_n=V_number;
	En_R=Encoder_Right*1.1; if(En_R<0)En_R=-En_R;			                   //对编码器数据就行数据处理便于图形化
	En_L=Encoder_Left*1.1;  if(En_L<0)En_L=-En_L;
	app_flag=!app_flag;
	if(PID_Send==1)//发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)(Balance_Kp*100),(int)(Balance_Kd*100),(int)(Velocity_Kp*100),(int)(Velocity_Ki*100),0,0,0,0,0);//打印到APP上面	
		PID_Send=0;	
	}	
	else if(app_flag==0)
		printf("{A%d:%d:%d:%d}$",(u8)En_R,En_L,V_number,(int)Angle_Balance);  	//打印到APP上面
	else
		printf("{B%d:%d:%d}$",(int)Angle_Balance,Encoder_Left,Encoder_Right); 	//打印到APP上面 显示波形
	
}
void oled_oneshow(void)
{
	u8 sss=200;
	
	OLED_DrawBMP(0,0,124,5,BMP_SHARK);
	while(sss--)delay_ms(1000);
	OLED_Clear();
	
	OLED_DrawBMP(0,0,64,2,BMP1);
	
	OLED_DrawBMP(0,3,32,5,BMP2);
	OLED_DrawBMP(48,3,64,5,BMP3);
	OLED_DrawBMP(64,3,128,5,BMP4);
	
	OLED_DrawBMP(0,6,32,8,BMP5);
	OLED_ShowString(32,6,":",16);
	OLED_DrawBMP(64,6,128,8,BMP6);
	sss=200;
	while(sss--)delay_ms(1000);
	OLED_Clear();
}
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
    if(Blue_flag)
	{
		OLED_DrawBMP(0,0,16,2,BMP_Blue);
		if(show1_flag)
			OLED_DrawBMP(16,0,32,2,BMP_Blue_1);
		else
			OLED_DrawBMP(16,0,32,2,BMP__);
	}
	else
	{
		if(show1_flag)
			OLED_DrawBMP(0,0,16,2,BMP_Blue);
		else
			OLED_DrawBMP(0,0,16,2,BMP__);
		
		OLED_DrawBMP(16,0,32,2,BMP__);
	}
	
	if(show2_flag)
		OLED_DrawBMP(112,0,128,2,BMP_Battary);
	else
		OLED_DrawBMP(112,0,128,2,BMP__);
	
	if(Temperature>=294)
		OLED_DrawBMP(96,0,112,2,BMP_T);
	else
		OLED_DrawBMP(96,0,112,2,BMP__);
	
	
	OLED_ShowString(0,3,"PITCH:",12);
	if(Angle_Balance>=0)
	{
		OLED_ShowString(56,3," ",12);
		OLED_ShowNum(62,3,Angle_Balance/1,2,12);
		OLED_ShowString(76,3,".",12);
		OLED_ShowNum(80,3,(int)(Angle_Balance*1000000)%1000000,6,12);
	}
	else
	{
		OLED_ShowString(56,3,"-",12);
		OLED_ShowNum(62,3,-Angle_Balance/1,2,12);
		OLED_ShowString(76,3,".",12);
		OLED_ShowNum(80,3,-(int)(Angle_Balance*1000000)%1000000,6,12);
	}
	
	OLED_ShowString(0,4,"Battery:",12);
	OLED_ShowNum(62,4,V_n,3,12);
	OLED_ShowString(80,4,"%",12);
	OLED_ShowNum(103,4,Voltage,4,12);
	
	OLED_ShowString(0,5,"L:",12);
	if(Encoder_Left>=0)
	{
		OLED_ShowString(12,5," ",12);
		OLED_ShowNum(18,5,Encoder_Left,5,12);
	}
	else
	{
		OLED_ShowString(12,5,"-",12);
		OLED_ShowNum(18,5,-Encoder_Left,5,12); 
	}	
	OLED_ShowString(79,5,"R:",12);	
	if(Encoder_Right>=0)
	{
		OLED_ShowString(91,5," ",12);
		OLED_ShowNum(97,5,Encoder_Right,5,12); 
	}
	else
	{
		OLED_ShowString(91,5,"-",12);
		OLED_ShowNum(97,5,-Encoder_Right,5,12); 
	}
	
	
	OLED_ShowString(0,6,"PWA:",12);
	OLED_ShowNum(30,6,PWMA,4,12);
	OLED_ShowString(73,6,"PWB:",12);	
	OLED_ShowNum(103,6,PWMB,4,12); 
	
	OLED_ShowString(0,7,"Temperature:",12);
	OLED_ShowNum(98,7,Temperature/10,3,12); 
	OLED_ShowString(116,7,".",12);
	OLED_ShowNum(122,7,Temperature%10,1,12); 
	
	
}


