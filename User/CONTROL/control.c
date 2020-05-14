#include "control.h"	
#include "filter.h"	
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "exti.h"
#include "usart.h"	 
#include "led.h"
#include "adc.h"

extern u8 usart_flag;
extern int Temperature,Voltage;                                //��ص�ѹ������صı���
extern u8 Flag_Stop,Flag_Show;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; //����ң����صı���
extern int Temperature;                            //��ʾmpu�¶�
extern int Encoder_Left,Encoder_Right;   
extern u8 delay_50,delay_500,delay_flag,Bi_zhang,PID_Send,Flash_Send;  //��ʱ�͵��εȱ���
extern int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�
extern float Acceleration_Z;  
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki;//PID����

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target,show1_flag,show2_flag,show3_flag;
int Voltage_Temp,Voltage_Count,Voltage_All;

/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI9_5_IRQHandler(void) 
{    
	
	if(MPU_INT==0)		
	{   
		EXTI->PR=1<<5;                                                      //���LINE5�ϵ��жϱ�־λ   
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			if(++delay_50==10)//���������ṩ50ms�ľ�׼��ʱ
				delay_50=0,delay_flag=0;  
			if(++delay_500==100)
			{
				Temperature=Read_Temperature();
				delay_500=0;
				show1_flag=!show1_flag;  
			}                  
		}
		if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		{
			//Get_Angle();                                             	 //===������̬			
			Voltage_Temp=Get_battery_volt();		                                //=====��ȡ��ص�ѹ		
			Voltage_Count++;                                                    //=====ƽ��ֵ������
			Voltage_All+=Voltage_Temp;                                          //=====��β����ۻ�
			if(Voltage_Count==100) 
				Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0; 	 	//=====��ƽ��ֵ		
			if(Voltage<500)show2_flag=1;
			else show2_flag=0;
			//return 0;	                                               
		}               	                                              	//===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
		Encoder_Left=-Read_Encoder(2);                                      //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=Read_Encoder(3);                                      //===��ȡ��������ֵ
	 	Get_Angle();                                               //===������̬	
		
		Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����	
		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		Turn_Pwm = turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===ת��PID����     
		
		Moto1 = Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===�������ֵ������PWM
		Moto2 = Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===�������ֵ������PWM
		Xianfu_Pnumber();              	 	 	 	 	 	 	 	 	 	  //===PWM�޷�
		
//		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//===����Ƿ�С��������
//			Flag_Stop=1;	                                                      //===���������͹رյ��
//		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))              //===����Ƿ�С��������
//			Flag_Stop=0;	                                                      //===��������¾��������
		
		if(Turn_Off(Angle_Balance)==0)                              //===����������쳣
		  	Set_Pwm(Moto1,Moto2);
		
	}       	
	return 0;	
	
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle;     	  	 	 	 	 	   //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;
	static float Encoder_Integral,Target_Velocity;
	//=============ң��ǰ�����˲���=======================// 
	if(Flag_sudu==1)  Target_Velocity=600;                 
    else 	          Target_Velocity=1200;
	
	if(1==Flag_Qian)	
		Movement=Target_Velocity/Flag_sudu;	 //===ǰ����־λ��1 
	     
	else if(1==Flag_Hou)		
		Movement=-Target_Velocity/Flag_sudu; //===���˱�־λ��1
	else  
		Movement=0;	
	//=============�ٶ�PI������=======================//	
			Encoder_Least =(Encoder_Left+Encoder_Right)-Movement;       //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ� 
	Encoder *= 0.8;		                                        //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                            //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                 //===���ֳ�λ�� ����ʱ�䣺10ms
			Encoder_Integral=Encoder_Integral-Movement;                 //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>20000)  	
		Encoder_Integral=20000;              	 	 	 	 	//===�����޷�
	if(Encoder_Integral<-20000)	
		Encoder_Integral=-20000;               	 	 	 	 	//===�����޷�	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;  //===�ٶȿ���	
	if(Turn_Off(Angle_Balance)==1||Flag_Stop==1)   
		Encoder_Integral=0;       	 	 	 	 	 	 	 	//===����رպ��������
	return Velocity;
}

/**************************************************************************
�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	float Turn_Amplitude=88/Flag_sudu,Kp=0,Kd=0;     
	//=============ң��������ת����=======================//
	if(Flag_Left==1||Flag_Right==1||Flag_Left==2||Flag_Right==2)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
	{
		if(Flag_Left==1||Flag_Right==1)
			Kp=6;
		else 
			Kp=2;
		
		if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
		Turn_Convert=50/Encoder_temp;
		if(Turn_Convert<0.6)
			Turn_Convert=0.6;
		if(Turn_Convert>3)
			Turn_Convert=3;
	}	
	else
	{
		Turn_Convert=0.9;
		Turn_Count=0;
		Encoder_temp=0;
	}	
	
	if(Flag_Left==1||Flag_Left==2)	           
		Turn_Target-=Turn_Convert;
	else if(Flag_Right==1||Flag_Right==2)	     
		Turn_Target+=Turn_Convert; 
	else 
		Turn_Target=0;
	
	if(Turn_Target>Turn_Amplitude)  
		Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	if(Turn_Target<-Turn_Amplitude) 
		Turn_Target=-Turn_Amplitude;
	
	if(Flag_Left==1||Flag_Right==1||Flag_Left==2||Flag_Right==2) 		
		Kd=0;	   
	else if(Flag_Qian==1||Flag_Hou==1)
		Kd=1.2;
	else 
		Kd=0.4;   //ת���ʱ��ȡ�������ǵľ��� 
	
	if(Flag_Left==1||Flag_Left==2)	           
		Turn=-Turn_Target*Kp -gyro*Kd; //===���Z�������ǽ���PD����
	else if(Flag_Right==1||Flag_Right==2)	     
		Turn=-Turn_Target*Kp -gyro*Kd;
	else 
		Turn=Turn_Target*Kp +gyro*Kd;
	
	return Turn;
}



/**************************************************************************
�������ܣ�����ת�ٸ�ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pnumber(void)
{	
	int Amplitude=3600;    //==������3000
	if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;	
	
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0)		AIN1=1, 	AIN2=0;			//L
	else 	        AIN1=0,	 	AIN2=1;
	
	PWMA=myabs(moto1)+280;
 	//printf("PWMA: %d 	 	\r",PWMA); //������

	if(moto2<0)	 	BIN1=0,		BIN2=1; 	 	//R
	else         	BIN1=1,		BIN2=0;
	
	PWMB=myabs(moto2)+200;	
 	//printf("PWMB: %d 	 	\r\n",PWMB); //������
}


/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float Angle)
{
	u8 temp;
	if(Angle<-40||Angle>40||1==Flag_Stop) 
	{	                                                 //===��Ǵ���40�ȹرյ��
		temp=1;                                            //===Flag_Stop��1�رյ��
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
    }
	else
		temp=0;
    return temp;			
}
	
/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(void)
{ 
	Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
	Angle_Balance=Pitch-ZHONGZHI;             //===����ƽ�����
	
	Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
	Gyro_Turn=gyro[2];               //===����ת����ٶ�
	Acceleration_Z=accel[2];         //===����Z����ٶȼ�
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //��һ��
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //����1��С���ӽ���ֹ
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //����ڶ���
	 {
		    if(++count1>200)       count1=0,flag=0;                                 //��ʱ���ٵȴ�2000ms
	      if(Acceleration>26000&&(Angle>(-20+ZHONGZHI))&&(Angle<(20+ZHONGZHI)))   //����2��С������0�ȸ���������
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //������
	 {
		  if(++count2>100)       count2=0,flag=0;                                   //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>135)                                 //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                               //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
�������ܣ����С���Ƿ񱻷���
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                           //��ֹ���      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+ZHONGZHI)&&Angle<(10+ZHONGZHI)&&encoder_left==0&&encoder_right==0)         //����1��С������0�ȸ�����
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<60&&encoder_right<60)                //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				flag=0;
				return 1;                                             //��⵽С��������
			}
	 }
	return 0;
}



