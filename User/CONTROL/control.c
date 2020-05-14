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
extern int Temperature,Voltage;                                //电池电压采样相关的变量
extern u8 Flag_Stop,Flag_Show;                 //停止标志位和 显示标志位 默认停止 显示打开
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; //蓝牙遥控相关的变量
extern int Temperature;                            //显示mpu温度
extern int Encoder_Left,Encoder_Right;   
extern u8 delay_50,delay_500,delay_flag,Bi_zhang,PID_Send,Flash_Send;  //延时和调参等变量
extern int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬
extern float Acceleration_Z;  
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki;//PID参数

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target,show1_flag,show2_flag,show3_flag;
int Voltage_Temp,Voltage_Count,Voltage_All;

/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI9_5_IRQHandler(void) 
{    
	
	if(MPU_INT==0)		
	{   
		EXTI->PR=1<<5;                                                      //清除LINE5上的中断标志位   
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			if(++delay_50==10)//给主函数提供50ms的精准延时
				delay_50=0,delay_flag=0;  
			if(++delay_500==100)
			{
				Temperature=Read_Temperature();
				delay_500=0;
				show1_flag=!show1_flag;  
			}                  
		}
		if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		{
			//Get_Angle();                                             	 //===更新姿态			
			Voltage_Temp=Get_battery_volt();		                                //=====读取电池电压		
			Voltage_Count++;                                                    //=====平均值计数器
			Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
			if(Voltage_Count==100) 
				Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0; 	 	//=====求平均值		
			if(Voltage<500)show2_flag=1;
			else show2_flag=0;
			//return 0;	                                               
		}               	                                              	//===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
		Encoder_Left=-Read_Encoder(2);                                      //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		Encoder_Right=Read_Encoder(3);                                      //===读取编码器的值
	 	Get_Angle();                                               //===更新姿态	
		
		Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		Turn_Pwm = turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===转向环PID控制     
		
		Moto1 = Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===计算左轮电机最终PWM
		Moto2 = Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===计算右轮电机最终PWM
		Xianfu_Pnumber();              	 	 	 	 	 	 	 	 	 	  //===PWM限幅
		
//		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//===检查是否小车被那起
//			Flag_Stop=1;	                                                      //===如果被拿起就关闭电机
//		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))              //===检查是否小车被放下
//			Flag_Stop=0;	                                                      //===如果被放下就启动电机
		
		if(Turn_Off(Angle_Balance)==0)                              //===如果不存在异常
		  	Set_Pwm(Moto1,Moto2);
		
	}       	
	return 0;	
	
} 

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle;     	  	 	 	 	 	   //===求出平衡的角度中值 和机械相关
	balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;
	static float Encoder_Integral,Target_Velocity;
	//=============遥控前进后退部分=======================// 
	if(Flag_sudu==1)  Target_Velocity=600;                 
    else 	          Target_Velocity=1200;
	
	if(1==Flag_Qian)	
		Movement=Target_Velocity/Flag_sudu;	 //===前进标志位置1 
	     
	else if(1==Flag_Hou)		
		Movement=-Target_Velocity/Flag_sudu; //===后退标志位置1
	else  
		Movement=0;	
	//=============速度PI控制器=======================//	
			Encoder_Least =(Encoder_Left+Encoder_Right)-Movement;       //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度 
	Encoder *= 0.8;		                                        //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                            //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                 //===积分出位移 积分时间：10ms
			Encoder_Integral=Encoder_Integral-Movement;                 //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>20000)  	
		Encoder_Integral=20000;              	 	 	 	 	//===积分限幅
	if(Encoder_Integral<-20000)	
		Encoder_Integral=-20000;               	 	 	 	 	//===积分限幅	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;  //===速度控制	
	if(Turn_Off(Angle_Balance)==1||Flag_Stop==1)   
		Encoder_Integral=0;       	 	 	 	 	 	 	 	//===电机关闭后清除积分
	return Velocity;
}

/**************************************************************************
函数功能：转向控制  修改转向速度，请修改Turn_Amplitude即可
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	float Turn_Amplitude=88/Flag_sudu,Kp=0,Kd=0;     
	//=============遥控左右旋转部分=======================//
	if(Flag_Left==1||Flag_Right==1||Flag_Left==2||Flag_Right==2)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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
		Turn_Target=Turn_Amplitude;    //===转向速度限幅
	if(Turn_Target<-Turn_Amplitude) 
		Turn_Target=-Turn_Amplitude;
	
	if(Flag_Left==1||Flag_Right==1||Flag_Left==2||Flag_Right==2) 		
		Kd=0;	   
	else if(Flag_Qian==1||Flag_Hou==1)
		Kd=1.2;
	else 
		Kd=0.4;   //转向的时候取消陀螺仪的纠正 
	
	if(Flag_Left==1||Flag_Left==2)	           
		Turn=-Turn_Target*Kp -gyro*Kd; //===结合Z轴陀螺仪进行PD控制
	else if(Flag_Right==1||Flag_Right==2)	     
		Turn=-Turn_Target*Kp -gyro*Kd;
	else 
		Turn=Turn_Target*Kp +gyro*Kd;
	
	return Turn;
}



/**************************************************************************
函数功能：限制转速赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pnumber(void)
{	
	int Amplitude=3600;    //==限制在3000
	if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;	
	
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0)		AIN1=1, 	AIN2=0;			//L
	else 	        AIN1=0,	 	AIN2=1;
	
	PWMA=myabs(moto1)+280;
 	//printf("PWMA: %d 	 	\r",PWMA); //俯仰角

	if(moto2<0)	 	BIN1=0,		BIN2=1; 	 	//R
	else         	BIN1=1,		BIN2=0;
	
	PWMB=myabs(moto2)+200;	
 	//printf("PWMB: %d 	 	\r\n",PWMB); //俯仰角
}


/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float Angle)
{
	u8 temp;
	if(Angle<-40||Angle>40||1==Flag_Stop) 
	{	                                                 //===倾角大于40度关闭电机
		temp=1;                                            //===Flag_Stop置1关闭电机
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
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(void)
{ 
	Read_DMP();                      //===读取加速度、角速度、倾角
	Angle_Balance=Pitch-ZHONGZHI;             //===更新平衡倾角
	
	Gyro_Balance=gyro[1];            //===更新平衡角速度
	Gyro_Turn=gyro[2];               //===更新转向角速度
	Acceleration_Z=accel[2];         //===更新Z轴加速度计
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //第一步
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //条件1，小车接近静止
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //进入第二步
	 {
		    if(++count1>200)       count1=0,flag=0;                                 //超时不再等待2000ms
	      if(Acceleration>26000&&(Angle>(-20+ZHONGZHI))&&(Angle<(20+ZHONGZHI)))   //条件2，小车是在0度附近被拿起
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                                   //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>135)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                               //检测到小车被拿起
			}
	 }
	return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                           //防止误检      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+ZHONGZHI)&&Angle<(10+ZHONGZHI)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<60&&encoder_right<60)                //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                                             //检测到小车被放下
			}
	 }
	return 0;
}



