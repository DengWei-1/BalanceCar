#include "usart2.h"	

extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; //蓝牙遥控相关的变量
extern float Show_Data_Mb;  
extern u8 PID_Send;
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki; 	//PID参数 

u8 Usart2_Receive;
u8 mode_data[8];
u8 six_data_stop[3]={0X59,0X59,0X59};  //停止数据样本
u8 six_data_start[3]={0X58,0X58,0X58};  //启动数据样本
 

void uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStrue;
	USART_InitTypeDef USART_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;

//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE); 	错误 这是时钟复位了
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);
	 
	// 外设使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);  //复位串口2 -> 可以没有
	
	//USART2_TX	  GPIOA.2初始化
	GPIO_InitStrue.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStrue.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	//USART2_RX	  GPIOA.3初始化
	GPIO_InitStrue.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStrue);  
	

	USART_InitStrue.USART_BaudRate = bound;//串口波特率
	USART_InitStrue.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStrue.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStrue.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStrue.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStrue.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2,&USART_InitStrue);
			
	USART_Cmd(USART2, ENABLE);  
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开启串口接受中断
	
	NVIC_InitStrue.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority= 0 ;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStrue); 
          
}

void USART2_IRQHandler(void)
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		static	int uart_receive=0;//蓝牙接收相关变量
		static u8 Flag_PID,i,j,APP_Receive[50];
		static float Data;
		
		uart_receive=USART_ReceiveData(USART2); 
		Usart2_Receive=uart_receive;
		if(uart_receive=='Y')  Flag_sudu=2;  //低速挡（默认值）
		if(uart_receive=='X')  Flag_sudu=1;  //高速档
	
		if(uart_receive=='Z')	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
		else if(uart_receive=='A')	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
		else if(uart_receive=='E')	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
		else if(uart_receive=='C')	 	 	//只左
		Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
		else if(uart_receive=='G')	 	 	//只右   
		Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  
		
		else if(uart_receive=='B'||uart_receive=='F')	 //左
		Flag_Left=2,Flag_Right=0;
		else if(uart_receive=='D'||uart_receive=='H')	 //右
		Flag_Left=0,Flag_Right=2; 
		
		else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
		
		
//		if(Usart2_Receive=='{') Flag_PID=1;  //指令起始位
//		if(Usart2_Receive=='}') Flag_PID=2;  //指令停止位
//		if(Flag_PID==1)                      //记录参数
//		{
//			APP_Receive[i]=Usart2_Receive;
//			i++;
//		}

//		if(Flag_PID==2)   //执行
//		{
//			Show_Data_Mb=i;
//			if(APP_Receive[3]==0x50) 	 PID_Send=1;  //获取设备参数
//			else if(APP_Receive[3]==0x57) 	 ;   //掉电保存参数
//			else if(APP_Receive[1]!=0x23)                    //更新PID参数
//			{								
//				for(j=i;j>=4;j--)
//				{
//					Data+=(APP_Receive[j-1]-48)*pow(10,i-j);
//				}
//				switch(APP_Receive[1])
//				{
//					case 0x30:  Balance_Kp=Data/100;break;
//					case 0x31:  Balance_Kd=Data/100;break;
//					case 0x32:  Velocity_Kp=Data/100;break;
//					case 0x33:  Velocity_Ki=Data/100;break;
//					case 0x34:  break;
//					case 0x35:  break;
//					case 0x36:  break;
//					case 0x37:  break;
//					case 0x38:  break;
//				}
//			}				 
//			Flag_PID=0;   //相关标志位清零
//			i=0;
//			j=0;
//			Data=0;
//			memset(APP_Receive, 0, sizeof(u8)*50);
//		} 
	}  							
}


