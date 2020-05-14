#include "usart2.h"	

extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; //����ң����صı���
extern float Show_Data_Mb;  
extern u8 PID_Send;
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki; 	//PID���� 

u8 Usart2_Receive;
u8 mode_data[8];
u8 six_data_stop[3]={0X59,0X59,0X59};  //ֹͣ��������
u8 six_data_start[3]={0X58,0X58,0X58};  //������������
 

void uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStrue;
	USART_InitTypeDef USART_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;

//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE); 	���� ����ʱ�Ӹ�λ��
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);
	 
	// ����ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);  //��λ����2 -> ����û��
	
	//USART2_TX	  GPIOA.2��ʼ��
	GPIO_InitStrue.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStrue.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	//USART2_RX	  GPIOA.3��ʼ��
	GPIO_InitStrue.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStrue);  
	

	USART_InitStrue.USART_BaudRate = bound;//���ڲ�����
	USART_InitStrue.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStrue.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStrue.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStrue.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStrue.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2,&USART_InitStrue);
			
	USART_Cmd(USART2, ENABLE);  
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//�������ڽ����ж�
	
	NVIC_InitStrue.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority= 0 ;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStrue); 
          
}

void USART2_IRQHandler(void)
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		static	int uart_receive=0;//����������ر���
		static u8 Flag_PID,i,j,APP_Receive[50];
		static float Data;
		
		uart_receive=USART_ReceiveData(USART2); 
		Usart2_Receive=uart_receive;
		if(uart_receive=='Y')  Flag_sudu=2;  //���ٵ���Ĭ��ֵ��
		if(uart_receive=='X')  Flag_sudu=1;  //���ٵ�
	
		if(uart_receive=='Z')	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
		else if(uart_receive=='A')	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
		else if(uart_receive=='E')	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
		else if(uart_receive=='C')	 	 	//ֻ��
		Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
		else if(uart_receive=='G')	 	 	//ֻ��   
		Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  
		
		else if(uart_receive=='B'||uart_receive=='F')	 //��
		Flag_Left=2,Flag_Right=0;
		else if(uart_receive=='D'||uart_receive=='H')	 //��
		Flag_Left=0,Flag_Right=2; 
		
		else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
		
		
//		if(Usart2_Receive=='{') Flag_PID=1;  //ָ����ʼλ
//		if(Usart2_Receive=='}') Flag_PID=2;  //ָ��ֹͣλ
//		if(Flag_PID==1)                      //��¼����
//		{
//			APP_Receive[i]=Usart2_Receive;
//			i++;
//		}

//		if(Flag_PID==2)   //ִ��
//		{
//			Show_Data_Mb=i;
//			if(APP_Receive[3]==0x50) 	 PID_Send=1;  //��ȡ�豸����
//			else if(APP_Receive[3]==0x57) 	 ;   //���籣�����
//			else if(APP_Receive[1]!=0x23)                    //����PID����
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
//			Flag_PID=0;   //��ر�־λ����
//			i=0;
//			j=0;
//			Data=0;
//			memset(APP_Receive, 0, sizeof(u8)*50);
//		} 
	}  							
}


