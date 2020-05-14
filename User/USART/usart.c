#include "sys.h"
#include "usart.h"

extern u8 Flag_Show;
u16 r_data;
u8 usart_flag,zf_flag;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	if(Flag_Show==1)
	{	
		while((USART2->SR&0X40)==0);//Flag_Show=0  使用串口3   
		USART2->DR = (u8) ch;      
	}
	else
	{	
		while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   
		USART1->DR = (u8) ch;      
	}	
	return ch;
}
#endif 

 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStr;
	USART_InitTypeDef USART_InitStr;
	NVIC_InitTypeDef NVIC_InitStr;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
	GPIO_InitStr.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStr.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStr);//初始化GPIOA.9
   
	//USART1_RX	  GPIOA.10初始化
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStr);//初始化GPIOA.10  

    //Usart1 NVIC 配置
    NVIC_InitStr.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStr.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStr.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStr);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStr.USART_BaudRate = bound;
	USART_InitStr.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStr.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStr.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStr); //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE); 	//使能串口1 
	
}	
void USART1_SendByte(u8 dat)
{
	while( USART_GetFlagStatus(USART1,USART_FLAG_TXE)!= SET); 	
	USART_SendData(USART1, dat); 	 	 	 	 	 	 // 存放数据
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  //发送结束
}
void USART1_SendString(char *str) 	 	// 发送字符串
{
	while(*str)
	{
		USART1_SendByte(*(str++));
	}
	
}
void _uart_print_dec(u16 dat) 	// 打印十进制数
{
	USART1_SendByte((dat/10000) + '0'); 	 
	USART1_SendByte((dat%10000/1000) + '0'); 		
	USART1_SendByte((dat%10000%1000/100) + '0'); 
	USART1_SendByte((dat%10000%1000%100/10) + '0'); 
	USART1_SendByte((dat%10000%1000%100%10) + '0'); 
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if((USART_RX_STA&0x8000)==0)//接收未完成
	{
		if(USART_RX_STA&0x4000)//接收到了0x0d
		{
			if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
			else USART_RX_STA|=0x8000;	//接收完成了 
				
		}
		else //还没收到0X0D
		{	
			if(Res==0x0d)USART_RX_STA|=0x4000;
			else
			 {
				USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
				USART_RX_STA++;
				if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
			}		 
		}
	}   		 
    

} 
#endif	

