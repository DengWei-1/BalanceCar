#include "sys.h"
#include "usart.h"

extern u8 Flag_Show;
u16 r_data;
u8 usart_flag,zf_flag;

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	if(Flag_Show==1)
	{	
		while((USART2->SR&0X40)==0);//Flag_Show=0  ʹ�ô���3   
		USART2->DR = (u8) ch;      
	}
	else
	{	
		while((USART1->SR&0X40)==0);//Flag_Show!=0  ʹ�ô���1   
		USART1->DR = (u8) ch;      
	}	
	return ch;
}
#endif 

 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStr;
	USART_InitTypeDef USART_InitStr;
	NVIC_InitTypeDef NVIC_InitStr;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
	GPIO_InitStr.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStr.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStr);//��ʼ��GPIOA.9
   
	//USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStr);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStr.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStr.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStr.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStr);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStr.USART_BaudRate = bound;
	USART_InitStr.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStr.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStr.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStr); //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE); 	//ʹ�ܴ���1 
	
}	
void USART1_SendByte(u8 dat)
{
	while( USART_GetFlagStatus(USART1,USART_FLAG_TXE)!= SET); 	
	USART_SendData(USART1, dat); 	 	 	 	 	 	 // �������
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  //���ͽ���
}
void USART1_SendString(char *str) 	 	// �����ַ���
{
	while(*str)
	{
		USART1_SendByte(*(str++));
	}
	
}
void _uart_print_dec(u16 dat) 	// ��ӡʮ������
{
	USART1_SendByte((dat/10000) + '0'); 	 
	USART1_SendByte((dat%10000/1000) + '0'); 		
	USART1_SendByte((dat%10000%1000/100) + '0'); 
	USART1_SendByte((dat%10000%1000%100/10) + '0'); 
	USART1_SendByte((dat%10000%1000%100%10) + '0'); 
}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if((USART_RX_STA&0x8000)==0)//����δ���
	{
		if(USART_RX_STA&0x4000)//���յ���0x0d
		{
			if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
			else USART_RX_STA|=0x8000;	//��������� 
				
		}
		else //��û�յ�0X0D
		{	
			if(Res==0x0d)USART_RX_STA|=0x4000;
			else
			 {
				USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
				USART_RX_STA++;
				if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
			}		 
		}
	}   		 
    

} 
#endif	

