#ifndef __USART_H
#define __USART_H
#include "sys.h"

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

void uart_init(u32 bound);
void USART1_SendByte(u8 dat);
void USART1_SendString(char *str); 
void _uart_print_dec(u16 dat);	// ��ӡʮ������
	

#endif

