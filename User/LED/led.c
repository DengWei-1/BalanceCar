#include "led.h"

void LED_Init(void)
{       
	GPIO_InitTypeDef GPIOD_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE); 
													   
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD, &GPIOD_InitStructure);  
		 
	//GPIO_ResetBits(GPIOD, GPIO_Pin_2);         
}



