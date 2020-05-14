/*
*********************************************************************************************************
*
*	ģ������ : I2C��������ģ��
*	�ļ����� : sim_i2c.h
*	��    �� : V2.0
*	˵    �� : ͷ�ļ���
*********************************************************************************************************
*/

#ifndef __I2C_H
#define __I2C_H
#include "sys.h"

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

/* ����I2C�������ӵ�GPIO�˿�, ������ı�SCL��SDA������ */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */

#define PORT_I2C_SCL	GPIOB			/* GPIO�˿� */
#define PIN_I2C_SCL		GPIO_Pin_6	/* GPIO���� */

#define PORT_I2C_SDA	GPIOB			/* GPIO�˿� */
#define PIN_I2C_SDA		GPIO_Pin_7		/* GPIO���� */

#define I2C_SCL_PIN		GPIO_Pin_6 		/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* ���ӵ�SDA�����ߵ�GPIO */



/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  PORT_I2C_SCL->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  PORT_I2C_SCL->BRR = I2C_SCL_PIN				/* SCL = 0 */

#define I2C_SDA_1()  PORT_I2C_SDA->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  PORT_I2C_SDA->BRR = I2C_SDA_PIN				/* SDA = 0 */

#define I2C_SDA_READ()  ((PORT_I2C_SDA->IDR & I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#define I2C_SCL_READ()  ((PORT_I2C_SCL->IDR & I2C_SCL_PIN) != 0)	/* ��SCL����״̬ */

void SIM_I2C_Init(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(u8 _ucByte); // ����һ���ֽ�
u8 i2c_ReceiveByte(void); 	 	// ����һ���ֽ�
u8 Slave_ACK(void); 	// I2C����дӦ���Ӻ�������slave����
void Mster_ACK(void); 	// I2C���߶�Ӧ���Ӻ�����Ӧ����Master����
void Mster_NOACK(void); // I2C������Ӧ���Ӻ��� ����Master����
u8 i2c_CheckDevice(u8 _Address);

void I2C_WriteByte(u8 address,u8 data_value);
u8 I2C_ReadByte(u8 address);  

u8 I2C_WriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 I2C_ReadBytes(u8 dev, u8 reg, u8 length, u8 *data);


#endif






