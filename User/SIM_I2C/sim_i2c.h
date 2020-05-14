/*
*********************************************************************************************************
*
*	模块名称 : I2C总线驱动模块
*	文件名称 : sim_i2c.h
*	版    本 : V2.0
*	说    明 : 头文件。
*********************************************************************************************************
*/

#ifndef __I2C_H
#define __I2C_H
#include "sys.h"

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

/* 定义I2C总线连接的GPIO端口, 可任意改变SCL和SDA的引脚 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */

#define PORT_I2C_SCL	GPIOB			/* GPIO端口 */
#define PIN_I2C_SCL		GPIO_Pin_6	/* GPIO引脚 */

#define PORT_I2C_SDA	GPIOB			/* GPIO端口 */
#define PIN_I2C_SDA		GPIO_Pin_7		/* GPIO引脚 */

#define I2C_SCL_PIN		GPIO_Pin_6 		/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* 连接到SDA数据线的GPIO */



/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  PORT_I2C_SCL->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  PORT_I2C_SCL->BRR = I2C_SCL_PIN				/* SCL = 0 */

#define I2C_SDA_1()  PORT_I2C_SDA->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  PORT_I2C_SDA->BRR = I2C_SDA_PIN				/* SDA = 0 */

#define I2C_SDA_READ()  ((PORT_I2C_SDA->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#define I2C_SCL_READ()  ((PORT_I2C_SCL->IDR & I2C_SCL_PIN) != 0)	/* 读SCL口线状态 */

void SIM_I2C_Init(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(u8 _ucByte); // 发送一个字节
u8 i2c_ReceiveByte(void); 	 	// 接收一个字节
u8 Slave_ACK(void); 	// I2C总线写应答子函数，由slave发出
void Mster_ACK(void); 	// I2C总线读应答子函数，应答由Master发出
void Mster_NOACK(void); // I2C总线无应答子函数 ，由Master发出
u8 i2c_CheckDevice(u8 _Address);

void I2C_WriteByte(u8 address,u8 data_value);
u8 I2C_ReadByte(u8 address);  

u8 I2C_WriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 I2C_ReadBytes(u8 dev, u8 reg, u8 length, u8 *data);


#endif






