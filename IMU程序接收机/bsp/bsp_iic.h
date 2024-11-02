#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f4xx_hal.h" 
//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include "BMI088_Read.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_delay.h"
#include <stdbool.h>

//IO方向设置
#define SDA_IN()  {GPIOA->MODER&=~(3<<(10*2));GPIOA->MODER|=0<<10*2;}	//PA10输入模式
#define SDA_OUT() {GPIOA->MODER&=~(3<<(10*2));GPIOA->MODER|=1<<10*2;} //PA10输出模式
//IO操作
#define IIC_SCL(a)   	(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,a)) 	//SCL	(GPIO_PIN_RESET/GPIO_PIN_SET)
#define IIC_SDA(b)   	(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,b)) 	//SDA	(GPIO_PIN_RESET/GPIO_PIN_SET)
#define READ_SDA  		(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10))  		//输入SDA


void IIC_Soft_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(void);
bool IIC_Reg_Write(uint8_t SlaveAddress,uint8_t Reg_Address,uint8_t Reg_data);
uint8_t IIC_Reg_Read(uint8_t SlaveAddress,uint8_t Reg_Address);

#endif



