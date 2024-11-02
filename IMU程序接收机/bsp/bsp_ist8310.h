#ifndef __BSP_IST8310_H
#define __BSP_IST8310_H

#include "stm32f4xx_hal.h" 
//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include "BMI088_Read.h"
#include "IST8310_Read.h"
#include "usart.h"
#include "bsp_iic.h"
#include "bsp_bmi088.h"
#include <stdbool.h>

// 定义IST8310内部地址
/********************************MAG******************************/
#define IST8310_ADDRESS                 0x1C

#define IST8310_REG_WHOAMI              0x00
#define IST8310_REG_HX_L                0x03
#define IST8310_REG_HX_H                0x04
#define IST8310_REG_HY_L                0x05
#define IST8310_REG_HY_H                0x06
#define IST8310_REG_HZ_L                0x07
#define IST8310_REG_HZ_H                0x08
#define IST8310_REG_CNTRL1              0x0A
#define IST8310_REG_CNTRL2              0x0B
#define IST8310_REG_TEMP_L							0x1C
#define IST8310_REG_TEMP_H							0x1D


bool IST8310_Init(void);
void IST8310_original_data_read(void);


#endif




