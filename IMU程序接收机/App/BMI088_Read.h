#ifndef __BMI088_READ_H
#define __BMI088_READ_H

#include "stm32f4xx_hal.h" 
//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include "bsp_spi.h"
#include "bsp_bmi088.h"
#include "Filter.h"
#include "data_processing.h"
#include "usart.h"
#include "bsp_can.h"
#include "Comm_Task.h"
#include "Status_Task.h"

//extern osThreadId CAN_MSG_SENDHandle;
	
extern float gyro_z[500];
void BMI_Read(void);


#endif


