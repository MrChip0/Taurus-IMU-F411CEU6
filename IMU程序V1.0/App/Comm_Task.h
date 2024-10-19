#ifndef __COMM_TASK_H
#define __COMM_TASK_H

#include "stm32f4xx_hal.h" 
//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"
#include "usart.h"
#include "bsp_uart.h"
#include "bsp_bmi088.h"

#define PALSTANCE_MSG_SEND    ( 1 << 7 )
#define ANGLE_MSG_SEND    		( 1 << 6 )

void can_msg_send_task(void);

#endif


