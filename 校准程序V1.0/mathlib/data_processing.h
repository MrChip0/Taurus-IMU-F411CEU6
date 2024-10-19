#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "stm32f4xx_hal.h" 
//#include "FreeRTOS.h"

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
float invSqrt(float x);

#endif

