#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "bsp_bmi088.h"
#include "Filter.h"


void BMI088_Calibration_Write(void);
void BMI088_Calibration_Read(void);

void Flash_Write_Float1(uint32_t address,float floatDataToWrite);
float Flash_Read_BmiVal(uint32_t address);

#endif


