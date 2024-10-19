#ifndef BSH_FLASH_H
#define BSH_FLASH_H
#include "main.h"
#define BIAS_GYRO_X_ADDRESS 0x08067890
#define BIAS_GYRO_Y_ADDRESS 0x0806889c
#define BIAS_GYRO_Z_ADDRESS 0x080698A8


float Flash_Read_Float(uint32_t address);
void Flash_Write_Float(uint32_t address, float data);
#endif
