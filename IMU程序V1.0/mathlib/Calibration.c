/**
	* @file Calibration.c
	* @version 1.0
	* @date 2019.12.1
  *
  * @brief  零偏校准函数
  *
  *	@author YY(Part of codes reference 无名创新)
  *
  */

#include "Calibration.h"
#include "pid.h"
#include "bsp_bmi088.h"
#include "bsp_flash.h"
#include "bsp_imu.h"
#include <stdint.h>
#include <string.h>

#define ABS(x)		((x>0)? (x): (-x))
IMU_FLOAT_DATA_T Bias;	//零偏值
float AccRatioOffset;

void Flash_Write_Float1(uint32_t address,float floatDataToWrite) 
{
    // float floatDataToWrite = 123.456f; // 要写入的float数据
    uint32_t dataToWrite = *(uint32_t *)&floatDataToWrite;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t sectorError = 0;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInitStruct.Sector = FLASH_SECTOR_7;  // 擦除第6扇区
    eraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
        // 擦除失败处理
        // Error_Handler();
    }

    // 3. 写入Flash数据
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, dataToWrite) != HAL_OK) {
        // 写入失败处理
        // Error_Handler();
    }
    HAL_FLASH_Lock();
}

#define GYRO_SAMPLES 20000
#define REMOVE_COUNT 100

void InsertMin(float *min_values, float value) {
    for(int i = REMOVE_COUNT - 1; i >=0; i--) {
        if(value < min_values[i]) {
            if(i < REMOVE_COUNT -1) 
            {
                min_values[i+1] = min_values[i];
            }
            min_values[i] = value;
        } else {
            break;
        }
    }
}

void InsertMax(float *max_values, float value) {
    for(int i = REMOVE_COUNT - 1; i >=0; i--) {
        if(value > max_values[i]) {
            if(i < REMOVE_COUNT -1) {
                max_values[i+1] = max_values[i];
            }
            max_values[i] = value;
        } else {
            break;
        }
    }
}

void BMI088_Calibration_Write(void)
{

    float sum_X = 0, sum_Y = 0, sum_Z = 0;
    float min_X[REMOVE_COUNT], min_Y[REMOVE_COUNT], min_Z[REMOVE_COUNT];
    float max_X[REMOVE_COUNT], max_Y[REMOVE_COUNT], max_Z[REMOVE_COUNT];
    uint16_t i;
    for(i = 0; i < REMOVE_COUNT; i++) {
        min_X[i] = 1e6; min_Y[i] = 1e6; min_Z[i] = 1e6;
        max_X[i] = -1e6; max_Y[i] = -1e6; max_Z[i] = -1e6;
    }
    BMI088_temp_data_read();
    Bias.init_tempture = Temperature;
    for(i = 0; i < GYRO_SAMPLES; i++)
    {
        BMI088_original_data_read();  
        BMI088_temp_data_read();
        BMI088_Filter();      
        sum_X += imu_output_data.Gyro.X;
        sum_Y += imu_output_data.Gyro.Y;
        sum_Z += imu_output_data.Gyro.Z;
        InsertMin(min_X, imu_output_data.Gyro.X);
        InsertMin(min_Y, imu_output_data.Gyro.Y);
        InsertMin(min_Z, imu_output_data.Gyro.Z);
        InsertMax(max_X, imu_output_data.Gyro.X);
        InsertMax(max_Y, imu_output_data.Gyro.Y);
        InsertMax(max_Z, imu_output_data.Gyro.Z);
    }
    float sum_min_X = 0, sum_min_Y = 0, sum_min_Z = 0;
    for(i = 0; i < REMOVE_COUNT; i++) {
        sum_min_X += min_X[i];
        sum_min_Y += min_Y[i];
        sum_min_Z += min_Z[i];
    }
    float sum_max_X = 0, sum_max_Y = 0, sum_max_Z = 0;
    for(i = 0; i < REMOVE_COUNT; i++) {
        sum_max_X += max_X[i];
        sum_max_Y += max_Y[i];
        sum_max_Z += max_Z[i];
    }
    float adjusted_sum_X = sum_X - sum_min_X - sum_max_X;
    float adjusted_sum_Y = sum_Y - sum_min_Y - sum_max_Y;
    float adjusted_sum_Z = sum_Z - sum_min_Z - sum_max_Z;
    Bias.Gyro.X = adjusted_sum_X / (GYRO_SAMPLES - 2 * REMOVE_COUNT);
    Bias.Gyro.Y = adjusted_sum_Y / (GYRO_SAMPLES - 2 * REMOVE_COUNT);
    Bias.Gyro.Z = adjusted_sum_Z / (GYRO_SAMPLES - 2 * REMOVE_COUNT);
    HAL_Delay(50);
    Flash_Write_Float1(BIAS_GYRO_X_ADDRESS, Bias.Gyro.X);
    Flash_Write_Float1(BIAS_GYRO_Y_ADDRESS, Bias.Gyro.Y);
    Flash_Write_Float1(BIAS_GYRO_Z_ADDRESS, Bias.Gyro.Z);
    HAL_Delay(50);
}

float Flash_Read_BmiVal(uint32_t address)
{
    uint32_t data = *(__IO uint32_t*)address;
    float result = *(float*)&data;
    return result;
}

void BMI088_Calibration_Read(void)
{
    Bias.Gyro.X = Flash_Read_BmiVal(BIAS_GYRO_X_ADDRESS);
    Bias.Gyro.Y = Flash_Read_BmiVal(BIAS_GYRO_Y_ADDRESS);
    Bias.Gyro.init_z = Flash_Read_BmiVal(BIAS_GYRO_Z_ADDRESS);
}



