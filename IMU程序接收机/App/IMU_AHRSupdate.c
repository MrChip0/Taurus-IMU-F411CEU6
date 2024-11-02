/** 
  * @file     IMU_AHRSupdate.c
  * @version  v1.0
  * @date     2020.1.10
	*
  * @brief    姿态角更新函数
	*
  *	@author   YY
  *
  */

#include "IMU_AHRSupdate.h"
/**
  * @brief IMU_AHRSupdate_task
  * @param 
  * @attention  
  * @note  
  */
void IMU_AHRSupdate_task(void)
{
    BMI088_original_data_read(); // 原始数据读取
    BMI088_temp_data_read();     // BMI088传感器温度处理
    BMI088_Filter();             // 滤波处理
    IMU_Values_Convert();      //原始数据换算
    IMU_AHRS_Calcu_task();
}

