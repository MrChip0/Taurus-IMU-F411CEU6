/**
 * @file BMI088_Read.c
 * @version 1.0
 * @date 2019.11.26
 *
 * @brief  BMI088���ݸ�������
 *
 *	@author YY
 *
 */

#include "BMI088_Read.h"
#include "bsp_JY901.h"
#include "KalmanFilter.h"
#include "pid.h"
#include "bsp_bmi088.h"
#include "tim.h"
IMU_FLOAT_DATA_T imu_output_data;
float gyro_z[500];



/**
 * @brief BMI_Read
 * @param
 * @attention
 * @note
 */
void BMI_Read(void)
{
    BMI088_original_data_read(); // ԭʼ���ݶ�ȡ
    BMI088_temp_data_read();     // BMI088�������¶ȴ���
    BMI088_Filter();             // �˲�����
}
