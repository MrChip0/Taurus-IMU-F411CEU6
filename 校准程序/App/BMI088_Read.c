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
uint8_t palstance_tx_data[12];	// [0,3]:X����ٶ� ��[4,7]:Y����ٶ� ��[8,11]:Z����ٶ�
uint8_t accelerat_tx_data[12];	// [0,3]:X����ٶ� ��[4,7]:Y����ٶ� ��[8,11]:Z����ٶ�
float gyro_z[500];
float* YawData;
float Kalman2yaw;
float test_yaw;
float JY901_yaw;

/**
  * @brief BMI_Read
  * @param 
  * @attention  
  * @note  
  */
void BMI_Read(void)
{

		BMI088_original_data_read();		//ԭʼ���ݶ�ȡ
		BMI088_temp_data_read();				 //BMI088�������¶ȴ���
		BMI088_Filter();								//�˲�����
        
		Float2Byte(&imu_output_data.Gyro.X,palstance_tx_data,0);	//ת��Ϊ�ɴ��������
		Float2Byte(&imu_output_data.Gyro.Y,palstance_tx_data,4);
		Float2Byte(&imu_output_data.Gyro.Z,palstance_tx_data,8);



}

