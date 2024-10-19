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

extern IMU_FLOAT_DATA_T imu_real_data;

uint8_t angle_tx_data[12];
uint8_t offset_time=0;

float flag_quaternion_intilized;
/**
  * @brief IMU_AHRSupdate_task
  * @param 
  * @attention  
  * @note  
  */
void IMU_AHRSupdate_task(void)
{

	
	 /*原方案
		IMU_Values_Convert(); //原始数据换算
		IMU_AHRS_Calcu();			//姿态角解算
		*/
	//新方案
		IMU_Values_Convert(); //原始数据换算
		IMU_AHRS_Calcu_task();
		// imu_real_data.yaw+= 180.0f;
		// Float2Byte(&imu_real_data.pitch ,angle_tx_data,0);	
		// Float2Byte(&imu_real_data.roll,angle_tx_data,4);
		// Float2Byte(&imu_real_data.yaw  ,angle_tx_data,8); //转化为可传输的数据

        Uart2_Dma_Printf("%f,%f,%f\n",imu_real_data.yaw,imu_real_data.pitch, -imu_real_data.roll);	
}

