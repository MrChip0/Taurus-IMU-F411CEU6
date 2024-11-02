/** 
  * @file bsp_JY901.c
  * @version 2.0
  * @date 2020.1.2
	*
  * @brief  JY901数据读取
	*
  *	@author YY
  *
  */

#include "bsp_JY901.h"

imu_typedef JY901_org_data;


/**
  * @brief JY901解算
  * @param 
  * @attention  
  * @note  
  */
void JY901_original_data_read(uint8_t imu_buf[])
{
	if(imu_buf[0]==0x55 && imu_buf[1]==0x53)
	{
		JY901_org_data.roll  = ((imu_buf[3]<<8)|imu_buf[2])/32768.0f*180.0f;
		JY901_org_data.roll = 180 - JY901_org_data.roll;
		JY901_org_data.pitch = ((imu_buf[5]<<8)|imu_buf[4])/32768.0f*180.0f;
		JY901_org_data.yaw   = ((imu_buf[7]<<8)|imu_buf[6])/32768.0f*180.0f;
	}
//	if(imu_buf[0]==0x55 && imu_buf[1]==0x52)
//	{
//	   JY901_org_data.Gyro_x=((imu_buf[3]<<8)|imu_buf[2])/32768.0f*180.0f;
//	   JY901_org_data.Gyro_y=((imu_buf[5]<<8)|imu_buf[4])/32768.0f*180.0f;
//		 JY901_org_data.Gyro_z=((short)(imu_buf[7]<<8)|imu_buf[6])/32768.0f*180.0f;
//		  
//	}
//		if(imu_buf[0+11]==0x55 && imu_buf[1+11]==0x53)
//	{
//		JY901_org_data.roll  = ((imu_buf[3+11]<<8)|imu_buf[2+11])/32768.0f*180.0f;
//		JY901_org_data.pitch = ((imu_buf[5+11]<<8)|imu_buf[4+11])/32768.0f*180.0f;
//		JY901_org_data.yaw   = ((imu_buf[7+11]<<8)|imu_buf[6+11])/32768.0f*180.0f;
//	}
	
}


