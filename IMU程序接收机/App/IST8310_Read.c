/**
	* @file IST8310_Read.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  IST8310数据更新任务
  *
  *	@author YY
  *
  */

#include "IST8310_Read.h"

uint8_t magnetism_tx_data[12];	// [0,3]:X轴磁力计 ；[4,7]:Y轴磁力计 ；[8,11]:Z轴磁力计


/**
  * @brief IST_Read
  * @param 
  * @attention  
  * @note  
  */
void IST_Read(void)
{
	//uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		IST8310_original_data_read();		 //磁力计原始数据读取
		IST8310_Filter();								 //滤波处理
//		osDelayUntil(&mode_wake_time,5); //频率200HZ
	}

}

