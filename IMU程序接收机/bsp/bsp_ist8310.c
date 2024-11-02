/**
	* @file bsp_ist8310.c
	* @version 1.0
	* @date 2019.11.29
  *
  * @brief  磁力计IST8310驱动
  *
  *	@author YY
  *
  */
	
#include "bsp_ist8310.h"

/**********************************************************************************************************
*函 数 名: IST8310_Init
*功能说明: IST8310寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
bool IST8310_Init(void)
{
	if(IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_WHOAMI) != 0x10)	//校验磁力计工作状态
		return false;
	else
	{
		IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL2,0x04);	//关闭数据就绪中断
		HAL_Delay(5);
		IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);	//设置单次测量模式
		HAL_Delay(5);
	}
	return true;
}

/**********************************************************************************************************
*函 数 名: IST8310_original_data_read
*功能说明: IST8310原始数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_original_data_read(void)
{
	uint8_t mag_rec_buff[6];
	
	IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);
	delay_us(2);
	
	mag_rec_buff[0] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HX_L);
	mag_rec_buff[1] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HX_H);
	imu_org_data.Mag.X = (uint16_t)mag_rec_buff[1]<<8 | mag_rec_buff[0];
	
	mag_rec_buff[2] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HY_L);
	mag_rec_buff[3] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HY_H);
	imu_org_data.Mag.Y = (uint16_t)mag_rec_buff[3]<<8 | mag_rec_buff[2];
	
	mag_rec_buff[4] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HZ_L);
	mag_rec_buff[5] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HZ_H);
	imu_org_data.Mag.Z = (uint16_t)mag_rec_buff[5]<<8 | mag_rec_buff[4];
	
}



