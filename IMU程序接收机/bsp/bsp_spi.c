/**
	* @file bsp_spi.c
	* @version 1.0
	* @date 2019.11.9
  *
  * @brief  SPI读写驱动
  *
  *	@author YY
  *
  */

#include "bsp_spi.h"


/*
 * 函数名：BMI088_Write_Reg
 * 描述  ：SPI写入寄存器
 * 输入  ：retyp:写Gyro/Accel的寄存器 ；
					 reg:指定的寄存器地址；
					 value：写入的值
 * 输出  ：无
 */ 
void BMI088_Write_Reg(reg_type_e regtyp, uint8_t regaddr,uint8_t value)
{
	regaddr &= 0x7f;	//寄存器地址+写命令
	regtyp==Gyro?GYRO_SS(0):ACCEL_SS(0);
	SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | (uint16_t)value);
	regtyp==Gyro?GYRO_SS(1):ACCEL_SS(1);
}


/*
 * 函数名：BMI088_Read_Reg
 * 描述  ：SPI读取寄存器
*  输入  ：retyp:读取Gyro/Accel的寄存器
					 regaddr:指定的寄存器地址
 * 输出  ：reg_val：reg寄存器地址对应的值
 */ 
uint8_t BMI088_Read_Reg(reg_type_e regtyp, uint8_t regaddr)
{
	uint8_t  reg_val;
	regaddr |= 0x80;	//寄存器地址+读命令
	switch(regtyp)
	{
		case Gyro:	
		{	
			GYRO_SS(0);																												
			reg_val=SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | 0x00ff);																
			GYRO_SS(1);																													
			break;
		}
		case Accel:
		{
			ACCEL_SS(0);
			SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | 0x00ff);
			reg_val=SPI_Read_Write_Byte(0xffff)>>8;
			ACCEL_SS(1);
			break;
		}
	}
	return(reg_val);
}


/*
 * 函数名：SPI_Read_Write_Byte
 * 描述  ：读写一个字节
 * 输入  ：TxData:要写入的字节
 * 输出  ：读取到的字节
 */ 
uint16_t SPI_Read_Write_Byte(uint16_t TxData)
{		
	uint16_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&TxData,(uint8_t*)&Rxdata,1, 1000);       
	return Rxdata;  				    
}



