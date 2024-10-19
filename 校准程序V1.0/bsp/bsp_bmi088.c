/**
	* @file bsp_bmi088.c
	* @version 1.0
	* @date 2019.11.10
  *
  * @brief  读取bmi088数据底层函数
  *
  *	@author YY
  *
  */
	
#include "bsp_bmi088.h"
#include "pid.h"
#include "bsp_imu.h"
IMU_INT16_DATA_T imu_org_data; //IMU原始数据
float 	 	 Temperature;	 //IMU温度
float     calibration_temperature = 31;//校准温度
float current;
float err;
/***************************************************************/
/*
 * 函数名：BMI088_Init
 * 描述  ：BMI088初始化函数
 * 输入  ：无
 * 输出  ：0：初始化失败 1：初始化成功
 */ 
/***************************************************************/
uint8_t BMI088_Init(void)
{
init:
    HAL_GPIO_WritePin(ACCEL_SS_GPIO_Port, ACCEL_SS_Pin, GPIO_PIN_SET); // 加计切换至SPI通信
    HAL_Delay(1);
    BMI088_Write_Reg(Accel, ACC_PWR_CTRL, 0x04); // 开启加计
    HAL_Delay(55);
    BMI088_Write_Reg(Accel, ACC_PWR_CONF, 0x00); // 解除加计挂起模式

    uint8_t Gyro_Init_Data[4][2] =
        {
            {GYRO_RANGE, 0x00}, // 量程2000°/s
            //	{GYRO_BANDWIDTH,		0x01}, //设置数据更新频率2000HZ；内部低通滤波器230HZ
            // {GYRO_BANDWIDTH, 0x02}, // 设置数据更新频率1000HZ；内部低通滤波器116HZ
            {GYRO_BANDWIDTH,		0x03}, //设置数据更新频率400HZ；内部低通滤波器47
            {GYRO_INT_CTRL, 0x00}, // 不开启数据更新中断
            //{GYRO_INT_CTRL,			0x08}, //开启数据更新中断
            {INT3_INT4_IO_MAP, 0x00}, // 数据中断引脚无映射
        };
    uint8_t Accel_Init_Data[2][2] =
        {
            //{ACC_CONF,					0x9c}, //设置数据输出速率1600HZ
            //{ACC_CONF,					0x87}, //设置数据输出速率800HZ
            // {ACC_CONF, 0xA7}, // 设置ODR=1.6kHz、连续采样模式、“正常模式”(norm_avg4)

            {ACC_CONF, 0xAA}, // 设置ODR=1.6kHz、连续采样模式、“正常模式”(norm_avg4)
            		// {ACC_RANGE,					0x03}, //设置量程为+-3g；低通滤波器234HZ
            {ACC_RANGE, 0x00}, // 设置量程为+-3g；低通滤波器234HZ

        };
    HAL_Delay(55);

    if (BMI088_Read_Reg(Gyro, GYRO_CHIP_ID) != 0x0F)
    {
        return 2;

    } // 校验陀螺仪通信地址

    else
    {
        for (uint8_t g = 0; g < 4; g++)
        {
            BMI088_Write_Reg(Gyro, Gyro_Init_Data[g][0], Gyro_Init_Data[g][1]);
            HAL_Delay(1);
        }
    }

    if (BMI088_Read_Reg(Accel, ACC_CHIP_ID) != 0x1E) // 校验加速度计通信地址
        return 3;
    else
    {
        for (uint8_t a = 0; a < 2; a++)
        {
            BMI088_Write_Reg(Accel, Accel_Init_Data[a][0], Accel_Init_Data[a][1]);
            HAL_Delay(1);
        }
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 加热电阻pwm初始化
    PID_struct_init(&pid_temperature, POSITION_PID, 999, 999, 180, 40, 18);
    TIM3->CCR4 = pid_calc(&pid_temperature, Temperature, calibration_temperature);

    BMI088_original_data_read();
    if (imu_org_data.Accel.X == 0)
    {
        goto init;
    }
    return 1;
}

/***************************************************************/
/*
 * 函数名：BMI088_original_data_read
 * 描述  ：读取BMI088原始数据
 * 输入  ：无
 * 输出  ：无
 */ 
/***************************************************************/
void BMI088_original_data_read(void)
{
	uint16_t gyro_rec_buff[4];
	uint16_t accel_rec_buff[3];
	
	GYRO_SS(0);
	gyro_rec_buff[0]=SPI_Read_Write_Byte((((uint16_t)RATE_X_LSB)<<8) | 0x80ff);
	for(uint8_t g=1; g<4; g++)
	{
		gyro_rec_buff[g] = SPI_Read_Write_Byte(0xffff);
	}
	GYRO_SS(1);
	
	ACCEL_SS(0);
	SPI_Read_Write_Byte((((uint16_t)ACC_X_LSB)<<8) | 0x80ff);
	for(uint8_t a=0; a<3; a++)
	{
		accel_rec_buff[a] = SPI_Read_Write_Byte(0xffff);
	}
	ACCEL_SS(1);
    
    imu_org_data.Gyro.X = 0;
	imu_org_data.Gyro.X = (gyro_rec_buff[1]&0xff00) | (gyro_rec_buff[0]&0x00ff);
	imu_org_data.Gyro.Y = (gyro_rec_buff[2]&0xff00) | (gyro_rec_buff[1]&0x00ff);
	imu_org_data.Gyro.Z = (gyro_rec_buff[3]&0xff00) | (gyro_rec_buff[2]&0x00ff);
	imu_org_data.Accel.X= (accel_rec_buff[0]<<8) | (accel_rec_buff[0]>>8);
	imu_org_data.Accel.Y= (accel_rec_buff[1]<<8) | (accel_rec_buff[1]>>8);
	imu_org_data.Accel.Z= (accel_rec_buff[2]<<8) | (accel_rec_buff[2]>>8);
	// BMI088_temp_data_read();

    	 
}

/***************************************************************/
/*
 * 函数名：BMI088_temp_data_read
 * 描述  ：读取BMI088温度
 * 输入  ：无
 * 输出  ：无
 */ 
/***************************************************************/
void BMI088_temp_data_read(void)
{
	uint16_t temp_rec_buff, temp_uint11;
	int16_t  temp_int11;
	
	ACCEL_SS(0);
	SPI_Read_Write_Byte((((uint16_t)TEMP_MSB)<<8) | 0x80ff);
	temp_rec_buff = SPI_Read_Write_Byte(0xffff);
	ACCEL_SS(1);

	temp_uint11 = ((temp_rec_buff>>8)*8) + ((temp_rec_buff&0x00ff)/32);
	if(temp_uint11>1023)
		temp_int11 = temp_uint11-2048;
	else
		temp_int11 = temp_uint11;
	Temperature = (temp_int11*0.125)+23;

	   current = pid_calc(&pid_temperature,Temperature,calibration_temperature);
		 if(current<0)
			   current=0;
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,current);
        
    // HAL_GPIO_WritePin(LED1_GPIO_Port,GPIO_PIN_1,SET);
}



