/**
	* @file Filter.c
	* @version 1.0
	* @date 2019.12.1
  *
  * @brief  滤波函数
  *
  *	@author YY/BZW(Part of codes reference 无名创新)
  *
  */

#include "Filter.h"
#include "KalmanFilter.h"
/*******************************************************************************/
//------------------------Butterworth滤波中间数组-----------------------//
Butter_Parameter Gyro_Parameter;
Butter_Parameter Accel_Parameter;
Butter_BufferData gyro_filter_buf_bug[3],gyro_filter_buf[3];
Butter_BufferData accel_filter_buf[3];
//---------------------------IST滤波buffer---------------------------//
float	Data_X_MAG[N2],Data_Y_MAG[N2],Data_Z_MAG[N2];
//----------------------------滤波后数据-----------------------------//
S_FLOAT_XYZ	gyro_filter,accel_filter,mag_filter,gyro_filter_bug;
//----------------------------滤波器参数-----------------------------//
Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  采样-阻带
  1,   0.627040f,  -0.290527f,
  0.354737f,   0.627040f,    0.354737f
};
Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  采样-阻带
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};
/*******************************************************************************/

/***************************************************************/
/*
 * 函数名：BMI088_Filter
 * 描述  ：BMI088滤波函数
 * 输入  ：无
 * 输出  ：无
 */ 
/***************************************************************/
float imu_Gyro_z;
void BMI088_Filter(void)
{
	//原始数据滤波处理	
	gyro_filter_bug.X=LPButterworth(imu_org_data.Gyro.X,&gyro_filter_buf[0],&Bandstop_Filter_Parameter_30_98);
	gyro_filter_bug.Y=LPButterworth(imu_org_data.Gyro.Y,&gyro_filter_buf[1],&Bandstop_Filter_Parameter_30_98);
	gyro_filter_bug.Z=LPButterworth(imu_org_data.Gyro.Z,&gyro_filter_buf[2],&Bandstop_Filter_Parameter_30_98);

	gyro_filter.X=LPButterworth(gyro_filter_bug.X,&gyro_filter_buf_bug[0],&Gyro_Parameter);
	gyro_filter.Y=LPButterworth(gyro_filter_bug.Y,&gyro_filter_buf_bug[1],&Gyro_Parameter);
	gyro_filter.Z=LPButterworth(gyro_filter_bug.Z,&gyro_filter_buf_bug[2],&Gyro_Parameter);		//滤波后Gyro数据
   if(fabs(Temperature-calibration_temperature)>2)
	    Bias.Gyro.Z =Bias.Gyro.init_z +(Temperature-calibration_temperature)*0.015f*16.384f;
	 else
		   Bias.Gyro.Z = Bias.Gyro.init_z;
	Bias.Gyro.Z =Bias.Gyro.init_z;

	imu_output_data.Gyro.X = gyro_filter.X - Bias.Gyro.X;
	imu_output_data.Gyro.Y = gyro_filter.Y - Bias.Gyro.Y;
	imu_output_data.Gyro.Z = gyro_filter.Z - Bias.Gyro.Z;			 //输出值等于滤波后值减去零偏值


    // printf("%7d,%7d,%7d\n", imu_output_data.Gyro.X,imu_output_data.Gyro.Y,imu_output_data.Gyro.Z);

	accel_filter.X=LPButterworth(imu_org_data.Accel.X,&accel_filter_buf[0],&Accel_Parameter);
	accel_filter.Y=LPButterworth(imu_org_data.Accel.Y,&accel_filter_buf[1],&Accel_Parameter);
	accel_filter.Z=LPButterworth(imu_org_data.Accel.Z,&accel_filter_buf[2],&Accel_Parameter);

   	imu_output_data.Accel.X = accel_filter.X;
	imu_output_data.Accel.Y = accel_filter.Y;
	imu_output_data.Accel.Z = accel_filter.Z;	
      
 
}

/***************************************************************/
/*
 * 函数名：IST8310_Filter
 * 描述  ：IST8310滤波函数
 * 输入  ：无
 * 输出  ：无
 */ 
/***************************************************************/
void IST8310_Filter(void)
{
	mag_filter.X=GildeAverageValueFilter_MAG(imu_org_data.Mag.X,Data_X_MAG);
  mag_filter.Y=GildeAverageValueFilter_MAG(imu_org_data.Mag.Y,Data_Y_MAG);
  mag_filter.Z=GildeAverageValueFilter_MAG(imu_org_data.Mag.Z,Data_Z_MAG);
	
	imu_output_data.Mag.X = mag_filter.X - Bias.Mag.X;
	imu_output_data.Mag.Y = mag_filter.Y - Bias.Mag.Y;
	imu_output_data.Mag.Z = mag_filter.Z - Bias.Mag.Z;					 //输出值等于滤波后值减去零偏值
}

/****************************************
Butterworth低通滤波器参数初始化：http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@函数名：Butterworth_Parameter_Init
@入口参数：无
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Butterworth_Parameter_Init(void)
{
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 50, &Gyro_Parameter);//姿态角速度反馈滤波参数  50
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30, &Accel_Parameter);//姿态解算加计修正滤波值 30
}

/*************************************************
函数名:	LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

/***********************************************************
@函数名：Set_Cutoff_Frequency
@入口参数：float 采样频率, float 截止频率,
Butter_Parameter *LPF
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(PI / fr);
  float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

/***********************************************************
@函数名：GildeAverageValueFilter_MAG
@入口参数：float NewValue,float *Data
@出口参数：无
功能描述：滑动窗口滤波
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N2-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N2-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/


