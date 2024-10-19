#ifndef __FILTER_H
#define __FILTER_H
/*----------------------------------------------------------------------------------------------------------------------/
        *               本程序只供购买者学习使用，版权著作权属于无名科创团队，
        *               无名科创团队将飞控程序源码提供给购买者，
        *               购买者要为无名科创团队提供保护，
        *               未经作者许可，不得将源代码提供给他人
        *               不得将源代码放到网上供他人免费下载，
        *               更不能以此销售牟利，如发现上述行为，
        *               无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
	*		无名科创开源飞控 V1.1	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrun无名小哥
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2017/10/30
        *               版本：V1.1
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学无名科创团队 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/

#include "stdint.h"
#include "math.h"
#include "bsp_bmi088.h"
#include "BMI088_Read.h"
#include "Calibration.h"

#define Imu_Sampling_Freq  1000	
#define N2 		3	

typedef struct
{
	float Input_Butter[3];
	float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
	float a[3];
	float b[3];
}Butter_Parameter;

extern float Data_X_MAG[N2];
extern float Data_Y_MAG[N2];
extern float Data_Z_MAG[N2];
extern Butter_Parameter Bandstop_Filter_Parameter_30_98;
extern Butter_Parameter Bandstop_Filter_Parameter_30_94;


void 	BMI088_Filter(void);
void 	IST8310_Filter(void);
void 	Butterworth_Parameter_Init(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void 	Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
float GildeAverageValueFilter_MAG(float NewValue,float *Data);

#endif

