/**
	* @file data_processing.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  一些数据处理函数
  *
  *	@author YY
  *
  */
	
#include "data_processing.h"


/***************************************************************/
/*
 * 函数名：Float2Byte
 * 描述  ：将单精度浮点数据转成4字节数据并存入指定地址
 * 输入  ：target:目标单精度数据
					 buf:待写入数组
					 beg:指定从数组第几个元素开始写入
 * 输出  ：无
 */ 
/***************************************************************/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
// Fast inverse square-root
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


