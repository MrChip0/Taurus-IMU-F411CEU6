/**
	* @file bsp_delay.c
	* @version 1.0
	* @date 2019.11.28
  *
  * @brief  用户自定义延时函数
  *
  *	@author YY
  *
  */
	
#include "bsp_delay.h"
	
void delay_us(uint16_t nus)
{
	uint16_t differ=0xffff-nus-5;	//设定定时器计数器起始值

	__HAL_TIM_SET_COUNTER(&htim3,differ);

	HAL_TIM_Base_Start(&htim10);		

  while(differ<0xffff-6)		
  {
    differ=__HAL_TIM_GET_COUNTER(&htim10);
  }
	
  HAL_TIM_Base_Stop(&htim10);
	
}



#include "stm32f4xx_hal.h"  // 根据你的芯片型号更改此头文件

// 创建一个结构体用于管理延迟状态
typedef struct {
    uint32_t start_time;    // 记录延迟的开始时间
    uint32_t delay_duration; // 记录需要的延迟时间
    uint8_t is_active;      // 延迟是否正在进行
} NonBlockingDelay;

// 初始化非阻塞延迟
void NonBlockingDelay_Start(NonBlockingDelay *delay, uint32_t duration)
{
    delay->start_time = HAL_GetTick();  // 获取当前时间
    delay->delay_duration = duration;   // 记录需要的延迟时间
    delay->is_active = 1;               // 标志位设置为“正在进行”
}

// 检查延迟是否已经完成
uint8_t NonBlockingDelay_IsFinished(NonBlockingDelay *delay)
{
    if (delay->is_active && (HAL_GetTick() - delay->start_time >= delay->delay_duration)) {
        delay->is_active = 0;  // 延迟完成后，标志位设置为“未激活”
        return 1;  // 返回1表示延迟完成
    }
    return 0;  // 返回0表示延迟未完成
}


