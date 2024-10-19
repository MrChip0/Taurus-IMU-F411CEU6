/**
	* @file bsp_delay.c
	* @version 1.0
	* @date 2019.11.28
  *
  * @brief  �û��Զ�����ʱ����
  *
  *	@author YY
  *
  */
	
#include "bsp_delay.h"
	
void delay_us(uint16_t nus)
{
	uint16_t differ=0xffff-nus-5;	//�趨��ʱ����������ʼֵ

	__HAL_TIM_SET_COUNTER(&htim3,differ);

	HAL_TIM_Base_Start(&htim10);		

  while(differ<0xffff-6)		
  {
    differ=__HAL_TIM_GET_COUNTER(&htim10);
  }
	
  HAL_TIM_Base_Stop(&htim10);
	
}



#include "stm32f4xx_hal.h"  // �������оƬ�ͺŸ��Ĵ�ͷ�ļ�

// ����һ���ṹ�����ڹ����ӳ�״̬
typedef struct {
    uint32_t start_time;    // ��¼�ӳٵĿ�ʼʱ��
    uint32_t delay_duration; // ��¼��Ҫ���ӳ�ʱ��
    uint8_t is_active;      // �ӳ��Ƿ����ڽ���
} NonBlockingDelay;

// ��ʼ���������ӳ�
void NonBlockingDelay_Start(NonBlockingDelay *delay, uint32_t duration)
{
    delay->start_time = HAL_GetTick();  // ��ȡ��ǰʱ��
    delay->delay_duration = duration;   // ��¼��Ҫ���ӳ�ʱ��
    delay->is_active = 1;               // ��־λ����Ϊ�����ڽ��С�
}

// ����ӳ��Ƿ��Ѿ����
uint8_t NonBlockingDelay_IsFinished(NonBlockingDelay *delay)
{
    if (delay->is_active && (HAL_GetTick() - delay->start_time >= delay->delay_duration)) {
        delay->is_active = 0;  // �ӳ���ɺ󣬱�־λ����Ϊ��δ���
        return 1;  // ����1��ʾ�ӳ����
    }
    return 0;  // ����0��ʾ�ӳ�δ���
}


