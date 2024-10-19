/** 
  * @file     bsp_uart.c
  * @version  v2.0
  * @date     2019.11.18
	*
  * @brief    串口驱动
	*
  *	@author   YY
  *
  */
	
#include "bsp_uart.h"

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/**
* @brief 使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
// void user_uart_init()
// {
// 	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
// 	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
// 	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);
// }


/**
* @brief 串口空闲中断
* @param UART_HandleTypeDef *huart
* @retval 无
*/
// void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
// {
// 	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
// 	{
// 		__HAL_UART_CLEAR_IDLEFLAG(huart);                     	//清除空闲中断标志（否则会一直不断进入中断）
// 		HAL_UART_DMAStop(huart);
// 		USAR_UART_IDLECallback(huart);                       	  //调用中断处理函数
// 	}
// }


// /**
// * @brief 串口空闲中断回调函数
// * @param 串口句柄
// * @retval 无
// */
// void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
// {
// 	if(huart->Instance== USART1 )
// 	{
// 		JY901_original_data_read(JY901_receive_buff); 			//JY901串口读取原始数据
// 	}
// 	memset(JY901_receive_buff,0,sizeof(JY901_receive_buff));                    //清零接收缓冲区
// 	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);  //重启开始DMA传输
// }

int ii = 0;
void Uart2Printf(const char* format, ...) 
{
   char buffer[256];  // 用来存储格式化后的字符串，大小可以根据需要调???
    if(ii>50)
    {
       va_list args;
       va_start(args, format);
       vsnprintf(buffer, sizeof(buffer), format, args);  // 格式化字符串
       va_end(args);
       HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
       ii=0;
    }
}


#define PRINTF_BUFFER_SIZE 256
// 全局变量
char dma_printf_buffer[PRINTF_BUFFER_SIZE];
volatile uint8_t dma_busy = 0;
void Uart2_Dma_Printf(const char *format, ...)
{
    if(ii>5)
    {
    if (dma_busy)
        return;
    dma_busy = 1;
    va_list args;
    va_start(args, format);
    vsnprintf(dma_printf_buffer, PRINTF_BUFFER_SIZE, format, args);
    va_end(args);
    // 启动DMA发送
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dma_printf_buffer, strlen(dma_printf_buffer)) != HAL_OK)
    {
        dma_busy = 0;
    }
           ii=0;
    }
}
