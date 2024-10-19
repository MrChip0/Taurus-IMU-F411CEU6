/**
 * @file     bsp_uart.c
 * @version  v2.0
 * @date     2019.11.18
 *
 * @brief    ��������
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
 * @brief ʹ�ܴ��ڿ����ж�,��������DMA����
 * @param  ��
 * @retval ��
 */
// void user_uart_init()
// {
// 	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
// 	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

// 	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);
// }

/**
 * @brief ���ڿ����ж�
 * @param UART_HandleTypeDef *huart
 * @retval ��
 */
// void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
// {
// 	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
// 	{
// 		__HAL_UART_CLEAR_IDLEFLAG(huart);                     	//��������жϱ�־�������һֱ���Ͻ����жϣ�
// 		HAL_UART_DMAStop(huart);
// 		USAR_UART_IDLECallback(huart);                       	  //�����жϴ�����
// 	}
// }

// /**
// * @brief ���ڿ����жϻص�����
// * @param ���ھ��
// * @retval ��
// */
// void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
// {
// 	if(huart->Instance== USART1 )
// 	{
// 		JY901_original_data_read(JY901_receive_buff); 			//JY901���ڶ�ȡԭʼ����
// 	}
// 	memset(JY901_receive_buff,0,sizeof(JY901_receive_buff));                    //������ջ�����
// 	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);  //������ʼDMA����
// }

int timestamp = 0;
void Uart2Printf(const char *format, ...)
{
    char buffer[256]; // �����洢��ʽ������ַ�������С���Ը�����Ҫ��???
    if (timestamp > 50)
    {
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args); // ��ʽ���ַ���
        va_end(args);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        timestamp = 0;
    }
}

#define PRINTF_BUFFER_SIZE 256
#define Print_Fre 20
char dma_printf_buffer[PRINTF_BUFFER_SIZE];
volatile uint8_t dma_busy = 0;

void Uart2_Dma_Printf(const char *format, ...)
{
    if (timestamp > Print_Fre)
    {
        if (dma_busy)
            return;
        dma_busy = 1;
        va_list args;
        va_start(args, format);
        vsnprintf(dma_printf_buffer, PRINTF_BUFFER_SIZE, format, args);
        va_end(args);
        if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_printf_buffer, strlen(dma_printf_buffer)) != HAL_OK)
        {
            dma_busy = 0;
        }
        timestamp = 0;
    }
}
