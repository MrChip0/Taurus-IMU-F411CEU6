/**
 * @file     bsp_uart.c
 * @date     2024.11.1
 *
 * @brief    串口协议
 *
 *	@author   CaoYue
 *
 */

#include "bsp_uart.h"

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_remote_ET08.h"
#include "usart.h"
#include "bsp_imu.h"
#include "QuaternionEKF.h"

// firewater协议VOFA，阻塞式
int timestamp = 0;
void Uart2Printf(const char *format, ...)
{
    char buffer[256]; // 用来存储格式化后的字符串，大小可以根据需要调???
    if (timestamp > 50)
    {
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args); // 格式化字符串
        va_end(args);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        timestamp = 0;
    }
}

// firewater协议VOFA，DMA
#define PRINTF_BUFFER_SIZE 256
#define Print_Fre 0
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


// Justfloat协议VOFA，DMA
#define CHANNEL_MAX 10
void SendSBUSDataViaDMA(void)
{
    static uint8_t buffer[sizeof(float) * CHANNEL_MAX + 4]; // 16 个浮点数 + 4 字节帧尾
    // 打包浮点数据
    static float *Channel = (float *)buffer; // 将 buffer 视为浮点数组
    if (timestamp > Print_Fre)
    {
        Channel[0]  = remote_vaule.LX;//左遥杆左右
        Channel[1]  = remote_vaule.LY;//左遥杆上下
        Channel[2]  = remote_vaule.RX;//右遥杆左右
        Channel[3]  = remote_vaule.RY;//右遥杆上下

        Channel[4]  = remote_vaule.SB;//左上角SB，三档
        Channel[5]  = remote_vaule.SC;//右上角SC，三档
        Channel[6]  = remote_vaule.SD;//右上角SD，两档

        Channel[7]  = imu_real_data.yaw;
        Channel[8]  = imu_real_data.pitch;
        Channel[9]  = imu_real_data.roll;

        // Channel[10] = imu_real_data.Accel.X;
        // Channel[11] = imu_real_data.Accel.Y;
        // Channel[12] = imu_real_data.Accel.Z;
        // Channel[13] = Q3;

        // Channel[14] = (float)sbus_buffer.Ch15;
        // Channel[15] = (float)sbus_buffer.Ch16;
        // 添加帧尾
        buffer[sizeof(float) * CHANNEL_MAX] = 0x00;
        buffer[sizeof(float) * CHANNEL_MAX + 1] = 0x00;
        buffer[sizeof(float) * CHANNEL_MAX + 2] = 0x80;
        buffer[sizeof(float) * CHANNEL_MAX + 3] = 0x7F;

        HAL_UART_Transmit_DMA(&huart2, buffer, sizeof(buffer));
        timestamp = 0;
    }
}


