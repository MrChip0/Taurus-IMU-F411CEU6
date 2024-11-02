/**
 * @file     bsp_remote_ET08.c
 * @version  v1.0
 * @date     2024/4/16
 *
 * @brief    天地飞遥控器数据解算函数
 * @note     接USART1   Baud Rate：100000 bits/s
 *
 *	@author   佚名搬运工 & LZY
 *
 **/

#include "bsp_remote_ET08.h"
#include "usart.h"
#include <math.h>

SBUS_Buffer sbus_buffer;
SUBS_Vaule remote_vaule;
uint8_t SBUS_RXBuffer[25]; // SBUS 数据包大小为 25 字节

void sbus_callback_handler(SBUS_Buffer *SBUS, uint8_t *SBUS_RXBuffer)
{
    SBUS->Start = SBUS_RXBuffer[0];
    SBUS->Ch3 = ((uint16_t)((SBUS_RXBuffer[3] & 0xc0) >> 6)) | ((((uint16_t)SBUS_RXBuffer[4]) << 2)) | (((uint16_t)(SBUS_RXBuffer[5] & 0x01)) << 10);
    if (SBUS->Ch3 == 1) // 遥控器关机后，状态维持原来
    {
        SBUS->Ch3 = 1024;
        return;
    }
    SBUS->Ch1 = ((uint16_t)SBUS_RXBuffer[1]) | ((uint16_t)((SBUS_RXBuffer[2] & 0x07) << 8));
    SBUS->Ch2 = ((uint16_t)((SBUS_RXBuffer[2] & 0xf8) >> 3)) | (((uint16_t)(SBUS_RXBuffer[3] & 0x3f)) << 6) / 2;
    SBUS->Ch4 = ((uint16_t)((SBUS_RXBuffer[5] & 0xfe) >> 1)) | (((uint16_t)(SBUS_RXBuffer[6] & 0x0f)) << 7);
    SBUS->Ch5 = ((uint16_t)((SBUS_RXBuffer[6] & 0xf0) >> 4)) | (((uint16_t)(SBUS_RXBuffer[7] & 0x7f)) << 4);
    SBUS->Ch6 = ((uint16_t)((SBUS_RXBuffer[7] & 0x80) >> 7)) | (((uint16_t)SBUS_RXBuffer[8]) << 1) | (((uint16_t)(SBUS_RXBuffer[9] & 0x03)) << 9);
    SBUS->Ch7 = ((uint16_t)((SBUS_RXBuffer[9] & 0xfc) >> 2)) | (((uint16_t)(SBUS_RXBuffer[10] & 0x1f)) << 6);
    SBUS->Ch8 = ((uint16_t)((SBUS_RXBuffer[10] & 0xe0) >> 5)) | (((uint16_t)(SBUS_RXBuffer[11])) << 3);
    // SBUS->Ch9=((uint16_t)SBUS_RXBuffer[12])|(((uint16_t)(SBUS_RXBuffer[13]&0x07))<<8);
    // SBUS->Ch10=((uint16_t)((SBUS_RXBuffer[13]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[14]&0x3f))<<5);
    // SBUS->Ch11=((uint16_t)((SBUS_RXBuffer[14]&0xc0)>>6))|(((uint16_t)SBUS_RXBuffer[15])<<2)|(((uint16_t)(SBUS_RXBuffer[16]&0x01))<<10);
    // SBUS->Ch12=((uint16_t)((SBUS_RXBuffer[16]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[17]&0x0f))<<7);
    // SBUS->Ch13=((uint16_t)((SBUS_RXBuffer[17]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[18]&0x7f))<<4);
    // SBUS->Ch14=((uint16_t)((SBUS_RXBuffer[18]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[19])<<1)|(((uint16_t)(SBUS_RXBuffer[20]&0x03))<<9);
    // SBUS->Ch15=((uint16_t)((SBUS_RXBuffer[20]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[21]&0x1f))<<6);
    // SBUS->Ch16=((uint16_t)((SBUS_RXBuffer[21]&0xe0)>>5))|(((uint16_t)SBUS_RXBuffer[22])<<3);
    SBUS->Flag = SBUS_RXBuffer[23];
    SBUS->End = SBUS_RXBuffer[24];
}

int round_to_int(double value)
{
    return (int)(value >= 0 ? value + 0.5 : value - 0.5);
}
float normalize_value(float value, float old_min, float old_max, float new_min, float new_max)
{
    return (-(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min)) + new_max + new_min);
}

void Sbus_Task(void)
{
    remote_vaule.LX = sbus_buffer.Ch3 - 1024;
    remote_vaule.LY = -(sbus_buffer.Ch4 - 1024);
    remote_vaule.RX = sbus_buffer.Ch1 - 1024;
    remote_vaule.RY = -(sbus_buffer.Ch2 - 1024);
    remote_vaule.SB = round_to_int(normalize_value(sbus_buffer.Ch6, 353, 1694, 1, 3)); // 三挡1 2 3 
    remote_vaule.SC = round_to_int(normalize_value(sbus_buffer.Ch7, 353, 1694, 1, 3)); // 三挡1 2 3 
    remote_vaule.SD = round_to_int(normalize_value(sbus_buffer.Ch8, 353, 1694, 1, 2)); // 三挡1 2 3 
}
