#ifndef _BSP_REMOTE_ET08_H_
#define _BSP_REMOTE_ET08_H_

#include "main.h"

typedef struct{
	uint8_t   Start;
	uint16_t  Ch1;
	uint16_t  Ch2;
	uint16_t  Ch3;
	uint16_t  Ch4;
	uint16_t  Ch5;
	uint16_t  Ch6;
	uint16_t  Ch7;
	uint16_t  Ch8;
	uint16_t  Ch9;
	uint16_t  Ch10;
	uint16_t  Ch11;
	uint16_t  Ch12;
	uint16_t  Ch13;
	uint16_t  Ch14;
	uint16_t  Ch15;
	uint16_t  Ch16;
	uint8_t   Flag;
	uint8_t   End;
}SBUS_Buffer;

typedef struct{
	float  LX;
	float  LY;
	float  RX;
	float  RY;
	float  SB;
	float  SC;
	float  SD;
}SUBS_Vaule;

extern uint8_t SBUS_RXBuffer[25];  // 声明外部缓冲区变量
extern SBUS_Buffer sbus_buffer;     // 声明外部解算数据结构体
extern SUBS_Vaule remote_vaule;
void sbus_callback_handler(SBUS_Buffer *SBUS, uint8_t *SBUS_RXBuffer);
void Sbus_Task(void);


#endif
