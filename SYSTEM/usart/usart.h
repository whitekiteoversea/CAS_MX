#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	  

typedef struct
{
	uint8_t identifyHeader_1;  //0xAA
	uint8_t identifyHeader_2;  //0x55
}FrameHeader;

typedef struct
{		
	FrameHeader Header; 
	uint8_t cur_ARMRunningTime_BoardNo; 
	uint8_t dataLen;		
	uint8_t cur_ARMRunningTime_High;  
	uint8_t cur_ARMRunningTime_Middle_1;  
	uint8_t cur_ARMRunningTime_Middle_2;     
	uint8_t cur_ARMRunningTime_Low;    
	uint8_t cur_RunningSpeed_High;
	uint8_t cur_RunningSpeed_Low;
	uint8_t Tailer;
}FeedBackInfoPack;

extern uint8_t UART4_SendBuf[200];   //发送缓冲区
extern FeedBackInfoPack monitorPack;

#define EN_UART4_RX 0 //默认关闭uart4接收中断，只做单向数据采集 
#define RS485_RE PGout(10)	// RS485_RE

void uart_init(u32 pclk2,u32 bound); 
void uart4_init(u32 pclk2,u32 bound);

#endif	   
















