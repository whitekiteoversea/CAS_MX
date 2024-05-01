#ifndef SYSTIME_H
#define SYSTIME_H

#include "sys.h"
#include "time.h"

//系统时间记录
extern volatile unsigned int systemPaceCnt;        // 本地时间
extern volatile unsigned int g_latestSystemCntMS;  // 最后一次收到时间同步报文时留存的本地systick计数
  
extern volatile uint32_t timeSpecStamp; 
extern volatile uint32_t g_latest_timeStamp;       //最近一次时间同步的本地时间戳
extern volatile uint32_t g_latest_UTCTime;         //最近一次是同步时的UTC时间戳
extern volatile unsigned short frameCnt;           //接收帧号

extern volatile unsigned int systemPaceCal;     	//系统计时溢出累计值
extern uint8_t timeCnt;					 		//与伺服器通信时间戳0-99
extern uint8_t g_timeCnt_60s;    					//60s溢出计数

#endif
