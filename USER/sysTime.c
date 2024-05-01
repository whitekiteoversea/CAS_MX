#include "sysTime.h"

//系统时间记录
volatile unsigned int systemPaceCnt = 0;        // 本地时间
volatile unsigned int g_latestSystemCntMS = 0;  // 最后一次收到时间同步报文时留存的本地systick计数
  
volatile uint32_t timeSpecStamp = 0; 
volatile uint32_t g_latest_timeStamp = 0;     	// 最近一次时间同步的本地时间戳
volatile uint32_t g_latest_UTCTime = 0;;        // 最近一次同步时的UTC时间戳
volatile unsigned short frameCnt;               // 接收帧号

//记录溢出次数
volatile unsigned int systemPaceCal = 0;     	//系统计时溢出累计值
uint8_t timeCnt = 0;					 		//与伺服器通信时间戳0-99
uint8_t g_timeCnt_60s = 0;    					//60s溢出计数
