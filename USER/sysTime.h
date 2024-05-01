#ifndef SYSTIME_H
#define SYSTIME_H

#include "sys.h"
#include "time.h"

//ϵͳʱ���¼
extern volatile unsigned int systemPaceCnt;        // ����ʱ��
extern volatile unsigned int g_latestSystemCntMS;  // ���һ���յ�ʱ��ͬ������ʱ����ı���systick����
  
extern volatile uint32_t timeSpecStamp; 
extern volatile uint32_t g_latest_timeStamp;       //���һ��ʱ��ͬ���ı���ʱ���
extern volatile uint32_t g_latest_UTCTime;         //���һ����ͬ��ʱ��UTCʱ���
extern volatile unsigned short frameCnt;           //����֡��

extern volatile unsigned int systemPaceCal;     	//ϵͳ��ʱ����ۼ�ֵ
extern uint8_t timeCnt;					 		//���ŷ���ͨ��ʱ���0-99
extern uint8_t g_timeCnt_60s;    					//60s�������

#endif
