#include "sysTime.h"

//ϵͳʱ���¼
volatile unsigned int systemPaceCnt = 0;        // ����ʱ��
volatile unsigned int g_latestSystemCntMS = 0;  // ���һ���յ�ʱ��ͬ������ʱ����ı���systick����
  
volatile uint32_t timeSpecStamp = 0; 
volatile uint32_t g_latest_timeStamp = 0;     	// ���һ��ʱ��ͬ���ı���ʱ���
volatile uint32_t g_latest_UTCTime = 0;;        // ���һ��ͬ��ʱ��UTCʱ���
volatile unsigned short frameCnt;               // ����֡��

//��¼�������
volatile unsigned int systemPaceCal = 0;     	//ϵͳ��ʱ����ۼ�ֵ
uint8_t timeCnt = 0;					 		//���ŷ���ͨ��ʱ���0-99
uint8_t g_timeCnt_60s = 0;    					//60s�������
