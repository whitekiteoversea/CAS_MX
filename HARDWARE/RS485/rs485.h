#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "sysTime.h"	 								  

typedef struct recvStorage
{
	uint32_t recvTimeStamp;			//接收到时的时间戳
	uint32_t feedbackPosi;  		//伺服器反馈位移
}cmdRecvData;	

//串口向下主动查询位移值
typedef struct dataStorage
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//下发给定速度
}comSndRequest;	

extern u8 RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
extern u8 RS485_RX_CNT;   			//接收到的数据长度

//如果想串口中断接收，设置EN_USART2_RX为1，否则设置为0
#define EN_USART2_RX 			1			//0,不接收;1,接收.

#define USART_REC_LEN  	54  		//定义最大接收字节数 54
#define EN_USART1_RX 			0			//使能（1）/禁止（0）串口1接收

extern uint8_t USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint8_t comRecvCnt;      				//当前接收数量标志

//MODBUS RTU
extern uint8_t g_RTU_Startflag;   //RTU 10ms计时开始
extern uint8_t g_RTU_RcvFinishedflag; //RTU接收一帧结束
extern uint8_t g_10ms_Cnt;   //10ms间隔计时

extern cmdRecvData curPosiTime;                   	    // 存储伺服器返回的反馈速度及接收时间

uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen); //LRC

//485处理函数						
void uart2_RS485_init(u32 pclk2,u32 bound); 

void RS485_Init(u32 pclk2,u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 *len);	
void RS485_TX_Set(u8 en);

//485 工具函数

void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len);    // RS485 ASCII 下发查询
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen);  // Modbus ASCII校验 

u32 g_RS485_recvDataDeal(void);  								// 串口Modbus RTU接收处理
void g_RS485_sendPacket(uint8_t packType, uint8_t *data); 		// 串口Modbus RTU控制命令下发
u16 g_RS485_CRC16Test(u8 *data, u8 num);					 	// Modbus RTU CRC校验

#endif	   
















