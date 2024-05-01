#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "sysTime.h"	 								  

typedef struct recvStorage
{
	uint32_t recvTimeStamp;			//���յ�ʱ��ʱ���
	uint32_t feedbackPosi;  		//�ŷ�������λ��
}cmdRecvData;	

//��������������ѯλ��ֵ
typedef struct dataStorage
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//�·������ٶ�
}comSndRequest;	

extern u8 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
extern u8 RS485_RX_CNT;   			//���յ������ݳ���

//����봮���жϽ��գ�����EN_USART2_RXΪ1����������Ϊ0
#define EN_USART2_RX 			1			//0,������;1,����.

#define USART_REC_LEN  	54  		//�����������ֽ��� 54
#define EN_USART1_RX 			0			//ʹ�ܣ�1��/��ֹ��0������1����

extern uint8_t USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint8_t comRecvCnt;      				//��ǰ����������־

//MODBUS RTU
extern uint8_t g_RTU_Startflag;   //RTU 10ms��ʱ��ʼ
extern uint8_t g_RTU_RcvFinishedflag; //RTU����һ֡����
extern uint8_t g_10ms_Cnt;   //10ms�����ʱ

extern cmdRecvData curPosiTime;                   	    // �洢�ŷ������صķ����ٶȼ�����ʱ��

uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen); //LRC

//485������						
void uart2_RS485_init(u32 pclk2,u32 bound); 

void RS485_Init(u32 pclk2,u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 *len);	
void RS485_TX_Set(u8 en);

//485 ���ߺ���

void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len);    // RS485 ASCII �·���ѯ
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen);  // Modbus ASCIIУ�� 

u32 g_RS485_recvDataDeal(void);  								// ����Modbus RTU���մ���
void g_RS485_sendPacket(uint8_t packType, uint8_t *data); 		// ����Modbus RTU���������·�
u16 g_RS485_CRC16Test(u8 *data, u8 num);					 	// Modbus RTU CRCУ��

#endif	   
















