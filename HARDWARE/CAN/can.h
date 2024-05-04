#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    

//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE			0		 			//0,��ʹ��;1,ʹ��.	
#define CAN2_RX0_INT_ENABLE			0		 			//0,��ʹ��;1,ʹ��.

#define CAN_BoardCastID 0x3F

// ϵͳͨѶCAN�ڵ���
#define NetNodeNum 		(3)

#pragma pack(1)															
//���ϼ��ṹ������
typedef union
{
	//�������λ���ǰ�0-11�͵��߷���ģ������Ҫע������˳�򣬱����������
	struct 
	{
		uint32_t MasterOrSlave : 1; 	//���ӽ�ɫ
		uint32_t CTRCode : 5; 				
		uint32_t NodeOrGroupID : 5; 		
		uint32_t Reserved : 21;        //����
	}CAN_Frame_Union;
	
	uint32_t Value;
}CAN_ID_Union;

//CANFrame
typedef struct
{
    CAN_ID_Union CANID;
    uint8_t CANData[8];
}CANFrame_STD;

#define recvBufLen 3000

//��DAC�·�����ָ��ķ���ʱ�䣬�ظ�����ʱ�䣬�·������ٶ�
typedef struct 
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//�·������ٶ�
}DACSndStorage;	

#pragma pack()

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//��������
u8 CAN1_Msg_Pend(u8 fifox);								//��ѯ���䱨��
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//��������
u8 CAN1_Tx_Staus(u8 mbox);  							//���ط���״̬
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							//��������
u8 canSendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len);

u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);
u8 CAN2_Tx_Staus(u8 mbox);
u8 can2SendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len);
u8 CAN2_Receive_Msg(u8 *buf);							//��������
u8 CAN2_Msg_Pend(u8 fifox);

extern uint8_t can1RecvFlag;
extern CAN_ID_Union CANRecvFrame;
extern uint8_t CAN1_RecData[8];

extern uint8_t can2RecvFlag;
extern CAN_ID_Union CAN2RecvFrame;
extern uint8_t CAN2_RecData[8];

extern uint8_t canRecvSyncFlag;

extern u8 canlocalCharacNode; //���ذ��ݵ�CAN�ڵ��ɫ

#endif

















