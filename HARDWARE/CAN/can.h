#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
#include "stm32f4xx_hal.h"

#define REG_CAN_ENABLE             		(0)

#define CAN1_FILTER_MODE_MASK_ENABLE    (1)
#define CAN2_FILTER_MODE_MASK_ENABLE    (1)
                 
#define PCNODEID                        (0)

/* CAN Frame Type 0x00-0x1F total 32 Type*/

#define CANSpeedCmd                     (1)
#define CANSpeedPreCmd                  (2)
#define CANTimeSyncCmd                  (4)
#define CANPisiAcquireCmd               (5)
#define CANTimeSyncErrorCalCmd          (6)
#define CANLocalPITestCmd               (7)

#define CANDriverInfoAcquire            (10)



// #define CAN1_RX0_INT_ENABLE			1		 			
// #define CAN2_RX0_INT_ENABLE			1		 		

// #define CAN_BoardCastID 0x3F
// #define NetNodeNum 		(3)

#pragma pack(1)															
typedef union
{
	struct 
	{
		uint32_t MasterOrSlave : 1; 	//主发1 从回0
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

typedef struct 
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//
}DACSndStorage;	

#pragma pack()






#if REG_CAN_ENABLE      

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

uint8_t HAL_CAN_Std_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Std_ID);
uint8_t HAL_CAN_Ext_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Ext_ID);

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAN_ID_Union CAN1RecvFrame;
extern CAN_ID_Union CAN2RecvFrame;
extern uint8_t CAN1_RecData[8];
extern uint8_t CAN2_RecData[8]; 



#endif

















