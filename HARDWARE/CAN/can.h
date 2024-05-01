#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    

//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE			0		 			//0,不使能;1,使能.	
#define CAN2_RX0_INT_ENABLE			0		 			//0,不使能;1,使能.

#define CAN_BoardCastID 0x3F

// 系统通讯CAN节点数
#define NetNodeNum 		(3)

#pragma pack(1)															
//联合及结构体声明
typedef union
{
	//这里分配位数是按0-11低到高分配的，因此需要注意声明顺序，避免解析错误
	struct 
	{
		uint32_t MasterOrSlave : 1; 	//主从角色
		uint32_t CTRCode : 5; 				
		uint32_t NodeOrGroupID : 5; 		
		uint32_t Reserved : 21;        //保留
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

//存DAC下发控制指令的发送时间，回复到达时间，下发给定速度
typedef struct 
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//下发给定速度
}DACSndStorage;	

#pragma pack()

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//发送数据
u8 CAN1_Msg_Pend(u8 fifox);								//查询邮箱报文
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//接收数据
u8 CAN1_Tx_Staus(u8 mbox);  							//返回发送状态
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据
u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
u8 canSendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len);

u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);
u8 CAN2_Tx_Staus(u8 mbox);
u8 can2SendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len);
u8 CAN2_Receive_Msg(u8 *buf);							//接收数据
u8 CAN2_Msg_Pend(u8 fifox);

extern uint8_t can1RecvFlag;
extern CAN_ID_Union CANRecvFrame;
extern uint8_t CAN1_RecData[8];

extern uint8_t can2RecvFlag;
extern CAN_ID_Union CAN2RecvFrame;
extern uint8_t CAN2_RecData[8];

extern uint8_t canRecvSyncFlag;

extern u8 canlocalCharacNode; //本地扮演的CAN节点角色

#endif

















