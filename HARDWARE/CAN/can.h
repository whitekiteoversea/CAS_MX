#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
#include "stm32f4xx_hal.h"

// PC CAN Node ID
#define PCNODEID                        (0)
#define CASNODEID                       (1)  // CAN1 Node
#define SLAVECANID                      (8)  // CAN2 Motor Node

#define TEST_CAN_STABLITY               (0)

#define CAN1_FILTER_MODE_MASK_ENABLE    (1)
#define CAN2_FILTER_MODE_MASK_ENABLE    (1)
                 
/* CAN Frame Type 0x00-0x1F total 32 Type*/

#define CANSpeedCmd                     (1)
#define CANSpeedPreCmd                  (2)
#define CANOperationModeCmd             (3)
#define CANTimeSyncCmd                  (4)
#define CANPisiAcquireCmd               (5)
#define CANTimeSyncErrorCalCmd          (6)
#define CANLocalPITestCmd               (7)

#define CANDriverInfoAcquire            (10)

#pragma pack(1)															
typedef union
{
	struct 
	{
		uint32_t MasterOrSlave : 1; 	//主发1 从回0
		uint32_t CTRCode : 5; 				
		uint32_t NodeOrGroupID : 5; 		
		uint32_t Reserved : 21;       
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

uint8_t HAL_CAN_Std_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Std_ID);
uint8_t HAL_CAN_Ext_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Ext_ID);

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAN_ID_Union CAN1RecvFrame;
extern uint8_t CAN1_RecData[8];

#endif

















