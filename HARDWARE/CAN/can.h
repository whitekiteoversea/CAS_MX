#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
#include "stm32f4xx_hal.h"
#include "global_data.h"

// PC CAN Node ID
#define PCNODEID                        (0)
#define CASCANID                        (1)

#define MASTERCANID                     (1)
#define SLAVECANID                      (8)

#define TEST_CAN_STABLITY               (0)

#define CAN1_FILTER_MODE_MASK_ENABLE    (1)
#define CAN2_FILTER_MODE_MASK_ENABLE    (1)
                 
/* CAN Frame Type 0x00-0x1F total 32 Type*/

#define CANTargetCmd                    (1)
#define CANSpeedPreCmd                  (2)
#define CANOperationModeCmd             (3)
#define CANTimeSyncCmd                  (4)
#define CANPisiAcquireCmd               (5)
#define CANTimeSyncErrorCalCmd          (6)
#define CANLocalPITestCmd               (7)

#define CANDriverInfoAcquire            (10)

uint8_t HAL_CAN_Std_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Std_ID);
uint8_t HAL_CAN_Ext_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Ext_ID);

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAN_ID_Union CAN1RecvFrame;
extern uint8_t CAN1_RecData[8];

#endif

















