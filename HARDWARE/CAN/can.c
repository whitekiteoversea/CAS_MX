#include "can.h"
#include "led.h"
#include "usart.h"

CAN_ID_Union CAN1RecvFrame;
CAN_ID_Union CAN2RecvFrame;
uint8_t CAN1_RecData[8] = {0};
uint8_t CAN2_RecData[8] = {0}; 

uint8_t HAL_CAN_Ext_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Ext_ID)
{
    uint32_t txmailbox = 0;
    uint32_t offset = 0;
    CAN_TxHeaderTypeDef hdr;

    hdr.IDE = CAN_ID_EXT;													
    hdr.RTR = CAN_RTR_DATA;													
    hdr.StdId = 0x00;														
    hdr.ExtId = Ext_ID;									
    hdr.TransmitGlobalTime = DISABLE;

    while (len != 0)
    {
        hdr.DLC = len > 8 ? 8 : len;			// 数据长度
		if (HAL_CAN_AddTxMessage(hcan, &hdr, ((uint8_t *)buf) + offset, &txmailbox) != HAL_OK) {
			return 1;
		}
        offset += hdr.DLC;
        len -= hdr.DLC;
    }
    return 0;
}

uint8_t HAL_CAN_Std_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Std_ID)
{
    uint32_t txmailbox = 0;
    uint32_t offset = 0;
    CAN_TxHeaderTypeDef hdr;

    hdr.IDE = CAN_ID_STD;													
    hdr.RTR = CAN_RTR_DATA;													
    hdr.StdId = Std_ID;														
    hdr.ExtId = 0x0;									
    hdr.TransmitGlobalTime = DISABLE;

    while (len != 0)
    {
        hdr.DLC = len > 8 ? 8 : len;		
		if (HAL_CAN_AddTxMessage(hcan, &hdr, ((uint8_t *)buf) + offset, &txmailbox) != HAL_OK) {
			return 1;
		}
		offset += hdr.DLC;
        len -= hdr.DLC;
    }
    return 0;
}

