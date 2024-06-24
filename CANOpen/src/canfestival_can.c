#include "stm32f4xx_hal.h"
#include "canfestival_can.h"
#include "canfestival.h"
#include "usart.h"

/* CANOPEN字典 */
extern CO_Data masterObjdict_Data;

uint8_t canSend(CAN_PORT notused, Message *message)
{
	uint32_t TransmitMailbox = 0;
	uint8_t len = message->len;
	uint8_t offset = 0;
	uint8_t ret = 0;
	CAN_TxHeaderTypeDef Header;

	Header.IDE = CAN_ID_STD;  					// 标准帧还是扩展帧
	Header.DLC = len;							/* 数据长度 */
	Header.StdId = message->cob_id;			    /* 标识符 */
	Header.ExtId = 0x0;									
  	Header.TransmitGlobalTime = DISABLE;
	Header.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;	/* 数据帧 or 远程帧*/
	
	ret = HAL_CAN_AddTxMessage(&hcan2, &Header, message->data, &TransmitMailbox);
	if (ret != HAL_OK) {
		printf ("CANOpen: CAN2 Send Fail! ErrorCode is 0x%x\n\r", ret);
	}
	return ret;
}

unsigned char isValidAddress(void *ptr) {
		uint32_t address = (uint32_t)ptr;
    if ((address >= SRAMADDR_START && address <= SRAMADDR_END) || (address >= FLASHADDR_START && address <= FLASHADDR_END)) {
        return 1; // 有效地址
    }
    return 0; // 无效地址
}

