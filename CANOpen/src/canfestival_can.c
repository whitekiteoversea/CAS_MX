#include "stm32f4xx_hal.h"
#include "canfestival_can.h"
#include "canfestival.h"
#include "usart.h"

/* CANOPEN字典 */
extern CO_Data masterObjdict_Data;

#if CAN2_GPIO_CANOPEN_ENABLE

/* 功能:	GPIO配置
	 参数:	无
	 返回值:无
 */
static void gpio_config(void)
{
	GPIO_InitTypeDef       GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORTA时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //100MHz  GPIO_Speed_100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);               //初始化

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //复用为CAN1
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //复用为CAN1
}

/* 功能:	中断嵌套控制器配置
	 参数:	无
	 返回值:无
 */
static void nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 设置CAN接收中断，先占优先级2，从站优先级2 */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/* 初始化中断 */
	NVIC_Init(&NVIC_InitStructure);
}

/* 功能:	CAN总线过滤器配置
	 参数:	无
	 返回值:无
 */
static void can_filter_config(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	/* 配置过滤器0组，配置成标准标识符且低7位都为0时接受 */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;						/* 设置过滤器组0 */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		/* 屏蔽模式 */
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	/* 32位模式 */
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;					/* 在CANOpen中标准标识符的低7位表示节点ID */
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;					/* 在CANOpen中只用标准标识符，数据帧/远程帧都有 */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;						/* 主节点ID为0 */
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;						/* 标准帧 */
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;	/* 过滤器关联到FIFO0 */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;					/* 使能过滤器 */
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/* 功能:	can总线配置
	 参数:	无
	 返回值:无
 */
void CANOpen_can_config(void)
{
	CAN_InitTypeDef CAN_InitStructure;

	/* 配置IO */
	gpio_config();

	/* 中断嵌套控制器配置 */
	nvic_config();

	/* 配置CAN总线时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN1默认参数 */
	CAN_DeInit(CAN1);

	/* 将结构体填入默认参数 */
	CAN_StructInit(&CAN_InitStructure);

	/* 关闭时间触发模式 */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	/* 关闭自动离线管理 */
	CAN_InitStructure.CAN_ABOM = ENABLE;
	/* 关闭自动唤醒 */
	CAN_InitStructure.CAN_AWUM = ENABLE;
	/* 自动重传 */
	CAN_InitStructure.CAN_NART = DISABLE;
	/* 禁止FIFO溢出时覆盖原报文 */
	CAN_InitStructure.CAN_RFLM = DISABLE;
	/* 关闭优先级取决于ID */
	CAN_InitStructure.CAN_TXFP = DISABLE;
	/* 正常模式 */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* 设置波特率:36MHz/9/(2+1+1)=1mbps */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 9;

	/* 初始化CAN总线 */
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN总线过滤器配置 */
	can_filter_config();

	/* 接收挂起中断 */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* can总线接收中断回调函数 */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg message;
	Message Rx_Message;

	/* 接收消息 */
	CAN_Receive(CAN1, CAN_FIFO0, &message);

	/* 组装canopen数据包 */
	Rx_Message.cob_id = message.StdId;						/* 功能码和节点ID */
	Rx_Message.rtr = (message.RTR == CAN_RTR_DATA) ? 0 : 1;	/* 标识符类型 */
	Rx_Message.len = message.DLC;							/* 数据包长度 */
	memcpy(Rx_Message.data, message.Data, message.DLC);		/* 数据 */

	/* canopen数据包分配处理函数 */
	canDispatch(&masterObjdict_Data, &Rx_Message);
}

/* 功能:	CAN发送数据函数
	 参数:	notused can总线端口
			message canopen数据包
	返回值:	0 成功
			1 失败
 */
uint8_t canSend(CAN_PORT notused, Message *message)
{
	uint32_t i = 0xFFFFFF;
	CanTxMsg TxMessage;
	uint8_t TransmitMailbox = 0;

	/* 组装CAN数据包 */
	TxMessage.DLC = message->len;							/* 数据长度 */
	memcpy(TxMessage.Data, message->data, message->len);	/* 数据 */
	TxMessage.IDE = CAN_ID_STD;								/* 标准ID */
	TxMessage.StdId = message->cob_id;						/* 标识符 */
	TxMessage.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;	/* 数据帧 */

	/* 发送数据包 */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	/* 等待发送成功 */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && --i);

	/* 成功0 超时1 */
	return (i != 0) ? 0 : 1;
}

#endif

uint8_t canSend(CAN_PORT notused, Message *message)
{
	uint32_t TransmitMailbox = 0;
	uint8_t len = message->len;
	uint8_t offset = 0;
	uint8_t ret = 0;
	CAN_TxHeaderTypeDef Header;

	/* 组装CAN数据包 */
	Header.IDE = CAN_ID_STD;
	Header.DLC = len;							/* 数据长度 */
	Header.StdId = message->cob_id;						/* 标识符 */
	Header.ExtId = 0;
	Header.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;	/* 数据帧 */

    while (len != 0)
    {
        Header.DLC = len > 8 ? 8 : len;			// 数据长度
		if (HAL_CAN_AddTxMessage(&hcan2, &Header, message->data + offset, &TransmitMailbox) != HAL_OK) {
			printf(" CAN2 Send Failed! \n\r");
			return 1;
		}
        offset += Header.DLC;
        len -= Header.DLC;
    }
		return ret;
}



