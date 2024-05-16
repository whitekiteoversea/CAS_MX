#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "global_data.h"	 								  

#define	GPIO_SPI_RS485_RE_SET  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define	GPIO_SPI_RS485_RE_RESET  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

#define USART_REC_LEN  	64  		//定义最大接收字节数 64
extern uint8_t USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 

extern UART_HandleTypeDef huart6;

#ifdef HAL_RS485_ENABLE

//485 工具函数
uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen); //LRC
void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len);    // RS485 ASCII 下发查询
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen);  // Modbus ASCII校验 

#endif

u32 g_RS485_recvDataDeal(void);  								// 串口Modbus RTU接收处理
void g_RS485_sendPacket(UART_HandleTypeDef *husart, uint8_t packType, uint8_t *data);
u16 g_RS485_CRC16Test(u8 *data, u8 num);					 	// Modbus RTU CRC校验

void HAL_RS485_Send_Data(UART_HandleTypeDef *husart, u8 *buf, u8 len); 
void USART6_IRQHandler(void);


#endif	   
















