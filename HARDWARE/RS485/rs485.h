#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "global_data.h"	 								  

#define	GPIO_SPI_RS485_RE_SET  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define	GPIO_SPI_RS485_RE_RESET  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

#define USART_REC_LEN  	64  		
extern uint8_t USART_RX_BUF[USART_REC_LEN];
extern uint8_t USART_IT_BUF[USART_REC_LEN]; // TEMP

extern UART_HandleTypeDef huart6;


#define HAL_MODBUS_ENABLE (1)

#if !HAL_MODBUS_ENABLE

//485 ���ߺ���
uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen); //LRC
void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len);    // RS485 ASCII �·���ѯ
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen);  // Modbus ASCIIУ�� 

#endif

u32 g_RS485_recvDataDeal(void);  								// ����Modbus RTU���մ���
void g_RS485_sendPacket(UART_HandleTypeDef *husart, uint8_t packType, uint8_t *data);
u16 g_RS485_CRC16Test(u8 *data, u8 num);					 	// Modbus RTU CRCУ��

void HAL_RS485_Send_Data(UART_HandleTypeDef *husart, u8 *buf, u8 len); 
void UART_Byte_Receive(UART_HandleTypeDef *huart); 

#endif	   
















