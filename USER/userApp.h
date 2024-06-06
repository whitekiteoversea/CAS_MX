#ifndef USERAPP_H
#define USERAPP_H

#include "stm32f4xx_hal.h"
#include "global_data.h"
#include "usart.h"
#include "sys.h"
#include "lcd.h"
#include "lcd_init.h"
#include "spi.h"
#include "socket.h"	
#include "string.h"
#undef RTR
#include "can.h"
#include "pid.h"
//#include "oled.h"
//#include "bmp.h"

#include "canfestival_timer.h"
#include "canfestival_can.h"
#include "canfestival.h"
#include "canfestival_master.h"
#include "timers.h"

#include "rs485.h"
#include "sdram.h"
#include "24cxx.h"

#define SOCK_TCPS                (0)
#define DATA_BUF_SIZE           (2048)

#define POSI_CHECK_PERIOD_1MS    (20)
#define MODBUS_INTERNAL_1MS      (10)

#define MAX_ALLOWED_SPEED_RPM   (1000)
#define MIN_ALLOWED_SPEED_RPM   (-1000) 

#define RPM2Vol_CONVERSE_COFF   (10.922)
#define SPEEDGIVEN_INDEX        (0x60FF)  


// function Switch

#define HAL_W5500_ENABLE         			 (0)

#define HAL_CANOPEN_ENABLE                   (1)
#define CAN2_SENDTEST_ON         			 (0)
#define CANOPEN_NONBLOACK_DELAY_ENABLE       (0)

#define HAL_SDRAM_SELFTEST       			 (0)
#define HAL_DAC_ENABLE           			 (0)
#define HAL_EEPROM_ENABLE        			 (0)


extern uint8_t gDATABUF[DATA_BUF_SIZE];  

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// toolbox function def
void network_register(void);
void network_init(void);			// Initialize Network information and display it

void systemParaInit(void);
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode); // can recv info distribute
void canOpen_Init(void);
void canopen_send_sdo(uint16_t message_sdo[]);
uint8_t canOpenSDOConfig(void);
// CANOpen TPDO Callback
// void tpdoCallback(Co_Data*d, UNS8 NodeID);
// void registerTPDOCallback(CO_Data* d);  


int32_t avgErrCollect(uint8_t node, int32_t sampleData);  
int32_t avgErrUpdate(int32_t *sampleData);

void fsmc_sdram_test(void); // SDRAM R/W TEST

uint32_t tim3_getCurrentTimeCnt(void);
// MS level nonblocking delay
uint8_t tim3_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);
void w5500_stateMachineTask(void);

uint32_t tim4_getCurrentTimeCnt(void);
uint8_t tim4_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);

extern uint16_t message_sdo[MAX_PRESET_SDO_NUM][9];

#endif


