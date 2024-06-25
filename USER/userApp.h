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

#include "canfestival_timer.h"
#include "canfestival_can.h"
#include "canfestival.h"
#include "canfestival_master.h"
#include "timers.h"

#include "rs485.h"
#include "sdram.h"
#include "24cxx.h"

#include "pid.h"

// PRESET DEF
#define SOCK_TCPS                (0)
#define DATA_BUF_SIZE           (2048)

#define POSI_CHECK_PERIOD_1MS    (20)
#define MODBUS_INTERNAL_1MS      (10)

#define MAX_ALLOWED_SPEED_RPM   (1000)
#define MIN_ALLOWED_SPEED_RPM   (-1000) 

#define RPM2Vol_CONVERSE_COFF   (10.922)
#define SPEEDGIVEN_INDEX        (0x60FF)  

#define DesignedTorqueNM        (2.80)

#define MOTOR_ENCODER_IDENTIFYWIDTH  (8388608)  // 23bit


// function Switch
#define HAL_W5500_ENABLE         			 (1)
#define HAL_CANOPEN_ENABLE                   (1)
#define CAN2_StateMachine_Trans_ENABLE       (1)
#define CANOPEN_NONBLOACK_DELAY_ENABLE       (1)
#define HAL_SDRAM_SELFTEST       			 (1)
#define HAL_DAC_ENABLE           			 (0)
#define HAL_EEPROM_ENABLE        			 (1)
#define HAL_LCD_ENABLE                       (0)  // CANOpen与LCD相冲

extern uint8_t gDATABUF[DATA_BUF_SIZE];  
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// toolbox function def
void network_register(void);
void network_init(void);			// Initialize Network information and display it
uint8_t w5500_Decoder(EthControlFrameSingleCAS frame);
uint32_t w5500_reportStatus(CASREPORTFRAME statusPack);

void systemParaInit(void);
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode); // can recv info distribute
void canOpenInit(void);
uint8_t canopen_send_sdo(uint16_t *message_sdo);
uint8_t canOpenSDOConfig(void);
uint8_t canOpenSDOSendWithDelay(CO_Data *d, uint8_t slaveNodeId, uint16_t sdoIndex, uint8_t subIndex, uint8_t sendNum, uint8_t sendType, uint32_t *sendContext); 

int32_t avgErrCollect(uint8_t node, int32_t sampleData);  
int32_t avgErrUpdate(int32_t *sampleData);

void fsmc_sdram_test(void); // SDRAM R/W TEST
uint32_t tim3_getCurrentTimeCnt(void);
// MS level nonblocking delay
uint8_t tim3_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);
void w5500_stateMachineTask(void);

uint32_t tim4_getCurrentTimeCnt(void);
uint8_t tim4_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);

// SpeedSend ToolBox function
uint8_t canopenDriverSpeedGive(short speedCmdRpm);
uint8_t DACDriverSpeedGive(short speedCmdRpm);

#endif



