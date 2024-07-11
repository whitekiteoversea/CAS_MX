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

#define MAX_ALLOWED_TORQUE_NM   (2.80)
#define MIN_ALLOWED_TORQUE_NM   (-2.80) 

#define RPM2Vol_CONVERSE_COFF   (10.922)
#define SPEEDGIVEN_INDEX        (0x60FF)  

#define DesignedTorqueNM        (2.80)

#define MOTOR_ENCODER_IDENTIFYWIDTH  (8388608)  // 23bit

#define MAXRECORDALLOWEDLENGTH         (500000)
#define MAXRECORDLENGTH                (800000)


// function Switch
#define HAL_W5500_ENABLE         			 (1)
#define HAL_CANOPEN_ENABLE                   (1)
#define CANOPEN_NONBLOACK_DELAY_ENABLE       (1)
#define HAL_SDRAM_ENABLE       			     (1)
#define HAL_SDRAM_TEST_ENABLE                (0)
#define HAL_DAC_ENABLE           			 (0)
#define HAL_EEPROM_ENABLE        			 (1)
#define HAL_LCD_ENABLE                       (0)  // CANOpen与LCD相冲

extern uint8_t gDATABUF[DATA_BUF_SIZE];  
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern MOTIONRECORD sramArray[MAXRECORDLENGTH]; // 最大数据记录长度 300s

// toolbox function def
void network_register(void);
void network_init(void);			// Initialize Network information and display it
uint8_t w5500_Decoder(EthControlFrameSingleCAS frame);
uint32_t w5500_reportStatus(CASREPORTFRAME statusPack);
uint32_t w5500_sdramDataReportTask(uint32_t reportFrameNum);

void systemParaInit(void);
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode); // can recv info distribute
void canOpenInit(void);
uint8_t canopen_send_sdo(uint16_t *message_sdo);
uint8_t canOpenSDOConfig(void);
uint8_t canOpenSDOSendWithDelay(CO_Data *d, uint8_t slaveNodeId, uint16_t sdoIndex, uint8_t subIndex, uint8_t sendNum, uint8_t sendType, uint32_t *sendContext); 

int32_t avgErrCollect(uint8_t node, int32_t sampleData);  
int32_t avgErrUpdate(int32_t *sampleData);

void fsmc_sdram_test(void); // SDRAM R/W TEST
void sdram_data_reset(void); // 重置SDRAM数据有效性
uint32_t tim3_getCurrentTimeCnt(void);

// MS level nonblocking delay
uint8_t tim3_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);
void w5500_stateMachineTask(void);
uint32_t w5500_sdramDataRequestReport(uint32_t readyReportNum);

uint32_t tim4_getCurrentTimeCnt(void);
uint8_t tim4_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt);

// SpeedSend ToolBox function
uint8_t canopenDriverSpeedGive(short speedCmdRpm);
uint8_t canopenDriverTorqueGive(short torqueCmd);
uint8_t DACDriverSpeedGive(short speedCmdRpm);
uint8_t canopenStateMachine(void);
void canopenStatusMonitor(void);
uint16_t canopenStopMachineAndTransMode(uint8_t targetOperationMode);

uint32_t bissc_processDataAcquire(void);
void bissc_errorRateMonitor(void);
void BISSC_ReStore(uint8_t *errCnt);
uint8_t HAL_BISSC_effectDataAcquire(void);

uint32_t enter_critical(void);
void exit_critical(uint32_t primask);

void sdram_write_recordData(uint32_t frameNum);
void sdram_read_recordData(uint32_t frameNum);
uint32_t bsp_TestExtSDRAM(void);

void set_BASEPRI_REG(uint32_t basePri);
uint32_t  get_BASEPRI_REG(void);

extern volatile uint32_t last_timeMS_upload;

#endif



