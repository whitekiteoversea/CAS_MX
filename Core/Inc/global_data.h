/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_DATA_H
#define __GLOBAL_DATA_H

#include <stdint.h>

enum TELMODE {
  IDLEMODE = 0,
  CANMODE,
  ETHMODE
};

enum WORKMODE {
		RECVSPEEDMODE = 0,
		PIPOSIMODE,     
		PREPOSIMODE
};

// CANOpen SDO NUM
#define MAX_PRESET_SDO_NUM       (11)

// 全局时间记录
typedef struct {
  volatile unsigned int l_time_cnt_10us;
  volatile unsigned int l_time_ms;
  volatile unsigned int g_time_ms;
} GLOBALTIME;

typedef struct {
  volatile unsigned char comRecvCnt;        // Single time consecutive receive cnt
  volatile uint8_t g_RTU_Startflag;         // RTU 10ms计时开始
  volatile uint8_t g_RTU_RcvFinishedflag;   // RTU接收一帧结束
  volatile uint8_t g_10ms_Cnt;              // 10ms间隔计时

  volatile unsigned int l_init_abs_posi_time; // 初始获得绝对位置本地时间 
  volatile unsigned int init_abs_posi_um;   // 启动初始绝对位置
  volatile unsigned int l_recv_abs_posi_time; // 最新获得绝对位置本地时间 
  volatile unsigned int latest_abs_posi_um;   // 最新绝对位置
} MODBUSVARS;

// Global Status Struct
typedef struct {
  // Time Sync
  volatile unsigned char l_time_overflow;   // 本地计时溢出
  volatile unsigned char l_time_heartbeat;  // 本地计时心跳
  //Work Mode
  enum TELMODE telmode;                     // 当前工作模式
	enum WORKMODE workmode;                   // Algorithm WorkMode: Speed/Torque/Position

  volatile unsigned char l_can1_recv_flag; 
  volatile unsigned char l_can2_send_flag; 

  volatile unsigned int  l_bissc_sensor_acquire; // 获取BISS-C 数据

  // Modbus RTU
  volatile unsigned char l_rs485_getposi_cnt;   // 控制获取最新位置的频率计数
  volatile unsigned char l_rs485_getposiEnable;  // 触发定时获取最新位置  
} GLOBALSTATUS;

// ETH Mode Parameter
typedef struct {
  unsigned char SrcRecvIP[4];
	unsigned short SrcRecvPort;

  unsigned char DstHostIP[4];
	unsigned short DstHostPort;

  // unsigned char *DstHostIP[2][4];
	// unsigned short DstHostPort[2];
} GLOBAL_ETH_UDP_VAR;

typedef struct {
  uint8_t g_posi[5];      // unit depends on BISS-C
	int32_t g_Distance; 	// um
	int16_t g_Speed; 			// rpm
  int16_t g_phaseAmp;   // A

	uint32_t g_InitialPosi; //um

} MOTIONVAR;

typedef struct {
  unsigned char NodeID;
  unsigned char slaveCANID;
  volatile unsigned int canDelayTime_MS[MAX_PRESET_SDO_NUM];

} GLOBAL_CAN_VAR;

typedef struct {
  unsigned int recNum;
  unsigned int CASLocalTime_ms;
  unsigned int CASPosi_um;
} POSI_RECORD_VAR;

typedef struct {
  unsigned int curSTOAddr;
  unsigned char overWriteFlag;
  POSI_RECORD_VAR lastestPosiData;
} SDRAM_STO_VAR;

// extern Var
extern GLOBALTIME gTime;
extern GLOBALSTATUS gStatus;
extern MODBUSVARS modbusPosi;
extern MOTIONVAR motionStatus;
extern GLOBAL_ETH_UDP_VAR w5500_udp_var;
extern GLOBAL_CAN_VAR can_var;
extern SDRAM_STO_VAR sdram_var;

#endif /* __MAIN_H */
