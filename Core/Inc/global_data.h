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

// 电机工作模式
enum WORKMODE {
		IDLE = 0,
		PIPOSIMODE = 1,     
    FAULTCHECKMODE = 2,
		RECVSPEEDMODE = 3,
    TORQUEMODE = 4
};

// 全局时间记录
typedef struct {
  volatile unsigned int l_time_cnt_10us;
  volatile unsigned int l_time_ms;
  volatile unsigned int g_time_ms;
} GLOBALTIME;

// SpeedMode
typedef union {
struct {
  uint16_t servo_rdyToSwitchON : 1;       // 伺服准备好
  uint16_t servo_allowToSwitchON : 1;     // 可以开启伺服运行
  uint16_t servo_OperationEnabled : 1;    // 伺服运行
  uint16_t servo_Fault : 1;               // 故障
  uint16_t servo_VoltageEnabled: 1;       // 主回路电已接通
  uint16_t servo_quickStopActivated : 1;  // 快速停车已生效
  uint16_t servo_SWitchONDisabled : 1;    // 伺服不可运行
  uint16_t servo_warning : 1;             // 警告
  uint16_t servo_mSpecific1 : 1;          // 厂家自定义
  uint16_t servo_remoteControl : 1;       // 远程控制
  uint16_t servo_targetReached : 1;       // 目标已到达  
  uint16_t servo_internalLimitActice : 1; // 位置反馈超限

  //bit 12-14 rely on OperationMode

  /* 速度模式下为
    uint16_t servo_speedIsZero : 1;        // 速度是否为0
    uint16_t servo_NA : 1;
    uint16_t servo_mSpecific2 : 1;

    位置模式下为：
    uint16_t servo_setPointAck : 1;        // 1：不可更新目标位置
    uint16_t servo_followingError : 1;     // 1： 位置误差超限
    uint16_t servo_mSpecific2 : 1;

    转矩模式下未使用
  */
  uint16_t modeRelyDefined1 : 1;        
  uint16_t modeRelyDefined2 : 1;
  uint16_t modeRelyDefined3 : 1;

  uint16_t servo_HomeFounded : 1;        //原点回零完成
}motorStatusWord;
uint16_t Value; 
} STATUS_WORD;

typedef union {
struct {
  uint16_t servo_rdyToSwitchON : 1;       // 可开启伺服运行
  uint16_t servo_enableVoltage : 1;       // 接通主回路电
  uint16_t servo_quickStop : 1;           // 快速停机
  uint16_t servo_enableOperation : 1;     // 伺服运行
  uint16_t reserved1 : 4;                 //快速停机 
  uint16_t servo_Halt : 1;                // 暂停
  uint16_t  reserved2 : 8;
}motorControlWord;
uint16_t Value; 
} CONTROL_WORD_SPEED;

//Torque Mode
typedef union {
struct {
  uint16_t servo_rdyToSwitchON : 1;       // 可开启伺服运行
  uint16_t servo_enableVoltage : 1;       // 接通主回路电
  uint16_t servo_quickStop : 1;           // 快速停机
  uint16_t servo_enableOperation : 1;     // 伺服运行
   uint16_t reserved1 : 4;                 //快速停机 
  uint16_t servo_Halt : 1;                // 暂停
  uint16_t  reserved2 : 8;
}motorControlWord;
uint16_t Value; 
} CONTROL_WORD_TORQUE;

typedef union {
struct {
  uint16_t servo_rdyToSwitchON : 1;        // 可开启伺服运行
  uint16_t servo_enableVoltage : 1;        // 接通主回路电
  uint16_t servo_quickStop : 1;            // 快速停机
  uint16_t servo_enableOperation : 1;      // 伺服运行
  uint16_t servo_newSetPoint : 1;          // 触发新目标位置 
  uint16_t servo_changeSetImmediately : 1; // 目标位置是否立即更新
  uint16_t servo_absORrel : 1;             // 0：绝对位置指令 1：相对位置指令
  uint16_t servo_halt : 1;                 // 1：暂停
  uint16_t reserved2 : 8;
} motorControlWord;
uint16_t Value; 
} CONTROL_WORD_POSITION;

// typedef union {
// typedef struct {
//   uint32_t 


// } di_status;
// uint32_t Value;
// } DI_STATUS;


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
  unsigned char SrcMAC[6];

  unsigned char SrcSendPort;  // No USED

  unsigned char DstHostIP[4];
	unsigned short DstHostPort;
} GLOBAL_ETH_UDP_VAR;

typedef struct {
  uint8_t g_posi[4];      // unit depends on BISS-C
	int32_t g_Distance; 	// um
	int32_t g_Speed; 			// rpm
  int16_t g_phaseAmp;   // A
  //int16_t g_realTimeTorque; 
  float g_realTimeTorque; 
	uint32_t g_InitialPosi; //um

  volatile uint8_t g_DS402_SMStatus;  //状态字处于0x1237时，此状态为1

	enum WORKMODE targetWorkmode;              // Algorithm WorkMode: Speed/Torque/Position
  volatile uint8_t g_curOperationMode; // 当前工作模式
  // 驱动器状态字
  volatile STATUS_WORD motorStatusWord;     
 // 驱动器控制字
  volatile CONTROL_WORD_SPEED motorCMD_speed;    
  volatile CONTROL_WORD_TORQUE motorCMD_torque;     
  volatile CONTROL_WORD_POSITION motorCMD_position;  
//DI DO状态
  // volatile DI_STATUS di;
  // volatile DO_STATUS do;  

} MOTIONVAR;

// CANOpen SDO NUM
#define MAX_PRESET_SDO_NUM       (11)

typedef struct {
  unsigned char CASNodeID; 

  unsigned char CANOpenMasterID;
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

#pragma pack(1)															
typedef union
{
	struct 
	{
		uint32_t MasterOrSlave : 1; 	//主发1 从回0
		uint32_t CTRCode : 5; 				
		uint32_t NodeOrGroupID : 5; 		
		uint32_t Reserved : 21;       
	}CAN_Frame_Union;
	
	uint32_t Value;
} CAN_ID_Union;

//CANFrame
typedef struct
{
    CAN_ID_Union CANID;
    uint8_t CANData[8];
} CANFrame_STD;

#define recvBufLen 3000

typedef struct 
{
	uint32_t transTimeStamp;	//
	uint16_t givenSpeed;  		//
}DACSndStorage;	

// 速度模式下单机控制结构体，对应报文类别为
typedef struct {
    uint32_t EHeader;        //Ethernet帧头
    uint32_t ENum;           //Ethernet帧号
    uint32_t ELen;           //Ethernet长度 Byte
    uint8_t EType;           //报文类型
    CANFrame_STD canpack;
    uint32_t FrameTailer;
}EthControlFrameSingleCAS;

typedef struct {
    uint32_t EHeader;        // Ethernet帧头
    uint32_t ENum;           // Ethernet帧号
    uint32_t ELen;           // Ethernet长度 Byte
    uint8_t EType;           // 报文类型
    uint8_t CASNodeID;       // 数据来源
    uint8_t curWorkMode;     // 当前工作模式
    uint16_t statusWord;     // 状态字
    uint32_t localTimeMS;    // 上报时间 
    uint32_t motorPosiUM;    // 当前绝对位置
    short motorRealTimeTorqueNM; //实时转矩
    short motorAveragePhaseAmp;  //平均相电流
    uint32_t FrameTailer;
} CASREPORTFRAME;

#pragma pack()

// extern Var
extern GLOBALTIME gTime;
extern GLOBALSTATUS gStatus;
extern MODBUSVARS modbusPosi;
extern MOTIONVAR motionStatus;
extern GLOBAL_ETH_UDP_VAR w5500_udp_var;
extern GLOBAL_CAN_VAR can_var;
extern SDRAM_STO_VAR sdram_var;

#endif /* __MAIN_H */
