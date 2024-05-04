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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// 全局工作模式
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

// 全局时间记录
typedef struct {
  unsigned int l_time_cnt_10us;
  unsigned int l_time_ms;
  unsigned int g_time_ms;
} GLOBALTIME;
// 全局状�?�记�?
typedef struct {
  volatile unsigned char l_time_overflow;   // 本地计时溢出
  volatile unsigned char l_time_heartbeat;  // 本地计时心跳
  enum TELMODE telmode;                   // 当前工作模式
	enum WORKMODE workmode;                 // Algorithm WorkMode: Speed/Torque/Position
  volatile unsigned char l_can1_recv_flag; 
  volatile unsigned char l_can2_recv_flag; 
} GLOBALSTATUS;

// ETH模式�?? UDP参数�??
typedef struct {
  unsigned char SrcRecvIP[4];
	unsigned short SrcRecvPort;

  unsigned char DstHostIP[4];
	unsigned short DstHostPort;

  // unsigned char *DstHostIP[2][4];
	// unsigned short DstHostPort[2];
} GLOBAL_ETH_UDP_VAR;

typedef struct {
	int32_t g_Distance; 	// um
	int16_t g_Speed; 			// rpm
	
	uint32_t g_InitialPosi; //um

} MOTIONVAR;

typedef struct {
  unsigned char NodeID;

} GLOBAL_CAN_VAR;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Magnet_RS485_RE_Pin GPIO_PIN_8
#define Magnet_RS485_RE_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define BK_RS485_RE_Pin GPIO_PIN_10
#define BK_RS485_RE_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
extern GLOBALTIME gtime;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
