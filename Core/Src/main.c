/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789_hal_spi.h"
#include "usart.h"
#include "sys.h"
#include "lcd_driver.h"
#include "spi.h"
#include "socket.h"	
#include "string.h"
#include "can.h"
#include "pid.h"
#include "oled.h"
#include "bmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOCK_TCPS        0
#define DATA_BUF_SIZE   2048

#define CAN2_SENDTEST_ON (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t gDATABUF[DATA_BUF_SIZE];  // ����ת�Ƶ��ⲿSDRAM???2KByteռ������SRAM???50%
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
GLOBALTIME gTime;
GLOBALSTATUS gStatus;
MOTIONVAR motionStatus;
GLOBAL_ETH_UDP_VAR w5500_udp_var;
GLOBAL_CAN_VAR can_var;
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0x11},
                            .ip = {192, 168, 1, 11},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

uint8_t flagStatus = 0; 					// �����λ���ڱ�ʶ���յ���Ӧ�ڵ�����?��??														
int32_t avgPosiErr[2] = {0}; 			// ƽ��ͬ������ȡ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void network_register(void);
void network_init(void);								// Initialize Network information and display it

void systemParaInit(void);
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode); // can recv info distribute

int32_t avgErrCollect(uint8_t node, int32_t sampleData);  
int32_t avgErrUpdate(int32_t *sampleData); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t ret =0;
	uint8_t snddata[8] = {0};
	CAN_ID_Union ext_ID;
  unsigned int sensorData = 0;
  uint8_t cnt = 0;
	uint8_t prx = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	systemParaInit();
	
  // 1. local timebase
  HAL_TIM_Base_Start_IT(&htim3);
  // 2. LCD Status Display
	// LCD_Init();
	// Lcd_Full(RED);

  // Backup: I2C1 OLED 128*64
		OLED_Init();			//��ʼ��OLED  
	//	OLED_Clear(); 
	
    // OLED_ShowNum(30, 30, 15, 2, 16); // ��(30,30)����ʾһ������15��"15"����λ���ַ��ֺ�Ϊ16

		// OLED_DrawBMP(0,0, 128, 8, BMP1);  //ͼƬ��ʾ(ͼƬ��ʾ���ã����ɵ��ֱ��ϴ󣬻�ռ�ý϶�ռ䣬FLASH�ռ�8K��������)
		// HAL_Delay(8000);
		// OLED_Clear();

  // 3. ETH Initial
  // network_register();
  // network_init();

  // 4. BISS-C Sensor Data Acquire
	// HAL_Delay(500);
  
//	HAL_BISSC_Setup();

  // 5. Motor Torque Controller
  // HAL_SPI1_DAC8563_Init();   
  // HAL_Delay(500);
  // HAL_DAC8563_cmd_Write(3, 0, spdDownLimitVol);   // �����ⲿ�ٶȳ�???

#if CAN2_SENDTEST_ON
	ext_ID.CAN_Frame_Union.CTRCode = CANTimeSyncCmd; // position acquire pack type
	ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack slave
	ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (gStatus.l_time_heartbeat == 1) {
      printf("%d ms HeartBeat Msg \n\r", gTime.l_time_ms);
      gStatus.l_time_heartbeat = 0;
    }
    
    // 1000ms Acquire BISS-C TEST
//    if (gStatus.l_bissc_sensor_acquire == 1) {
//      HAL_SG_SenSorAcquire(motionStatus.g_posi);
//			sensorData = 0;
//      for (cnt=0; cnt<5; cnt++) {
//        sensorData |= motionStatus.g_posi[cnt];
//        sensorData <<= 8;
//      }
//      printf("%d ms Posi is %d unit \n\r", gTime.l_time_ms, sensorData);
//      gStatus.l_bissc_sensor_acquire = 0;
//    }


    if (gStatus.l_can1_recv_flag == 1) {
        CANRecvMsgDeal(&hcan1, CAN1RecvFrame.CAN_Frame_Union.CTRCode);
			  gStatus.l_can1_recv_flag = 0;
    }

    if (gStatus.l_can2_recv_flag == 1) {
        CANRecvMsgDeal(&hcan2, CAN2RecvFrame.CAN_Frame_Union.CTRCode);
			  gStatus.l_can2_recv_flag = 0;
    }  



#if CAN2_SENDTEST_ON
		HAL_CAN_Ext_Transmit(&hcan2, (void *)snddata, 4, ext_ID.Value);
		HAL_Delay(500);
#endif
		/*
    switch (getSn_SR(0))																					    // ��ȡsocket0��״???
		{
			case SOCK_UDP:																							    // Socket���ڳ�ʼ����???(��)״???
					HAL_Delay(100); // ʱ��̫���ˣ�???Ҫ�����ܲ��ܼ���
					if(getSn_IR(0) & Sn_IR_RECV)
					{
						setSn_IR(0, Sn_IR_RECV);															    // Sn_IR��RECVλ��1
					}
					// ���ݻػ����Գ������ݴ�Զ����λ������W5500��W5500���յ����ݺ��ٻظ�Զ����λ��
					if((ret = getSn_RX_RSR(0)) > 0)
					{ 
						memset(gDATABUF, 0, ret+1);
						recvfrom(0, gDATABUF, ret, w5500_udp_var.DstHostIP, &w5500_udp_var.DstHostPort);			// W5500��������Զ����λ�������ݣ���ͨ��SPI��???��MCU
						printf(" %d ms %s\r\n", gTime.l_time_ms, gDATABUF);															  // ���ڴ�ӡ���յ�������
						sendto(0, gDATABUF,ret, w5500_udp_var.DstHostIP, w5500_udp_var.DstHostPort);		  		// ���յ����ݺ��ٻظ�Զ����λ����������ݻ�??????
					}
			break;
			case SOCK_CLOSED:																						    // Socket���ڹر�״???
					socket(0, Sn_MR_UDP, w5500_udp_var.SrcRecvPort, 0x00);			// ��Socket0��������ΪUDPģʽ����??????�����ض�???
			break;
		}
    */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN2_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  sFilterConfig.FilterBank = 0;   //channel 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  
  sFilterConfig.FilterIdLow = 0x0000;   //??????   
  //MASK bit 0 means don't care,bit 0 means match 
  sFilterConfig.FilterMaskIdHigh = 0x0000;   
  sFilterConfig.FilterMaskIdLow = 0x0000;                                 
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; 
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;   //enable filter
  sFilterConfig.SlaveStartFilterBank = 14;    
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    printf("CAN1 Start Failed!\n\r");
  }
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
  CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  sFilterConfig.FilterBank = 14;   //channel 14
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  
  sFilterConfig.FilterIdLow = 0x0000;   
  //MASK bit 0 means don't care,bit 0 means match 
  sFilterConfig.FilterMaskIdHigh = 0x0000;   
  sFilterConfig.FilterMaskIdLow = 0x0000;                                 
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; 
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;   //enable filter
  sFilterConfig.SlaveStartFilterBank = 28;    
  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    printf("CAN2 Start Failed!\n\r");
  }
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 500000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1943200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 3750000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|RS485_Senser_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|SPI1_CS_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1|GETSENS_Pin|NER_Pin|NWR_E_Pin
                          |NRD_RNW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BK_RS485_RE_GPIO_Port, BK_RS485_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C1_SCLK_Pin|I2C1_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PE3 SPI4_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI11 EOT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|EOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI5_CS_Pin */
  GPIO_InitStruct.Pin = SPI5_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI5_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 RS485_Senser_RE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|RS485_Senser_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 SPI1_CS_Pin SPI3_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|SPI1_CS_Pin|SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PH2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 SPI2_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PI1 GETSENS_Pin NER_Pin NWR_E_Pin
                           NRD_RNW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GETSENS_Pin|NER_Pin|NWR_E_Pin
                          |NRD_RNW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : BK_RS485_RE_Pin */
  GPIO_InitStruct.Pin = BK_RS485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BK_RS485_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCLK_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCLK_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_GPIO_WritePin(GPIOE, SPI4_CS_Pin, GPIO_PIN_SET); // SPI CSĬ�ϲ�ʹ???
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin, GPIO_PIN_SET); // SPI CSĬ�ϲ�ʹ???
  HAL_GPIO_WritePin(GPIOF, SPI5_CS_Pin, GPIO_PIN_SET); // SPI CSĬ�ϲ�ʹ???
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|SPI3_CS_Pin, GPIO_PIN_SET); // SPI CSĬ�ϲ�ʹ???
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //LCD ��ʼ��ʱΪ��������״̬
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //���⿪��

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // weak �ú������û����¶���
{
  static unsigned int heartbeatChangedMs = 0; // ���ڱ�ʶ�Ƿ����仯
	 if(htim == &htim3)
	 {
      // 1�����ؼ�??? 10us
      if (gTime.l_time_cnt_10us < 360000000 ) { // 60min���???
        gTime.l_time_cnt_10us++;
      } else {
        gTime.l_time_cnt_10us = 0;
        gTime.l_time_ms = 0;
        gStatus.l_time_overflow++;
      }
      // 2����??? 1s
      if (gTime.l_time_cnt_10us % 100 == 0) {
        gTime.l_time_ms++;
      }

      if ((gTime.l_time_ms % 1000 == 0) && (0 != gTime.l_time_ms-heartbeatChangedMs)) {
        gStatus.l_time_heartbeat = 1;
        heartbeatChangedMs = gTime.l_time_ms;
				
				gStatus.l_bissc_sensor_acquire = 1;
      }
	 }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan)
{
    UNUSED(phcan);
    CAN_RxHeaderTypeDef  RxMessage;
    unsigned char rxbuf[8] = {0};
    unsigned char cnt = 0;

		printf("%d ms CAS: recv New Frame \n\r", gTime.l_time_ms);
		
    if (phcan == &hcan1) {
        HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &RxMessage, rxbuf);
        CAN1RecvFrame.Value = RxMessage.ExtId;
        if ((CAN1RecvFrame.CAN_Frame_Union.NodeOrGroupID == can_var.NodeID) && (CAN1RecvFrame.CAN_Frame_Union.MasterOrSlave == 1)){
            for (cnt = 0; cnt < 8; cnt++) {
                CAN1_RecData[cnt] = rxbuf[cnt];
            }
            printf("%d ms CAS: recv New Frame Type is %d\n\r", gTime.l_time_ms, CAN1RecvFrame.CAN_Frame_Union.CTRCode);
            gStatus.l_can1_recv_flag = 1;
        } else {
            gStatus.l_can1_recv_flag = 0;
        }
    } else if (phcan == &hcan2) {
        HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &RxMessage, rxbuf);
        CAN2RecvFrame.Value = RxMessage.ExtId;
        if (CAN2RecvFrame.CAN_Frame_Union.NodeOrGroupID == can_var.NodeID) {
            for (cnt = 0; cnt < 8; cnt++) {
                CAN2_RecData[cnt] = rxbuf[cnt];
            }
            printf("%d ms CAS: recv New Frame Type is %d\n\r", gTime.l_time_ms, CAN1RecvFrame.CAN_Frame_Union.CTRCode);
            gStatus.l_can2_recv_flag = 1;
        } else {
            gStatus.l_can2_recv_flag = 0;
        }
    } else {
        printf("%d recv error can frame \n\r", gTime.l_time_ms);
    }
}

// ETH��ʼ��
void network_init(void)
{
    uint8_t tmpstr[6];
    ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
    ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);

    // Display Network Information
    ctlwizchip(CW_GET_ID,(void*)tmpstr);
    printf("\r\n === %d ms %s NET CONF ===\r\n", gTime.l_time_ms, (char*)tmpstr);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],
        gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
    printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
    printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
    printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
    printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
    printf("======================\r\n");
}

void network_register(void)
{
	uint8_t tmp;
	int16_t ret = 0;
  uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
		
  reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);	//ע���ٽ�����???
  /* Chip selection call back */
  #if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
    reg_wizchip_cs_cbfunc(SPI4_CS_Select, SPI4_CS_Deselect);//ע��SPIƬ???�źź�???
  #elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
    reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
  #else
    #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
        #error "Unknown _WIZCHIP_IO_MODE_"
    #else
        reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    #endif
  #endif
    /* SPI Read & Write callback function */
    reg_wizchip_spi_cbfunc(HAL_SPI4_ReadByte, HAL_SPI4_WriteByte);	//ע���д����???

    /* WIZCHIP SOCKET Buffer initialize */
    if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
      printf("%d ms WIZCHIP Initialized fail.\r\n", gTime.l_time_ms);
      while(1);
    }

    /* PHY link status check */
    do{
      if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){
          printf("Unknown PHY Link stauts.\r\n");
      }
    }while(tmp == PHY_LINK_OFF);
}

void systemParaInit(void)
{
  gStatus.telmode = IDLEMODE; //������Ĭ�Ͽɽ���ETH��CANͨ�ţ�ʵ��DAC���ǰ���л���CAN/ETH��һģʽ???
  gStatus.workmode = RECVSPEEDMODE; // ����ģʽ�������������������������???
	
	motionStatus.g_Distance = 0; // �ϵ���eeprom��ȡ
	motionStatus.g_Speed = 0;
	
  can_var.NodeID = 0x01; // �ϵ���EEPROM��ȡ
  
  w5500_udp_var.DstHostIP[0] = 192;
	w5500_udp_var.DstHostIP[1] = 168;
	w5500_udp_var.DstHostIP[2] = 1;
	w5500_udp_var.DstHostIP[3] = 5;
	w5500_udp_var.DstHostPort = 8888;
	
	w5500_udp_var.SrcRecvIP[0] = 192;
	w5500_udp_var.SrcRecvIP[1] = 168;
	w5500_udp_var.SrcRecvIP[2] = 1;
	w5500_udp_var.SrcRecvIP[3] = 11;
  w5500_udp_var.SrcRecvPort = 8001;
}


// ʵʱƽ��ͬ�����ɼ�
int32_t avgErrCollect(uint8_t node, int32_t sampleData) 
{
	int32_t duss_result = 0;
	uint8_t snddata[8] = {0};

	if (node >= 3) {
		printf("UTC:%d ms CAS: %d Recv Error Context Pack\n\r", gTime.g_time_ms, gTime.l_time_ms);
		return -1;
	}
	avgPosiErr[node] = sampleData;

	//��3λ����3���ڵ��Լ�����Ϣ�ռ����???
	flagStatus |= (0x01 << (node-1));

	return duss_result;
}

// ����CAN ���ݴ�������
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode)
{
    UNUSED(phcan);
   
    volatile u16 lastNewestSpeed = 0;
    volatile u32 curLocalTimeStamp = 0;

    u8 snddata[8]={0};
    short curGivenSpeed = 0; //rpm
    u32 tempGivenVol = 0;    //mV
    u32 sendCnt = 0;
    u16 tempFrameCnt = 0;
    int32_t tempPosiErr = 0;
    int32_t tempRecvPosi = 0;

    // Ext CAN ID
    CAN_ID_Union ext_ID;
    ext_ID.Value = 0;

    // ֡�Ż�ȡ
    tempFrameCnt = CAN1_RecData[0];
    tempFrameCnt <<= 8;
    tempFrameCnt |= CAN1_RecData[1];
    
    //���ݿ�����ظ�CAN��������
    switch(CTRCode)
    {
        //�ٶȸ���
        case CANSpeedCmd:
          //�ٶ�������
          curGivenSpeed = CAN1_RecData[5];
          curGivenSpeed <<= 8;
          curGivenSpeed |= CAN1_RecData[6];
        
          //��Ч�ٶȷ�Χ:��ʱ�������ٶȷ�Χ -1800-1800rpm
          if((curGivenSpeed <= 1800) && (curGivenSpeed >= -1800))
          {	
            tempGivenVol = 15.8 *curGivenSpeed + spdDownLimitVol ;
          
            //��Hex�ֽ��·������ٶ�ֵ��-1800rpm��1800rpm���ֱ���1rpm��ת��Ϊ��Ӧ��ѹDAC�ź� ��10V
            // 16384 - 10V -1800rpm
            HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
            printf("UTC: %d ms CAS: %d, frameNum: %d, update Speed :%d rpm\n\r", gTime.g_time_ms,
                                                                                  gTime.l_time_ms,
                                                                                  tempFrameCnt,
                                                                                  curGivenSpeed);
          }
        break;
        
        //�ٶ�Ԥ�����???
        case CANSpeedPreCmd:
          curGivenSpeed = CAN1_RecData[5];
          curGivenSpeed <<= 8;
          curGivenSpeed |= CAN1_RecData[6];
        
          //��Ч�ٶȷ�Χ:��ʱ�������ٶȷ�Χ -1800-1800rpm
          if (((short)curGivenSpeed <= 1800) && ((short)curGivenSpeed >= -1800)) {	
              tempGivenVol = 15.8 *curGivenSpeed + spdDownLimitVol ;
            
              //��Hex�ֽ��·������ٶ�ֵ��-1800rpm��1800rpm���ֱ���1rpm��ת��Ϊ��Ӧ��ѹDAC�ź� ��10V
              // 16384 - 10V -1800rpm
          
              //DAC���??? 2022.05.27
              HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
              printf("UTC: %d ms CAS: %d, update Speed :%d rpm, t\n\r",  gTime.g_time_ms,
                                                                        gTime.l_time_ms,
                                                                        curGivenSpeed);
          }
        break;
          
        //ʱ��ͬ��ָ��
        case CANTimeSyncCmd:
            //update global timeStamp
            gTime.g_time_ms = CAN1_RecData[2];
            gTime.g_time_ms <<= 8;
            gTime.g_time_ms |= CAN1_RecData[3];
            gTime.g_time_ms <<= 8;
            gTime.g_time_ms |= CAN1_RecData[4];	
            gTime.g_time_ms = gTime.g_time_ms;
            memcpy(snddata, CAN1_RecData, 8);		

            ext_ID.CAN_Frame_Union.CTRCode = CANTimeSyncCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack slave
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node
            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 4, ext_ID.Value);

        break;
      
        //SEC λ�Ʋ�ѯ
        case CANPisiAcquireCmd:
            //֡�� + ʱ������ٶ��޻�·ʱ�ӣ�???
            memcpy(snddata, CAN1_RecData, 8);
            //������ڻ�·ʱ�ӣ���?���ظ���ʱ�������?;��������ڣ�������λ������ʱ�����ȡ
            
            //����������
            snddata[5] = (motionStatus.g_Distance & 0x00FF0000) >> 16;
            snddata[6] = (motionStatus.g_Distance & 0x0000FF00) >> 8;
            snddata[7] = (motionStatus.g_Distance & 0x000000FF);

            ext_ID.CAN_Frame_Union.CTRCode = CANPisiAcquireCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node

            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
            printf("UTC:%d ms CAS: %d CurPosi is %d \n\r", gTime.g_time_ms, gTime.l_time_ms, motionStatus.g_Distance);
        break;

        // SEC ƽ��ͬ�����ʵʱ����???: ʱ���???+ƽ�����??? (�������ڵ��?����λ���ڵ㲻�·�??)
        case CANTimeSyncErrorCalCmd: 
            tempRecvPosi = CAN1_RecData[5];
            tempRecvPosi <<= 8;
            tempRecvPosi |= CAN1_RecData[6];

            avgErrCollect(ext_ID.CAN_Frame_Union.NodeOrGroupID, tempRecvPosi);
            if (flagStatus == 7) {
                tempPosiErr = avgErrUpdate(avgPosiErr);
                flagStatus = 0;	

              // 1�Žڵ����ϸ���ƽ��ͬ�����???
              if (can_var.NodeID == 0x01) {
                  // ʱ���???(ȫ��ʱ����?�����??)
                  snddata[2] = 0;
                  snddata[3] = 0;
                  snddata[4] = 0;
                  // ƽ��ͬ�����???
                  snddata[5] = (tempPosiErr & 0x00FF0000) >> 16;
                  snddata[6] = (tempPosiErr & 0x0000FF00) >> 8;
                  snddata[7] = (tempPosiErr & 0x000000FF);

                  ext_ID.CAN_Frame_Union.CTRCode = CANTimeSyncErrorCalCmd; // position acquire pack type
                  ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
                  ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node
								  HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
              }
          }
        break;
          
        // ����PIλ�û�����
        case CANLocalPITestCmd:
          if (CAN1_RecData[0] == 1) {
            PIDController_Reset(&pid);
            gStatus.workmode = PIPOSIMODE;
            givenExecPosiVal += 20600;
          }
          else if (CAN1_RecData[0] == 2) {
            gStatus.workmode = PREPOSIMODE;
          }
          else {
            gStatus.workmode = RECVSPEEDMODE;
          }
          printf("UTC:%d ms CAS: %d Start PI Position Control, Initial posi is %d, givenPosiVal is %d\n\r", gTime.g_time_ms, 
                                                            gTime.l_time_ms,
                                                            recvPosiInitVal,
                                                            givenExecPosiVal);
        break;

        default:
          printf("UTC:%d ms CAS: %d Recv Error Context Pack\n\r", gTime.g_time_ms, gTime.l_time_ms);
        break;
    }
}

int32_t avgErrUpdate(int32_t *sampleData) 
{
	int32_t duss_result =0;
	
	
	return duss_result;
}
/* USER CODE END 1 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
