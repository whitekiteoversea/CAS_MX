#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "stm32f4xx_hal.h"
#include "mb4_1sf_driver.h"
	 				    
// SPI Speed
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7

#define spdDownLimitVol (32820)
#define spdUpLimitVol   (43742)

#define HAL_DAC_ENABLE      (0)  
#define HAL_BISSC_ENABLE    (1)

// 位置增长方向为向下
#define POSIRANGESTART_LEFT (8697903)
#define POSIRANGEEND_LEFT   (9203551)

#define POSIRANGESTART_RIGHT (9259537)
#define POSIRANGEEND_RIGHT   (9769376)

// 从左向右递增
#define POSIRANGESTART_XAXIS (0)
#define POSIRANGEEND_XAXIS   (0)

// SPI1 DAC8563 GPIO Operations
#define	GPIO_SPI_DAC8563_SYNC_SET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define	GPIO_SPI_DAC8563_SDIN_SET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define	GPIO_SPI_DAC8563_SCLK_SET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)   
#define	GPIO_SPI_DAC8563_LDAC_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)  
#define GPIO_SPI_DAC8563_CLR_SET    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define	GPIO_SPI_DAC8563_SYNC_RESET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define	GPIO_SPI_DAC8563_SDIN_RESET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define	GPIO_SPI_DAC8563_SCLK_RESET  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)   
#define	GPIO_SPI_DAC8563_LDAC_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)  
#define GPIO_SPI_DAC8563_CLR_RESET    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

// SPI4 W5500 GPIO Operation
#define	GPIO_SPI_W5500_CS_SET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)
#define	GPIO_SPI_W5500_SCLK_SET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)
#define	GPIO_SPI_W5500_MISO_SET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET)   
#define	GPIO_SPI_W5500_MOSI_SET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET)  
// #define GPIO_SPI_W5500_INT_SET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_SET)
// #define GPIO_SPI_W5500_RST_SET    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

#define	GPIO_SPI_W5500_CS_RESET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
#define	GPIO_SPI_W5500_SCLK_RESET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET)
#define	GPIO_SPI_W5500_MISO_RESET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET)   
#define	GPIO_SPI_W5500_MOSI_RESET  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET)  
// #define GPIO_SPI_W5500_INT_RESET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_RESET)
// #define GPIO_SPI_W5500_RST_RESET    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)

#ifdef HAL_BISSC_ENABLE
// SPI2 BISS-C Operation
#define	GPIO_SPI_BISSC_CS_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define	GPIO_SPI_BISSC_SCLK_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define	GPIO_SPI_BISSC_MISO_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)   
#define	GPIO_SPI_BISSC_MOSI_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)  
#define GPIO_SPI_BISSC_NRES_SET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET)
#define	GPIO_SPI_BISSC_EOT_SET  	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET)  
#define GPIO_SPI_BISSC_GETSENS_SET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET)
#define GPIO_SPI_BISSC_NER_SET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_SET)
#define	GPIO_SPI_BISSC_NWRE_SET  	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET)  
#define GPIO_SPI_BISSC_NRDRNW_SET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET)

#define	GPIO_SPI_BISSC_CS_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define	GPIO_SPI_BISSC_SCLK_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define	GPIO_SPI_BISSC_MISO_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)   
#define	GPIO_SPI_BISSC_MOSI_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)  
#define GPIO_SPI_BISSC_NRES_RESET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET)
#define	GPIO_SPI_BISSC_EOT_RESET  	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET)  
#define GPIO_SPI_BISSC_GETSENS_RESET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET)
#define GPIO_SPI_BISSC_NER_RESET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4, GPIO_PIN_RESET)
#define	GPIO_SPI_BISSC_NWRE_RESET  	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET)  
#define GPIO_SPI_BISSC_NRDRNW_RESET    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET)

enum BISSC_CMD {
    WriteData = 0x02,
    ReadData = 0x03,
    ReadStatus = 0x05,
    WriteInstruction = 0x07,
    ReadData0 = 0x09,
    WriteData0 = 0x0b 
};

#endif

void SPI5_Init(void);			 
void SPI5_SetSpeed(u8 SpeedSet);   
u8 SPI5_ReadWriteByte(u8 TxData);

/* SPI1 DAC8563 相关函数*/
void SPI1_SetSpeed(u8 SpeedSet);
u8 SPI1_ReadWriteByte(u8 TxData);
void SPI1_DAC8563_Init(void);  //���Ķ�F429�������Ѿ���֤���ĵ�SPI�����⿪һ����ʼ������
void DAC8563_Config(void);
void SPI1_DAC_Init(void);
u8 DAC8563_cmd_Write(u8 cmd, u8 addr, u16 data);

// HAL库下执行
void HAL_SPI1_DAC_Init(void);
void HAL_SPI1_DAC8563_Init(void);
void HAL_DAC8563_Config(void);
void HAL_SPI1_SetSpeed(u8 SpeedSet);
HAL_StatusTypeDef HAL_DAC8563_cmd_Write(u8 cmd, u8 addr, u16 data);

/* SPI4 W5500相关函数 */
uint8_t HAL_SPI4_WriteAndReadByte(uint8_t TxData);
void HAL_SPI4_WriteByte(uint8_t TxData);
uint8_t HAL_SPI4_ReadByte(void);

void SPI_CrisEnter(void);
void SPI_CrisExit(void);
void SPI4_CS_Select(void);
void SPI4_CS_Deselect(void);
uint8_t BISSC_F0Status_Display(uint8_t status);

#ifdef HAL_BISSC_ENABLE

/* SPI2 BISS-C相关函数*/
void HAL_BISSC_Setup(void);
uint8_t HAL_SG_SenSorAcquire(uint32_t *pSG_Data); 
void HAL_CTLRegsWrite_Slave0(uint8_t reg_addr, uint8_t reg_data); 
uint8_t HAL_CTLRegs_Read_Slave0(uint8_t readAddr);
void HAL_BISSC_reStartAGS(void);
void HAL_BISSC_StartAGS(void); 

#endif

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
		 
#endif

