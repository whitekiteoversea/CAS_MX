#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
	 				    
// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7

#define spdDownLimitVol (32820)
#define spdUpLimitVol   (58000)
						  	    													  
void SPI5_Init(void);			 //初始化SPI口
void SPI5_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI5_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

void SPI1_SetSpeed(u8 SpeedSet);
u8 SPI1_ReadWriteByte(u8 TxData);
void SPI1_DAC8563_Init(void);  //不改动F429开发板已经验证过的的SPI，另外开一个初始化函数
void DAC8563_Config(void);
void SPI1_DAC_Init(void);
u8 DAC8563_cmd_Write(u8 cmd, u8 addr, u16 data);
		 
#endif

