#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"
#include "spi.h"

#define usr_lcd_spi hspi3
#define ST7789_240_135

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#define LCD_W 240
#define LCD_H 135

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET)

//MISO 或者叫 DC
#define LCD_DC_Clr()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET)//CS
#define LCD_CS_Set()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化

#endif


