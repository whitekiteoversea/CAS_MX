#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
#include "hal_delay.h"
   	   		   
#define SDA_IN()  {GPIOH->MODER&=~(3<<(5*2));GPIOH->MODER|=0<<5*2;}
#define SDA_OUT() {GPIOH->MODER&=~(3<<(5*2));GPIOH->MODER|=1<<5*2;} 

#define IIC_SCL    PHout(4) //SCL
#define IIC_SDA    PHout(5) //SDA	 
#define READ_SDA   PHin(5)  //SDA 

void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź� 
#endif
















