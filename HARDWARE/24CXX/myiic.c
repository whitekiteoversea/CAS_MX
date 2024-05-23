#include "myiic.h"
#include "stm32f4xx_hal.h"
#include "hal_delay.h"

void IIC_Init(void)
{					     
	RCC->AHB1ENR|=1<<7;   	   	  
	GPIO_Set(GPIOH,PIN4|PIN5,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PH4/PH5
	IIC_SCL=1;
	IIC_SDA=1;
}

void IIC_Start(void)
{
	SDA_OUT();     
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	HAL_Delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	HAL_Delay_us(4);
	IIC_SCL=0;
}	  

void IIC_Stop(void)
{
	SDA_OUT();//sda
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	HAL_Delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;HAL_Delay_us(1);	   
	IIC_SCL=1;HAL_Delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	HAL_Delay_us(2);
	IIC_SCL=1;
	HAL_Delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	HAL_Delay_us(2);
	IIC_SCL=1;
	HAL_Delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		HAL_Delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		HAL_Delay_us(2); 
		IIC_SCL=0;	
		HAL_Delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        HAL_Delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		HAL_Delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























