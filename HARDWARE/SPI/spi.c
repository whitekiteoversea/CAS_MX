#include "spi.h"
#include "sys.h"
#include "stm32f4xx_hal.h"

#define	DAC8563_SYNC 		PAout(4) 	//CS�ź�
#define	DAC8563_SDIN 		PAout(7) 	//MOSI�ź�
#define	DAC8563_SCLK 		PAout(5) 	//SCLK�ź�
#define	DAC8563_LDAC 		PBout(0)  //
#define	DAC8563_CLR 		PBout(1)  //

/*DAC8653��һ֡�������ֹ���
*  bit23-22 not use
*  bit21-19 cmd
*  bit18-16 addr
*  bit15-0  data
*/

u8 DAC8563_cmd_Write(u8 cmd, u8 addr, u16 data)
{
		u8 sndData[3]={0};
		u8 i =0;
		u8 returnData = 0;
		
		sndData[0] = (cmd << 3) | addr;
		sndData[1] = (u8)((data & 0xFF00) >> 8);
		sndData[2] = (u8)(data & 0x00FF);
		
		DAC8563_SYNC = 0; //Ƭѡ���Ϳ�ʼͨ��
		for(i=0;i<3;i++)
		{
				returnData = SPI1_ReadWriteByte(sndData[i]);
		}
		DAC8563_SYNC = 1;
		return returnData;
}	

void DAC8563_Config(void)
{
		u8 rtData = 0;
		// Power up DAC-A  DAC-B
		rtData = DAC8563_cmd_Write(4,0,3);
		HAL_Delay(50);
		
		// LDAC pin inactive for DAC-B and DAC-A  
		//����channel����ʹ��LDAC���Ÿ������� 
		rtData = DAC8563_cmd_Write(6,0,3);
		HAL_Delay(50);

		// ��λDAC-A��0, ���������Ϊ0V 
		DAC8563_cmd_Write(3, 0, spdDownLimitVol);
		HAL_Delay(50);

		// ʹ���ڲ��ο�����λ2��DAC������=2  
		DAC8563_cmd_Write(7, 0, 1);
		HAL_Delay(20);
}

//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
		while((SPI1->SR&1<<1)==0);		//�ȴ��������� 
		SPI1->DR=TxData;	 	  		//����һ��byte  
		while((SPI1->SR&1<<0)==0);		//�ȴ�������һ��byte  
		return SPI1->DR;          		//�����յ�������				    
}

//SPI1�ٶ����ú���
//SpeedSet:0~7
//SPI�ٶ�=fAPB2/2^(SpeedSet+1)
//fAPB2ʱ��һ��Ϊ90Mhz
void SPI1_SetSpeed(u8 SpeedSet)
{
		SpeedSet&=0X07;					//���Ʒ�Χ
		SPI1->CR1&=0XFFC7; 
		SPI1->CR1|=SpeedSet<<3;	//����SPI1�ٶ�  
		SPI1->CR1|=1<<6; 				//SPI�豸ʹ��	  
} 

//CAS DAC
void SPI1_DAC8563_Init(void)
{
		u8 temp;   
		RCC->AHB1ENR |= 1<<1;    			//ʹ��PORTBʱ�� 
		RCC->AHB1ENR |= 1<<0;					//ʹ��PORTAʱ��
		RCC->APB2ENR |= 1<<12;				//ʹ��SPI1����ʱ��
	
		//GPIOʹ��
		GPIO_Set(GPIOA,PIN5|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);	 
		GPIO_Set(GPIOA,PIN4,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_Set(GPIOB,PIN0|PIN1,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);	
	
		//SPI1�������� �������ڵ�SPI1����ѡ���Ǵ�ģ��ÿ��ο��ֲᣩ
		//GPIO_AF_Set(GPIOA,4,5);		//PA4,AF5 CS
		GPIO_AF_Set(GPIOA,5,5);		//PA5,AF5 SCLK
		GPIO_AF_Set(GPIOA,7,5);		//PA7,AF5 MOSI 
	
		//GPIO���ͣ���ֹ����
		DAC8563_LDAC = 0;   				//����Ҫ���ͨ��ͬ��ģʽ
		DAC8563_CLR = 0;
		
		//��ʼ��������SPI1
		SPI1_DAC_Init();
		SPI1_SetSpeed(SPI_SPEED_16); //����MHz ʹ��SPI1
	
		//д��DAC8563��ʼ����
		DAC8563_Config();
}

void SPI1_DAC_Init(void)
{
		u16 tempreg = 0;
	
		RCC->APB2RSTR |= 1<<12;				//��λSPI1
		RCC->APB2RSTR &= ~(1<<12);		//ֹͣ��λSPI1

		tempreg|=0<<10;			//ȫ˫��ģʽ	
		tempreg|=1<<9;			//���nss����
		tempreg|=1<<8;			 
		tempreg|=1<<2;			//SPI����  
		tempreg|=0<<11;			//8λ���ݸ�ʽ	
		tempreg|=1<<1;			//����ģʽ��SCKΪ1 CPOL=1 
		tempreg&= ~(1<<0);			//���ݲ����ӵ�1��ʱ����ؿ�ʼ,CPHA=0  
		//��SPI1����APB2������.ʱ��Ƶ�����Ϊ90MhzƵ��.
		tempreg|=7<<3;			//Fsck=Fpclk1/256
		tempreg|=0<<7;			//MSB First  
		SPI1->CR1=tempreg; 		//����CR1 
				
		SPI1->I2SCFGR &= ~(1<<11);		//ѡ��SPIģʽ
}

