#include "spi.h"
#include "sys.h"
#include "stm32f4xx_hal.h"

#define	DAC8563_SYNC 		PAout(4) 	//CS信号
#define	DAC8563_SDIN 		PAout(7) 	//MOSI信号
#define	DAC8563_SCLK 		PAout(5) 	//SCLK信号
#define	DAC8563_LDAC 		PBout(0)  //
#define	DAC8563_CLR 		PBout(1)  //

/*DAC8653的一帧由三部分构成
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
		
		DAC8563_SYNC = 0; //片选拉低开始通信
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
		//两个channel均不使用LDAC引脚更新数据 
		rtData = DAC8563_cmd_Write(6,0,3);
		HAL_Delay(50);

		// 复位DAC-A到0, 并更新输出为0V 
		DAC8563_cmd_Write(3, 0, spdDownLimitVol);
		HAL_Delay(50);

		// 使能内部参考并复位2个DAC的增益=2  
		DAC8563_cmd_Write(7, 0, 1);
		HAL_Delay(20);
}

//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
		while((SPI1->SR&1<<1)==0);		//等待发送区空 
		SPI1->DR=TxData;	 	  		//发送一个byte  
		while((SPI1->SR&1<<0)==0);		//等待接收完一个byte  
		return SPI1->DR;          		//返回收到的数据				    
}

//SPI1速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB2/2^(SpeedSet+1)
//fAPB2时钟一般为90Mhz
void SPI1_SetSpeed(u8 SpeedSet)
{
		SpeedSet&=0X07;					//限制范围
		SPI1->CR1&=0XFFC7; 
		SPI1->CR1|=SpeedSet<<3;	//设置SPI1速度  
		SPI1->CR1|=1<<6; 				//SPI设备使能	  
} 

//CAS DAC
void SPI1_DAC8563_Init(void)
{
		u8 temp;   
		RCC->AHB1ENR |= 1<<1;    			//使能PORTB时钟 
		RCC->AHB1ENR |= 1<<0;					//使能PORTA时钟
		RCC->APB2ENR |= 1<<12;				//使能SPI1外设时钟
	
		//GPIO使能
		GPIO_Set(GPIOA,PIN5|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);	 
		GPIO_Set(GPIOA,PIN4,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_Set(GPIOB,PIN0|PIN1,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_NONE);	
	
		//SPI1复用设置 （程序内的SPI1复用选项是错的，得看参考手册）
		//GPIO_AF_Set(GPIOA,4,5);		//PA4,AF5 CS
		GPIO_AF_Set(GPIOA,5,5);		//PA5,AF5 SCLK
		GPIO_AF_Set(GPIOA,7,5);		//PA7,AF5 MOSI 
	
		//GPIO拉低，防止干扰
		DAC8563_LDAC = 0;   				//不需要输出通道同步模式
		DAC8563_CLR = 0;
		
		//初始化并启动SPI1
		SPI1_DAC_Init();
		SPI1_SetSpeed(SPI_SPEED_16); //低速MHz 使能SPI1
	
		//写入DAC8563初始配置
		DAC8563_Config();
}

void SPI1_DAC_Init(void)
{
		u16 tempreg = 0;
	
		RCC->APB2RSTR |= 1<<12;				//复位SPI1
		RCC->APB2RSTR &= ~(1<<12);		//停止复位SPI1

		tempreg|=0<<10;			//全双工模式	
		tempreg|=1<<9;			//软件nss管理
		tempreg|=1<<8;			 
		tempreg|=1<<2;			//SPI主机  
		tempreg|=0<<11;			//8位数据格式	
		tempreg|=1<<1;			//空闲模式下SCK为1 CPOL=1 
		tempreg&= ~(1<<0);			//数据采样从第1个时间边沿开始,CPHA=0  
		//对SPI1属于APB2的外设.时钟频率最大为90Mhz频率.
		tempreg|=7<<3;			//Fsck=Fpclk1/256
		tempreg|=0<<7;			//MSB First  
		SPI1->CR1=tempreg; 		//设置CR1 
				
		SPI1->I2SCFGR &= ~(1<<11);		//选择SPI模式
}

