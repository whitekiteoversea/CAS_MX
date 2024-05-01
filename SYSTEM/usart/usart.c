#include "sys.h"
#include "usart.h"	
  
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)  
//解决HAL库使用时,某些情况可能报错的bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}

//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;      

//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 默认USART1
// int fputc(int ch, FILE *f)
// {      
// 	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
// 	USART1->DR = (u8) ch;      
// 	return ch;
// }

//重定义fputc函数 默认UART4
int fputc(int ch, FILE *f)
{      
	while((UART4->SR & 0X40)==0);//循环发送,直到发送完毕   
	UART4->DR = (u8) ch;      
	return ch;
}

#endif 


/*************串口处理函数***************/

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误  
//MODBUS ASCII
void USART1_IRQHandler(void)
{
		char recvByte = 0; //这个地方因为发过来的数据我只关心0x30-0x39,0x41-0x46
		uint8_t comNewFrameCnt = 0;   	//标识新的一帧数据接收
		uint8_t comNewRecvEndFlag = 0;  //用于标识当前接收是否为结束字符起始 
		//如果串口接收缓冲区非空
		if(USART1->SR&(1<<5))
		{
				recvByte = USART1->DR; //读取当前字节
				//校验帧数据
				if(recvByte == 0x3A)   //接收起始
				{
						comNewFrameCnt = 1;
				}
				else if(recvByte == 0x0D)
				{
						comNewRecvEndFlag = 1;
				}
				else if(recvByte == 0x0A)
				{
						//先只考虑单帧接收
						if(comNewRecvEndFlag == 1)
						{
								//LRC校验（得先转Hex再做加法）
								lrcResult = g_lrc_Test(USART_RX_BUF, comRecvCnt);
								if(lrcResult != NULL)
								{
										comNewFrameCnt++;  //接收到了完整一帧
								}
								else if(lrcResult == NULL)
								{
										//校验不通过，丢弃
								}
								comNewRecvEndFlag = 0;
						}
						//丢弃
				}
				//正常数据范围
				else if(((0 <=(recvByte - '0')) && (0 >= (recvByte - '9'))) ||
								((0 <= (recvByte - 'A')) && (0 >=(recvByte -'F'))))
				{
						USART_RX_BUF[comRecvCnt++] = recvByte;
				}
		}
} 
#endif	


//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
	
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
		mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
		RCC->APB2ENR|=1<<4;  	//使能串口1时钟 
	
		GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA9,PA10,复用功能,上拉输出
		GPIO_AF_Set(GPIOA,9,7);	//PA9,AF7
		GPIO_AF_Set(GPIOA,10,7);//PA10,AF7  	
	
		//波特率设置
		USART1->BRR=mantissa; 	//波特率设置	 
		USART1->CR1&=~(1<<15); 	//设置OVER8=0 
		USART1->CR1|=1<<3;  	//串口发送使能 
	
#if EN_USART1_RX		  	//如果使能了接收
		//使能接收中断 
		USART1->CR1|=1<<2;  	//串口接收使能
		USART1->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		//MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级 
		MY_NVIC_Init(1,1,USART1_IRQn,2);//抢占1，组内1，优先级
#endif
		USART1->CR1|=1<<13;  	//串口使能
}

//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
uint8_t UART4_SendBuf[200] = {0}; 

void uart4_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
	
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 								 //得到整数部分
		fraction=(temp-mantissa)*16;           //得到小数部分@OVER8=0 
		mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<2;   	//使能PORTC口时钟  
		RCC->APB1ENR|=1<<19;  		//使能串口4时钟 
	
		GPIO_Set(GPIOC, PIN10|PIN11, GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA0,PA1,复用功能,上拉输出
		GPIO_AF_Set(GPIOC,10,8);		//PA0,AF8
		GPIO_AF_Set(GPIOC,11,8);		//PA1,AF8 	
	
		//波特率设置
		UART4->BRR = mantissa; 	//波特率设置	 
		UART4->CR1 = 0x2008;  //只发送,使能串口4,使能发送中断
	
#if EN_UART4_RX		  	//如果使能了接收
		//使能接收中断 
		UART4->CR1|=1<<2;  	//串口接收使能
		UART4->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		//MY_NVIC_Init(3,3,UART4_IRQn,2);//组2，最低优先级 
#endif
}

