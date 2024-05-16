#include "sys.h"		    
#include "rs485.h"	 
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
 	
uint8_t USART_RX_BUF[USART_REC_LEN];     		//接收缓冲,最大USART_REC_LEN个字节.
												//接收状态
												//bit15，	接收完成标志
												//bit14，	接收到0x0d
												//bit13~0，	接收到的有效字节数目
uint8_t *lrcResult = NULL; //用于获取转化为Hex文件后的串口数据（需要及时读取，不然会被覆盖）

//MODBUS RTU 10ms断帧
void USART6_IRQHandler(void)
{
	uint8_t res;
	//如果串口接收缓冲区非空
	if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)) //接收中断
	{
		modbusPosi.g_RTU_Startflag = 1;   // 刷新定时器
		modbusPosi.g_10ms_Cnt = 0;		  // 计时重新开始
	
		HAL_UART_Receive(&huart6, &res, 1, 50);
		USART_RX_BUF[modbusPosi.comRecvCnt++] = res;

		//缓存区溢出
		if(modbusPosi.comRecvCnt >= USART_REC_LEN-1) {
			modbusPosi.comRecvCnt = 0;
		}
	}
} 

//CRC16校验码生成
u16 g_RS485_CRC16Test(u8 *data, u8 num)
{
	u8 i,j,con1,con2;
	u16 CrcR = 0xffff, con3=0x00;
	for(i=0;i<num; i++)
	{
		//把第一个8位二进制数据（既通讯信息帧的第一个字节）与16位的CRC寄存器的低
		//8位相异或，把结果放于CRC寄存器，高八位数据不变；
		con1=CrcR&0xff;
		con3=CrcR&0xff00;
		CrcR=con3+data[i]^con1;
		//把CRC寄存器的内容右移一位（朝低位）用0填补最高位，并检查右移后的移出位；
		for(j=0;j<8;j++)
		{
			con2=CrcR&0x0001;
			CrcR=CrcR>>1;
			if(con2==1)
			{
				CrcR=CrcR^0xA001;
			}
		}
	}
	con1=CrcR>>8;//高字节
	con2=CrcR&0xff;//低字节
	CrcR=con2;
	CrcR=(CrcR<<8)+con1;
	return CrcR;
}
//USART2 RS485报文查询下发
void g_RS485_sendPacket(UART_HandleTypeDef *husart, uint8_t packType, uint8_t *data)
{
	switch (packType) {
		case 0x01: HAL_RS485_Send_Data(husart, data, 8);
			break;
		default: 
			break;
	}
}

//串口6 磁栅尺 Modbus RTU 接收数据解包处理
u32 g_RS485_recvDataDeal(void)
{
	u8 waitDealArray[USART_REC_LEN]={0};
	u16 testCRC16 = 0x0000;	
	u32 returnPosi = 0;
	static int rcvFrameCnt = 0;
	
	//单次接收到数据长度小于缓存区
	if (modbusPosi.comRecvCnt >= 9 && modbusPosi.comRecvCnt <= USART_REC_LEN-1) {
		memcpy (waitDealArray, USART_RX_BUF, modbusPosi.comRecvCnt);
		
		testCRC16 = waitDealArray[modbusPosi.comRecvCnt-2];
		testCRC16 = testCRC16 << 8;
		testCRC16 |= waitDealArray[modbusPosi.comRecvCnt-1];
	
		if (g_RS485_CRC16Test(waitDealArray, modbusPosi.comRecvCnt-2) == testCRC16) {
			//找到包头，查询结果03
			switch(waitDealArray[1]) {
				//查询报文回复解包：这里只查位移
				case 0x03:
				//3-4 低 5-6 高
				returnPosi |= waitDealArray[5];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[6];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[3];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[4];
				
				modbusPosi.l_recv_abs_posi_time = gTime.l_time_ms;
				modbusPosi.latest_abs_posi_um = returnPosi;
				
				//首帧接收
				if(rcvFrameCnt == 0) {
					modbusPosi.l_init_abs_posi_time = gTime.l_time_ms;
					modbusPosi.init_abs_posi_um = returnPosi;
				}
				break;
				
				default:
				break;
			}
			rcvFrameCnt++;
		}
	} else {
		printf("%d ms RS485 recv data length overflow! \n\r", gTime.l_time_ms);
	}
	
	//单条解析完成，直接清零
	modbusPosi.comRecvCnt = 0;
	return returnPosi;
}

// HAL_RS485_Send
void HAL_RS485_Send_Data(UART_HandleTypeDef *husart, u8 *buf, u8 len)
{
	HAL_StatusTypeDef retStatus = HAL_OK;
	GPIO_SPI_RS485_RE_SET;
	retStatus = HAL_UART_Transmit(husart, buf, len, 50); // 串口发送数据
	GPIO_SPI_RS485_RE_RESET;				 // 设置为接收模式	

	if (retStatus != HAL_OK) {
		printf("RS485 Send Failed! \n\r");
	}
}


#ifdef HAL_RS485_ENBALE 

//LRC校验函数
uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen)
{
    uint8_t lrcResult = 0;
    uint8_t ii=0;
    uint8_t hexCnt = 0; 
    uint8_t hexCombine = 0; 
    uint8_t *dataPtr = StartAddr;
    static uint8_t hexArray[24] = {0};
    //1、合并为Hex
    for(ii =0; ii<TestLen; ii++)
    {
        if(ii % 2 == 0)//Hex合并起始
        {
            //1-9
            if((*dataPtr >= '0')&&(*dataPtr <= '9'))
            {
                hexCombine = (*dataPtr - '0');
                hexCombine <<= 4;
            }
            //A-F
            else if((*dataPtr >= 'A')&&(*dataPtr <= 'F'))
            {
                hexCombine = (*dataPtr - 'A')+10;
                hexCombine <<= 4;
            }
            //a-f不处理
            else
            {
								;
            }
            dataPtr++;
        }
        else if(ii % 2 == 1) //Hex合并末尾
        {
            //1-9
            if((*dataPtr >= '0')&&(*dataPtr <= '9'))
            {
								hexCombine |= (*dataPtr - '0');
								hexArray[hexCnt] = hexCombine;
								hexCnt++;
            }
            else if((*dataPtr >= 'A')&&(*dataPtr <= 'F'))
            {
								hexCombine |= (*dataPtr - 'A'+10);
								hexArray[hexCnt] = hexCombine;
								hexCnt++;
            }
            else
            {
                ;
            }
            dataPtr++;
        }	
        else
        {
            ;
        }
    }
    //2、累加
    for(ii = 0; ii<hexCnt-1 ; ii++)
    {
        lrcResult += hexArray[ii];
    }
    //3、
    lrcResult = ~lrcResult; 
    lrcResult += 1;
    //4、校验LRC
    if(lrcResult == hexArray[hexCnt-1])
    {
        return hexArray;
    }
    else
    {
        return NULL;
    }
}

//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz),APB1一般为48Mhz
//bound:波特率	  
void RS485_Init(u32 pclk1,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
		temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分	 
			mantissa<<=4;
		mantissa+=fraction; 
		
		//PCF8574_Init();				//初始化PCF8574,用于控制RE脚	
		
		RCC->AHB1ENR|=1<<0;   		//使能PORTA口时钟   
		GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,复用功能,上拉 
		GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	   
	 
		RCC->APB1ENR|=1<<17;  		//使能串口2时钟  
		RCC->APB1RSTR|=1<<17;   	//复位串口2
		RCC->APB1RSTR&=~(1<<17);	//停止复位	   	   
		//波特率设置
		USART2->BRR=mantissa; 		// 波特率设置	 
		USART2->CR1|=0X200C;  		//1位停止,无校验位.
	#if EN_USART2_RX		  		//如果使能了接收
		//使能接收中断 
		USART2->CR1|=1<<2;  		//串口接收使能
		USART2->CR1|=1<<5;    		//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 
	#endif
		RS485_TX_Set(0);			//默认设置为接收模式	
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
		u8 t;
		RS485_RE = 1;				//设置为发送模式
		for(t=0;t<len;t++)			//循环发送数据
		{
			while((USART2->SR&0X40)==0);//等待发送结束		  
			USART2->DR=buf[t];
		}	 
		while((USART2->SR&0X40)==0);//等待发送结束	
		RS485_RX_CNT=0;	  
		RS485_RE  = 0;			//设置为接收模式	
}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf,u8 *len)
{
		u8 rxlen=RS485_RX_CNT;
		u8 i=0;
		*len=0;				//默认为0
		//HAL_Delay(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束 modbus RTU
		if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
		{
			for(i=0;i<rxlen;i++)
			{
					buf[i]=RS485_RX_BUF[i];	
			}		
			*len=RS485_RX_CNT;	//记录本次数据长度
			RS485_RX_CNT=0;		//清零
		}
} 
//RS485模式控制.
//en:0,接收;1,发送.
void RS485_TX_Set(u8 en)
{
	RS485_RE = 1;
}

//USART2串口初始化
void uart2_RS485_init(u32 pclk2,u32 bound)
{
		float temp;
		u16 mantissa;
		u16 fraction;	 
	
		//波特率处理
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
				mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<6;     	//使能PORTG时钟 
		GPIO_Set(GPIOG, PIN10, GPIO_MODE_OUT, 0, 2, GPIO_PUPD_PD);	//PG10设置为推挽输出，50MHz，默认下拉
		RS485_RE = 0; //默认接收
	
		RCC->AHB1ENR|=1<<0;   		//使能PORTA口时钟  
		//使能GPIO
		GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA2,PA3,复用功能,上拉输出
		GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	
		//开启时钟
		RCC->APB1ENR|=1<<17;  		//使能USART2时钟 
		RCC->APB1RSTR|=1<<17;   	//复位串口2
		RCC->APB1RSTR&=~(1<<17);	//停止复位	
	
		//波特率设置
		USART2->BRR=mantissa; 		//波特率设置	
		//RS485格式帧设置 
		USART2->CR1|=0X200C;  		//1位停止,无校验位，串口使能

#if EN_USART2_RX		  				//如果使能了接收
		//使能接收中断 
		USART2->CR1|=1<<2;  			//串口接收使能
		USART2->CR1|=1<<5;    		//接收缓冲区非空中断使能
		MY_NVIC_Init(1,1,USART2_IRQn,2);//抢占1，组内1，优先级组2
#endif
		RS485_RE = 0;			//默认设置为接收模式	  		
}


//发送485控制数据 ASCII
void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len)
{
	uint8_t ii =0;
	uint8_t lrc_Res = g_Modbus_lrc_SendTest(data, data_Len);
	uint8_t lrc_Low = 0;
	uint8_t lrc_High = 0;
	
	//1、计算LRC
	ii = (lrc_Res & 0x0F);
	if(((ii-0) >= 0) && ((ii-9) <= 0))
	{
		lrc_Low = ii + 0x30;
	}
	else if(((ii - 0x0A) >= 0) && ((ii -0x0F) <= 0))
	{
		lrc_Low = ii + 0x37;
	}
	else
	{
		;
	}
	
	ii = (lrc_Res >> 4) & 0x0F;
	if(((ii - 0) >= 0) && ((ii - 9) <= 0))
	{
		lrc_High = ii + 0x30;
	}
	else if((ii >= 0x0A) && (ii <= 0x0F))
	{
		lrc_High = ii + 0x37;
	}
	else
	{
		;
	}
	//2、数据传输
	RS485_RE = 1;			//信息发送模式
	//发送
	while((USART2->SR&0x40)==0);		//等待发送结束
	USART2->DR = 0x3A;
	while((USART2->SR&0x40)==0);		//等待发送结束
	for(ii = 0;ii<data_Len; ii++)
	{
		USART2->DR = data[ii];
		while((USART2->SR&0x40)==0);
	}
	//LRC
	USART2->DR = lrc_High;
	while((USART2->SR&0x40)==0);
	USART2->DR = lrc_Low;
	while((USART2->SR&0x40)==0);
	//结束
	USART2->DR = 0x0D;
	while((USART2->SR&0X40)==0);
	USART2->DR = 0x0A;
	while((USART2->SR&0X40)==0);
	//3、发送结束，置位为接收模式
	RS485_RE = 0;			//默认设置为接收模式	
}

//LRC校验函数
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen)
{
    static uint8_t lrcRes = 0;
    uint8_t ii=0;
    uint8_t hexCnt = 0; 
    uint8_t hexCombine = 0; 
    uint8_t *dataPtr = StartAddr;
    static uint8_t hexArray[24] = {0};
		//每次进入校验前先清除上一次剩余lrcRes值
		lrcRes = 0;
    //1、合并为Hex
    for(ii =0; ii<TestLen; ii++) {
		//Hex合并起始 
        if(ii % 2 == 0) {
            //1-9
            if((*dataPtr >= '0')&&(*dataPtr <= '9')){
                hexCombine = (*dataPtr - '0');
                hexCombine <<= 4;
            }
            //A-F
            else if((*dataPtr >= 'A')&&(*dataPtr <= 'F')) {
                hexCombine = (*dataPtr - 'A')+10;
                hexCombine <<= 4;
            }
            //a-f不处理
            else {
								;
            }
            dataPtr++;
        }
		//Hex合并末尾
        else if(ii % 2 == 1) {
            //1-9
            if((*dataPtr >= '0')&&(*dataPtr <= '9')) {
				hexCombine |= (*dataPtr - '0');
				hexArray[hexCnt] = hexCombine;
				hexCnt++;
            }
            else if((*dataPtr >= 'A')&&(*dataPtr <= 'F')) {
				hexCombine |= (*dataPtr - 'A'+10);
				hexArray[hexCnt] = hexCombine;
				hexCnt++;
            }
            else {
                ;
            }
            dataPtr++;
        }	
        else {
            ;
        }
    }
    //2、累加
    for(ii = 0; ii<hexCnt ; ii++) {
        lrcRes += hexArray[ii];
    }
    //3、取补码
    lrcRes = ~lrcRes; 
    lrcRes += 1;

    return lrcRes;
}

#endif

