#include "sys.h"		    
#include "rs485.h"	 
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
 	
uint8_t USART_RX_BUF[USART_REC_LEN];  
uint8_t USART_IT_BUF[USART_REC_LEN]; // TEMP RX BUFFER
uint8_t *lrcResult = NULL;

#if !HAL_MODBUS_ENABLE

//MODBUS RTU 10ms
void USART6_IRQHandler(void)
{
	uint8_t res;
	if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)) //�����ж�
	{
		modbusPosi.g_RTU_Startflag = 1;   // ˢ�¶�ʱ��
		modbusPosi.g_10ms_Cnt = 0;		  // ��ʱ���¿�ʼ
	
		HAL_UART_Receive(&huart6, &res, 1, 50);
		USART_RX_BUF[modbusPosi.comRecvCnt++] = res;

		//���������
		if(modbusPosi.comRecvCnt >= USART_REC_LEN-1) {
			modbusPosi.comRecvCnt = 0;
		}
	}
} 
#endif

u16 g_RS485_CRC16Test(u8 *data, u8 num)
{
	u8 i,j,con1,con2;
	u16 CrcR = 0xffff, con3=0x00;
	for(i=0;i<num; i++)
	{
		con1=CrcR&0xff;
		con3=CrcR&0xff00;
		CrcR=con3+data[i]^con1;
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
	con1=CrcR>>8;//���ֽ�
	con2=CrcR&0xff;//���ֽ�
	CrcR=con2;
	CrcR=(CrcR<<8)+con1;
	return CrcR;
}

void g_RS485_sendPacket(UART_HandleTypeDef *husart, uint8_t packType, uint8_t *data)
{
	switch (packType) {
		case 0x01: HAL_RS485_Send_Data(husart, data, 8);
			break;
		default: 
			break;
	}
}

u32 g_RS485_recvDataDeal(void)
{
	u8 waitDealArray[USART_REC_LEN]={0};
	u16 testCRC16 = 0x0000;	
	u32 returnPosi = 0;
	static int rcvFrameCnt = 0;
	
	if (modbusPosi.comRecvCnt >= 9 && modbusPosi.comRecvCnt <= USART_REC_LEN-1) {
		memcpy (waitDealArray, USART_RX_BUF, modbusPosi.comRecvCnt);
		
		testCRC16 = waitDealArray[modbusPosi.comRecvCnt-2];
		testCRC16 = testCRC16 << 8;
		testCRC16 |= waitDealArray[modbusPosi.comRecvCnt-1];
	
		if (g_RS485_CRC16Test(waitDealArray, modbusPosi.comRecvCnt-2) == testCRC16) {
			switch(waitDealArray[1]) {
				case 0x03:
					
			#ifdef BIGLITTLE
				returnPosi |= waitDealArray[5];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[6];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[3];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[4];
				
			#else // AMG2000
				returnPosi |= waitDealArray[3];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[4];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[5];
				returnPosi <<= 8;
				returnPosi |= waitDealArray[6];
			#endif
				
				modbusPosi.l_recv_abs_posi_time = gTime.l_time_ms;
				modbusPosi.latest_abs_posi_um = returnPosi;

				// 由于初始缓冲区会被冲爆，所以这里的数据并不有效，需要进行进一步处理
				// if (rcvFrameCnt == 0) {
				// 	modbusPosi.l_init_abs_posi_time = gTime.l_time_ms;
				// 	modbusPosi.init_abs_posi_um = returnPosi;
				// }
				break;
				
				default:
				break;
			}
			rcvFrameCnt++;
		}
	} else {
		printf("%d ms RS485 recv data length overflow! \n\r", gTime.l_time_ms);
	}
	
	modbusPosi.comRecvCnt = 0;
	return returnPosi;
}

// HAL_RS485_Send
void HAL_RS485_Send_Data(UART_HandleTypeDef *husart, u8 *buf, u8 len)
{
	HAL_StatusTypeDef retStatus = HAL_OK;
	GPIO_SPI_RS485_RE_SET;
	retStatus = HAL_UART_Transmit(husart, buf, len, 50); 
	GPIO_SPI_RS485_RE_RESET;				

	if (retStatus != HAL_OK) {
		printf("RS485 Send Failed! \n\r");
	}
}

// Periodically Scanning Recv Reg
void UART_Byte_Receive(UART_HandleTypeDef *huart) 
{
  	HAL_UART_Receive_IT(huart, USART_IT_BUF, 1); // Enable Recv Interrupt
}

// RES485 Recv Data Logic Deal 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6) {
		modbusPosi.g_RTU_Startflag = 1; // Update Modbus RTU Recv Timing
		modbusPosi.g_10ms_Cnt++;

		USART_RX_BUF[modbusPosi.comRecvCnt++] = huart->Instance->DR; // Acquire One Byte
		if (modbusPosi.comRecvCnt >= USART_REC_LEN-1) {
			modbusPosi.comRecvCnt = 0;
		}
	}
}


#ifdef HAL_RS485_ENBALE 

//LRCУ�麯��
uint8_t* g_lrc_Test(uint8_t *StartAddr, uint8_t TestLen)
{
    uint8_t lrcResult = 0;
    uint8_t ii=0;
    uint8_t hexCnt = 0; 
    uint8_t hexCombine = 0; 
    uint8_t *dataPtr = StartAddr;
    static uint8_t hexArray[24] = {0};
    //1���ϲ�ΪHex
    for(ii =0; ii<TestLen; ii++)
    {
        if(ii % 2 == 0)//Hex�ϲ���ʼ
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
            //a-f������
            else
            {
								;
            }
            dataPtr++;
        }
        else if(ii % 2 == 1) //Hex�ϲ�ĩβ
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
    //2���ۼ�
    for(ii = 0; ii<hexCnt-1 ; ii++)
    {
        lrcResult += hexArray[ii];
    }
    //3��
    lrcResult = ~lrcResult; 
    lrcResult += 1;
    //4��У��LRC
    if(lrcResult == hexArray[hexCnt-1])
    {
        return hexArray;
    }
    else
    {
        return NULL;
    }
}

//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz),APB1һ��Ϊ48Mhz
//bound:������	  
void RS485_Init(u32 pclk1,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
		temp=(float)(pclk1*1000000)/(bound*16);//�õ�USARTDIV
		mantissa=temp;				 //�õ���������
		fraction=(temp-mantissa)*16; //�õ�С������	 
			mantissa<<=4;
		mantissa+=fraction; 
		
		//PCF8574_Init();				//��ʼ��PCF8574,���ڿ���RE��	
		
		RCC->AHB1ENR|=1<<0;   		//ʹ��PORTA��ʱ��   
		GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,���ù���,���� 
		GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	   
	 
		RCC->APB1ENR|=1<<17;  		//ʹ�ܴ���2ʱ��  
		RCC->APB1RSTR|=1<<17;   	//��λ����2
		RCC->APB1RSTR&=~(1<<17);	//ֹͣ��λ	   	   
		//����������
		USART2->BRR=mantissa; 		// ����������	 
		USART2->CR1|=0X200C;  		//1λֹͣ,��У��λ.
	#if EN_USART2_RX		  		//���ʹ���˽���
		//ʹ�ܽ����ж� 
		USART2->CR1|=1<<2;  		//���ڽ���ʹ��
		USART2->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	    	
		MY_NVIC_Init(3,3,USART2_IRQn,2);//��2��������ȼ� 
	#endif
		RS485_TX_Set(0);			//Ĭ������Ϊ����ģʽ	
}

//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf,u8 len)
{
		u8 t;
		RS485_RE = 1;				//����Ϊ����ģʽ
		for(t=0;t<len;t++)			//ѭ����������
		{
			while((USART2->SR&0X40)==0);//�ȴ����ͽ���		  
			USART2->DR=buf[t];
		}	 
		while((USART2->SR&0X40)==0);//�ȴ����ͽ���	
		RS485_RX_CNT=0;	  
		RS485_RE  = 0;			//����Ϊ����ģʽ	
}
//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data(u8 *buf,u8 *len)
{
		u8 rxlen=RS485_RX_CNT;
		u8 i=0;
		*len=0;				//Ĭ��Ϊ0
		//HAL_Delay(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս��� modbus RTU
		if(rxlen==RS485_RX_CNT&&rxlen)//���յ�������,�ҽ��������
		{
			for(i=0;i<rxlen;i++)
			{
					buf[i]=RS485_RX_BUF[i];	
			}		
			*len=RS485_RX_CNT;	//��¼�������ݳ���
			RS485_RX_CNT=0;		//����
		}
} 
//RS485ģʽ����.
//en:0,����;1,����.
void RS485_TX_Set(u8 en)
{
	RS485_RE = 1;
}

//USART2���ڳ�ʼ��
void uart2_RS485_init(u32 pclk2,u32 bound)
{
		float temp;
		u16 mantissa;
		u16 fraction;	 
	
		//�����ʴ���
		temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
		mantissa=temp;				 //�õ���������
		fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
				mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<6;     	//ʹ��PORTGʱ�� 
		GPIO_Set(GPIOG, PIN10, GPIO_MODE_OUT, 0, 2, GPIO_PUPD_PD);	//PG10����Ϊ���������50MHz��Ĭ������
		RS485_RE = 0; //Ĭ�Ͻ���
	
		RCC->AHB1ENR|=1<<0;   		//ʹ��PORTA��ʱ��  
		//ʹ��GPIO
		GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA2,PA3,���ù���,�������
		GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	
		//����ʱ��
		RCC->APB1ENR|=1<<17;  		//ʹ��USART2ʱ�� 
		RCC->APB1RSTR|=1<<17;   	//��λ����2
		RCC->APB1RSTR&=~(1<<17);	//ֹͣ��λ	
	
		//����������
		USART2->BRR=mantissa; 		//����������	
		//RS485��ʽ֡���� 
		USART2->CR1|=0X200C;  		//1λֹͣ,��У��λ������ʹ��

#if EN_USART2_RX		  				//���ʹ���˽���
		//ʹ�ܽ����ж� 
		USART2->CR1|=1<<2;  			//���ڽ���ʹ��
		USART2->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��
		MY_NVIC_Init(1,1,USART2_IRQn,2);//��ռ1������1�����ȼ���2
#endif
		RS485_RE = 0;			//Ĭ������Ϊ����ģʽ	  		
}


//����485�������� ASCII
void g_RS485_ComSendASCII(uint8_t *data, uint8_t data_Len)
{
	uint8_t ii =0;
	uint8_t lrc_Res = g_Modbus_lrc_SendTest(data, data_Len);
	uint8_t lrc_Low = 0;
	uint8_t lrc_High = 0;
	
	//1������LRC
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
	//2�����ݴ���
	RS485_RE = 1;			//��Ϣ����ģʽ
	//����
	while((USART2->SR&0x40)==0);		//�ȴ����ͽ���
	USART2->DR = 0x3A;
	while((USART2->SR&0x40)==0);		//�ȴ����ͽ���
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
	//����
	USART2->DR = 0x0D;
	while((USART2->SR&0X40)==0);
	USART2->DR = 0x0A;
	while((USART2->SR&0X40)==0);
	//3�����ͽ�������λΪ����ģʽ
	RS485_RE = 0;			//Ĭ������Ϊ����ģʽ	
}

//LRCУ�麯��
uint8_t g_Modbus_lrc_SendTest(uint8_t *StartAddr, uint8_t TestLen)
{
    static uint8_t lrcRes = 0;
    uint8_t ii=0;
    uint8_t hexCnt = 0; 
    uint8_t hexCombine = 0; 
    uint8_t *dataPtr = StartAddr;
    static uint8_t hexArray[24] = {0};
		//ÿ�ν���У��ǰ�������һ��ʣ��lrcResֵ
		lrcRes = 0;
    //1���ϲ�ΪHex
    for(ii =0; ii<TestLen; ii++) {
		//Hex�ϲ���ʼ 
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
            //a-f������
            else {
								;
            }
            dataPtr++;
        }
		//Hex�ϲ�ĩβ
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
    //2���ۼ�
    for(ii = 0; ii<hexCnt ; ii++) {
        lrcRes += hexArray[ii];
    }
    //3��ȡ����
    lrcRes = ~lrcRes; 
    lrcRes += 1;

    return lrcRes;
}

#endif

