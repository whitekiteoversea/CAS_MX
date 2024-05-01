#include "sys.h"
#include "usart.h"	
  
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)  
//���HAL��ʹ��ʱ,ĳЩ������ܱ����bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}

//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;      

//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� Ĭ��USART1
// int fputc(int ch, FILE *f)
// {      
// 	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
// 	USART1->DR = (u8) ch;      
// 	return ch;
// }

//�ض���fputc���� Ĭ��UART4
int fputc(int ch, FILE *f)
{      
	while((UART4->SR & 0X40)==0);//ѭ������,ֱ���������   
	UART4->DR = (u8) ch;      
	return ch;
}

#endif 


/*************���ڴ�����***************/

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���  
//MODBUS ASCII
void USART1_IRQHandler(void)
{
		char recvByte = 0; //����ط���Ϊ��������������ֻ����0x30-0x39,0x41-0x46
		uint8_t comNewFrameCnt = 0;   	//��ʶ�µ�һ֡���ݽ���
		uint8_t comNewRecvEndFlag = 0;  //���ڱ�ʶ��ǰ�����Ƿ�Ϊ�����ַ���ʼ 
		//������ڽ��ջ������ǿ�
		if(USART1->SR&(1<<5))
		{
				recvByte = USART1->DR; //��ȡ��ǰ�ֽ�
				//У��֡����
				if(recvByte == 0x3A)   //������ʼ
				{
						comNewFrameCnt = 1;
				}
				else if(recvByte == 0x0D)
				{
						comNewRecvEndFlag = 1;
				}
				else if(recvByte == 0x0A)
				{
						//��ֻ���ǵ�֡����
						if(comNewRecvEndFlag == 1)
						{
								//LRCУ�飨����תHex�����ӷ���
								lrcResult = g_lrc_Test(USART_RX_BUF, comRecvCnt);
								if(lrcResult != NULL)
								{
										comNewFrameCnt++;  //���յ�������һ֡
								}
								else if(lrcResult == NULL)
								{
										//У�鲻ͨ��������
								}
								comNewRecvEndFlag = 0;
						}
						//����
				}
				//�������ݷ�Χ
				else if(((0 <=(recvByte - '0')) && (0 >= (recvByte - '9'))) ||
								((0 <= (recvByte - 'A')) && (0 >=(recvByte -'F'))))
				{
						USART_RX_BUF[comRecvCnt++] = recvByte;
				}
		}
} 
#endif	


//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
	
		temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
		mantissa=temp;				 //�õ���������
		fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
		mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
		RCC->APB2ENR|=1<<4;  	//ʹ�ܴ���1ʱ�� 
	
		GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
		GPIO_AF_Set(GPIOA,9,7);	//PA9,AF7
		GPIO_AF_Set(GPIOA,10,7);//PA10,AF7  	
	
		//����������
		USART1->BRR=mantissa; 	//����������	 
		USART1->CR1&=~(1<<15); 	//����OVER8=0 
		USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 
	
#if EN_USART1_RX		  	//���ʹ���˽���
		//ʹ�ܽ����ж� 
		USART1->CR1|=1<<2;  	//���ڽ���ʹ��
		USART1->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
		//MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
		MY_NVIC_Init(1,1,USART1_IRQn,2);//��ռ1������1�����ȼ�
#endif
		USART1->CR1|=1<<13;  	//����ʹ��
}

//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
uint8_t UART4_SendBuf[200] = {0}; 

void uart4_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
	
		temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
		mantissa=temp;				 								 //�õ���������
		fraction=(temp-mantissa)*16;           //�õ�С������@OVER8=0 
		mantissa<<=4;
		mantissa+=fraction; 
	
		RCC->AHB1ENR|=1<<2;   	//ʹ��PORTC��ʱ��  
		RCC->APB1ENR|=1<<19;  		//ʹ�ܴ���4ʱ�� 
	
		GPIO_Set(GPIOC, PIN10|PIN11, GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA0,PA1,���ù���,�������
		GPIO_AF_Set(GPIOC,10,8);		//PA0,AF8
		GPIO_AF_Set(GPIOC,11,8);		//PA1,AF8 	
	
		//����������
		UART4->BRR = mantissa; 	//����������	 
		UART4->CR1 = 0x2008;  //ֻ����,ʹ�ܴ���4,ʹ�ܷ����ж�
	
#if EN_UART4_RX		  	//���ʹ���˽���
		//ʹ�ܽ����ж� 
		UART4->CR1|=1<<2;  	//���ڽ���ʹ��
		UART4->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
		//MY_NVIC_Init(3,3,UART4_IRQn,2);//��2��������ȼ� 
#endif
}

