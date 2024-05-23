#include "sdram.h"
#include "stm32f4xx_hal.h"
#include "hal_delay.h"

u8 SDRAM_Send_Cmd(u8 bankx,u8 cmd,u8 refresh,u16 regval)
{
    u32 target_bank=0;
    FMC_SDRAM_CommandTypeDef Command;
    
    if(bankx==0) target_bank=FMC_SDRAM_CMD_TARGET_BANK1;       
    else if(bankx==1) target_bank=FMC_SDRAM_CMD_TARGET_BANK2;   
    Command.CommandMode=cmd;                //命令
    Command.CommandTarget=target_bank;      //目标SDRAM存储区域
    Command.AutoRefreshNumber=refresh;      //自刷新次数
    Command.ModeRegisterDefinition=regval;  //要写入模式寄存器的值
    if(HAL_SDRAM_SendCommand(&hsdram1,&Command,0X1000)==HAL_OK) //向SDRAM发送命令
    {
        return 0;  
    }
    else return 1;  
} 

//发送SDRAM初始化序列
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram)
{
    u32 temp=0;
    //SDRAM控制器初始化完成以后还需要按照如下顺序初始化SDRAM
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_CLK_ENABLE,1,0); //时钟配置使能
    HAL_Delay_us(500);                              //at least wait for 200us to precharge SDRAM
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_PALL,1,0);       //对所有存储区预充电
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_AUTOREFRESH_MODE, 8, 0);//设置自刷新次数 
    //配置模式寄存器,SDRAM的bit0~bit2为指定突发访问的长度，
	//bit3为指定突发访问的类型，bit4~bit6为CAS值，bit7和bit8为运行模式
	//bit9为指定的写突发模式，bit10和bit11位保留位
	temp=(u32)SDRAM_MODEREG_BURST_LENGTH_1          |	//设置突发长度:1(可以是1/2/4/8)
              SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |	//设置突发类型:连续(可以是连续/交错)
              SDRAM_MODEREG_CAS_LATENCY_3           |	//设置CAS值:3(可以是2/3)
              SDRAM_MODEREG_OPERATING_MODE_STANDARD |   //设置操作模式:0,标准模式
              SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;     //设置突发写模式:1,单点访问
    SDRAM_Send_Cmd(0,FMC_SDRAM_CMD_LOAD_MODE,1,temp);   //设置SDRAM的模式寄存器
}


#if HAL_SDRAM_EANBLE
//SDRAM Init
void SDRAM_Init(void)
{ 
	u32 sdctrlreg=0,sdtimereg=0;
	u16 mregval=0;
	
	RCC->AHB3ENR|=1<<0;     	//ʹ��FMCʱ��  
	RCC->AHB1ENR|=0X1F<<2;		//ʹ��PC/PD/PE/PF/PGʱ��  
	
	GPIO_Set(GPIOC,PIN0|PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);			//PC0/2/3			
	GPIO_Set(GPIOD,3<<0|7<<8|3<<14,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);		//PD0/1/8/9/10/14/15		
	GPIO_Set(GPIOE,3<<0|0X1FF<<7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);			//PE0/1/7~15				
	GPIO_Set(GPIOF,0X3F<<0|0X1F<<11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);		//PG0~5/11~15					
	GPIO_Set(GPIOG,7<<0|3<<4|PIN8|PIN15,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);	//PF0~2/4/5/8/15				
	
	GPIO_AF_Set(GPIOC,0,12);	//PC0,AF12
	GPIO_AF_Set(GPIOC,2,12);	//PC2,AF12
	GPIO_AF_Set(GPIOC,3,12);	//PC3,AF12
	
	GPIO_AF_Set(GPIOD,0,12);	//PD0,AF12 
	GPIO_AF_Set(GPIOD,1,12);	//PD1,AF12 
	GPIO_AF_Set(GPIOD,8,12);	//PD8,AF12
	GPIO_AF_Set(GPIOD,9,12);	//PD9,AF12
	GPIO_AF_Set(GPIOD,10,12);	//PD10,AF12  
	GPIO_AF_Set(GPIOD,14,12);	//PD14,AF12
	GPIO_AF_Set(GPIOD,15,12);	//PD15,AF12
	
	GPIO_AF_Set(GPIOE,0,12);	//PE0,AF12 
	GPIO_AF_Set(GPIOE,1,12);	//PE1,AF12 
	GPIO_AF_Set(GPIOE,7,12);	//PE7,AF12
	GPIO_AF_Set(GPIOE,8,12);	//PE8,AF12
	GPIO_AF_Set(GPIOE,9,12);	//PE9,AF12
	GPIO_AF_Set(GPIOE,10,12);	//PE10,AF12
	GPIO_AF_Set(GPIOE,11,12);	//PE11,AF12
	GPIO_AF_Set(GPIOE,12,12);	//PE12,AF12
	GPIO_AF_Set(GPIOE,13,12);	//PE13,AF12
	GPIO_AF_Set(GPIOE,14,12);	//PE14,AF12
	GPIO_AF_Set(GPIOE,15,12);	//PE15,AF12

	GPIO_AF_Set(GPIOF,0,12);	//PF0,AF12 
	GPIO_AF_Set(GPIOF,1,12);	//PF1,AF12 
	GPIO_AF_Set(GPIOF,2,12);	//PF2,AF12
	GPIO_AF_Set(GPIOF,3,12);	//PF3,AF12
	GPIO_AF_Set(GPIOF,4,12);	//PF4,AF12
	GPIO_AF_Set(GPIOF,5,12);	//PF5,AF12
	GPIO_AF_Set(GPIOF,11,12);	//PF11,AF12
	GPIO_AF_Set(GPIOF,12,12);	//PF12,AF12
	GPIO_AF_Set(GPIOF,13,12);	//PF13,AF12
	GPIO_AF_Set(GPIOF,14,12);	//PF14,AF12
	GPIO_AF_Set(GPIOF,15,12);	//PF15,AF12
	
	GPIO_AF_Set(GPIOG,0,12);	//PG0,AF12 
	GPIO_AF_Set(GPIOG,1,12);	//PG1,AF12 
	GPIO_AF_Set(GPIOG,2,12);	//PG2,AF12
	GPIO_AF_Set(GPIOG,4,12);	//PG4,AF12
	GPIO_AF_Set(GPIOG,5,12);	//PG5,AF12  
	GPIO_AF_Set(GPIOG,8,12);	//PG8,AF12
	GPIO_AF_Set(GPIOG,15,12);	//PG15,AF12	
		
	sdctrlreg|=1<<0;				//9λ�е�ַ
	sdctrlreg|=2<<2;				//13λ�е�ַ
	sdctrlreg|=1<<4;				//16λ����λ��
	sdctrlreg|=1<<6;				//4���ڲ�����(4 BANKS)
	sdctrlreg|=2<<7;				//2��CAS�ӳ�
	sdctrlreg|=0<<9;				//����д����
	sdctrlreg|=2<<10;				//SDRAMʱ��=HCLK/2=192M/2=96M=10.4ns
	sdctrlreg|=1<<12;				//ʹ��ͻ������ 
	sdctrlreg|=0<<13;				//��ͨ���ӳ�0��HCLK
	FMC_Bank5_6->SDCR[0]=sdctrlreg;	//����FMC BANK5 SDRAM���ƼĴ���(BANK5��6���ڹ���SDRAM).

	sdtimereg|=1<<0;				//����ģʽ�Ĵ���������ʱ����ӳ�Ϊ2��ʱ������
	sdtimereg|=6<<4;				//�˳���ˢ���ӳ�Ϊ7��ʱ������
	sdtimereg|=5<<8;				//��ˢ��ʱ��Ϊ6��ʱ������
	sdtimereg|=5<<12;				//��ѭ���ӳ�Ϊ6��ʱ������
	sdtimereg|=1<<16;				//�ָ��ӳ�Ϊ2��ʱ������
	sdtimereg|=1<<20;				//��Ԥ����ӳ�Ϊ2��ʱ������
	sdtimereg|=1<<24;				//�е����ӳ�Ϊ2��ʱ������
	FMC_Bank5_6->SDTR[0]=sdtimereg;	//����FMC BANK5 SDRAMʱ��Ĵ��� 

	SDRAM_Send_Cmd(0,1,0,0);		//ʱ������ʹ��
	HAL_Delay_us(500);					//�����ӳ�200us.
	SDRAM_Send_Cmd(0,2,0,0);		//�����д洢��Ԥ���
	SDRAM_Send_Cmd(0,3,8,0);		//������ˢ�´��� 
	mregval|=1<<0;					//����ͻ������:1(������1/2/4/8)
	mregval|=0<<3;					//����ͻ������:����(����������/����)
	mregval|=2<<4;					//����CASֵ:2(������2/3)
	mregval|=0<<7;					//���ò���ģʽ:0,��׼ģʽ
	mregval|=1<<9;					//����ͻ��дģʽ:1,�������
	SDRAM_Send_Cmd(0,4,0,mregval);	//����SDRAM��ģʽ�Ĵ���
	
	//刷新频率计数器(以SDCLK频率计数),计算方法:
	//COUNT=SDRAM刷新周期/行数-20=SDRAM刷新周期(us)*SDCLK频率(Mhz)/行数
    //我们使用的SDRAM刷新周期为64ms,SDCLK=180/2=90Mhz,行数为8192(2^13).
	//所以,COUNT=64*1000*90/8192-20=683
	FMC_Bank5_6->SDRTR=730<<1;		//����ˢ��Ƶ�ʼ�����
} 

#endif


//在指定地址(WriteAddr+Bank5_SDRAM_ADDR)开始,连续写入n个字节.
//pBuffer:字节指针
//WriteAddr:要写入的地址
//n:要写入的字节数
void FMC_SDRAM_WriteBuffer(u8 *pBuffer,u32 WriteAddr,u32 n)
{
	for(;n!=0;n--)
	{
		*(vu8*)(Bank5_SDRAM_ADDR+WriteAddr)=*pBuffer;
		WriteAddr++;
		pBuffer++;
	}
}

//在指定地址((WriteAddr+Bank5_SDRAM_ADDR))开始,连续读出n个字节.
//pBuffer:字节指针
//ReadAddr:要读出的起始地址
//n:要写入的字节数
void FMC_SDRAM_ReadBuffer(u8 *pBuffer,u32 ReadAddr,u32 n)
{
	for(;n!=0;n--)
	{
		*pBuffer++=*(vu8*)(Bank5_SDRAM_ADDR+ReadAddr);
		ReadAddr++;
	}
}






























