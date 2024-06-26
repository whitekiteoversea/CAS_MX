#include "spi.h"
#include "sys.h"
#include "usart.h"

/*DAC8653��һ֡�������ֹ���
*  bit23-22 not use
*  bit21-19 cmd
*  bit18-16 addr
*  bit15-0  data
*/
#if HAL_ENABLE

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

	//SPI1�������� �������ڵ�SPI1����ѡ���Ǵ��ģ��ÿ��ο��ֲᣩ
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
	tempreg|=1<<9;			//����nss����
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

#endif


HAL_StatusTypeDef HAL_DAC8563_cmd_Write(u8 cmd, u8 addr, u16 data)
{
	u8 sndData[3]={0};
	u8 i =0;
	HAL_StatusTypeDef retStatus = 0;
	
	sndData[0] = (cmd << 3) | addr;
	sndData[1] = (u8)((data & 0xFF00) >> 8);
	sndData[2] = (u8)(data & 0x00FF);

	GPIO_SPI_DAC8563_SYNC_RESET; 
	for(i=0;i<3;i++)
	{
		retStatus = HAL_SPI_Transmit(&hspi1, &sndData[i], 1, 50);
	}
	GPIO_SPI_DAC8563_SYNC_SET;
	return retStatus;
}	

void HAL_DAC8563_Config(void)
{
	u8 rtData = 0;
	// Power up DAC-A  DAC-B
	rtData = HAL_DAC8563_cmd_Write(4, 0, 3);
	HAL_Delay(50);
	
	// LDAC pin inactive for DAC-B and DAC-A  
	//两个channel均不使用LDAC引脚更新数据  
	rtData = HAL_DAC8563_cmd_Write(6, 0, 3);
	HAL_Delay(50);

	// 复位DAC-A到0, 并更新输出为0V 
	HAL_DAC8563_cmd_Write(3, 0, spdDownLimitVol);
	HAL_Delay(50);

	// 使能内部参考并复位2个DAC的增益=2  
	HAL_DAC8563_cmd_Write(7, 0, 1);
	HAL_Delay(20);
}

void HAL_SPI1_DAC8563_Init(void)
{
	HAL_DAC8563_Config();
}


/* SPI4 W5500 相关函数*/
uint8_t HAL_SPI4_WriteAndReadByte(uint8_t TxData)
{
	HAL_StatusTypeDef retStatus = HAL_OK;
	unsigned char retData = 0;
	unsigned int timeToWait_Ms = 50;

	GPIO_SPI_W5500_CS_RESET; 
	retStatus = HAL_SPI_TransmitReceive(&hspi4, &TxData, &retData, 1, timeToWait_Ms);
	if (retStatus != HAL_OK) {
		printf("W5500: SPI4 WriteAndRecv Failed! \r\n");
	}
	GPIO_SPI_W5500_CS_SET; 
	return retData;
}

void HAL_SPI4_WriteByte(uint8_t TxData)
{
	HAL_StatusTypeDef retStatus = HAL_OK;
	unsigned int timeToWait_Ms = 50;

	// GPIO_SPI_W5500_CS_RESET; 
	retStatus = HAL_SPI_Transmit(&hspi4, &TxData, 1, timeToWait_Ms);
	if (retStatus != HAL_OK) {
		printf(" SPI4 Send Failed! \n\r");
	}
	// GPIO_SPI_W5500_CS_SET; 
}

uint8_t HAL_SPI4_ReadByte(void)
{
	unsigned char retData = 0xff;
	unsigned int timeToWait_Ms = 50;

	// GPIO_SPI_W5500_CS_RESET; 
	HAL_SPI_Receive(&hspi4, &retData, 1, timeToWait_Ms);
	// GPIO_SPI_W5500_CS_SET; 

	return retData; 
}

/**
  * @brief  进入临界区
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}
/**
  * @brief  退出临界区
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  片选信号输出低电平
  * @retval None
  */
void SPI4_CS_Select(void)
{
	GPIO_SPI_W5500_CS_RESET;
}
/**
  * @brief  片选信号输出高电平
  * @retval None
  */
void SPI4_CS_Deselect(void)
{
	GPIO_SPI_W5500_CS_SET;
}

/* SPI2 BISS-C*/

#ifdef HAL_BISSC_ENABLE
// MB4 Wrap_Function
void mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize)
{
	uint16_t sendCnt = 0;
	HAL_StatusTypeDef retStatus = HAL_OK;

	GPIO_SPI_BISSC_CS_RESET; 
	for (sendCnt=0; sendCnt < datasize; sendCnt++) {
		retStatus = HAL_SPI_TransmitReceive(&hspi2, &data_tx[sendCnt], &data_rx[sendCnt], 1, 50);
		if (retStatus != HAL_OK) {
			printf("BISS-C: SPI2 Send Failed! \n\r");
		}
	}
	GPIO_SPI_BISSC_CS_SET;
}

// 配置 SPI2 BISS-C 基本参数
void HAL_BISSC_Setup(void)
{
	//BiSS/SSI Interface
	uint8_t txData[3] = {0};
	uint8_t rData = 0;
	uint8_t curStatus = 0x00;
	
	HAL_Delay(500); // wait for sensor initial >350ms
	mb4_read_status(&curStatus, 1);
	printf("BISS-C: 0xF0 Status is 0x%x \n\r", curStatus);
	
	txData[0] = 0x01;
	mb4_write_registers(0xED, txData, 1); //CFGCH1=0x01 (BiSS C)

	txData[0] = 0x09; 
	mb4_write_registers(0xF5, txData, 1); //CFGIF=0x02  (RS422) Internal Clock Enabled

	//Single-Cycle Data: Data channel configuration
	txData[0] = 0x61;
	mb4_write_registers(0xC0, txData, 1); //ENSCD1=1, SCDLEN1=0x21 (26+2+6 bit)  从机0协议配置

	txData[0] = 0x06;
	mb4_write_registers(0xC1, txData, 1); //SELCRCS1=0x00, SCRCLEN1=0x06 (6-bit CRC polynomial 0x43)
	txData[0] = 0x00;
	mb4_write_registers(0xC2, txData, 1); //SCRCSTART1(7:0)=0x00 (CRC start value)
	txData[0] = 0x00;
	mb4_write_registers(0xC3, txData, 1); //SCRCSTART1(15:8)=0x00 (CRC start value)

	//Frame Control: Master configuration
	txData[0] = 0x04;
	mb4_write_registers(0xE6, txData, 1); //FREQS=0x04 (2 MHz) SPI Bandwidth
	txData[0] = 0x63;
	mb4_write_registers(0xE8, txData, 1); //FREQAGS=0x63 (10 kHz) 控制RS422的最小循环周期 也就是100us向传感器自动获取一次数据

	txData[0] = 0x10;
	mb4_write_registers(0xF4, txData, 1); // Initial Slave

	mb4_read_status(&curStatus, 1);
	printf("BISS-C: After Initial, 0xF0 Status is 0x%x \n\r", curStatus);

	//Reset SVALID flags
	txData[0] = 0x00;
	mb4_write_registers(0xF1, txData, 1);

	//Start AGS
	txData[0] = 0x01;
	mb4_write_registers(0xF4, txData, 1); // 使能对传感器数据自动获取
}

// 获取传感器过程数据
void HAL_SG_SenSorAcquire(uint8_t *SG_Data) 
{
	uint8_t cnt = 0;
	uint8_t curStatus = 0;
	uint8_t TxData[8] = {0};
	uint8_t RxData[8] = {0};
	uint8_t StatusInformationF0 = 0x00;
	uint8_t StatusInformationF1 = 0x00;

	curStatus = 0;
	mb4_read_registers(0xF0, &curStatus, 1);
	printf("BISS-C: Read 0xF0 Value: %x \n\r", curStatus);

	//Read Status Information register 0xF0, wait for end of transmission EOT=1
	while (( StatusInformationF0 & 0x01) == 0) {
		mb4_read_registers(0xF0, &StatusInformationF0, 1);
	}

	//Read and reset SVALID flags in Status Information register 0xF1
	mb4_read_registers(0xF1, &StatusInformationF1, 1);
	printf("BISS-C: Read 0xF1 Value: %x \n\r", StatusInformationF1);
	
	TxData[0] = 0x00;
	mb4_write_registers(0xF1, TxData, 1); // 读取状态后重置0xF1 SVALID1 reg状态

	if ((StatusInformationF0 & 0x70) == 0x70) { // REGEND, EOT均置1
		//If Status ok but SVALID flag is not set, restart AGS
		curStatus = (StatusInformationF1 & 0x02); // bit SVALID1
		if (curStatus == 0) { // if SVALID1 != 1 And ReStart AGS
			printf(" BISS-C Acquire SG Data Failed! SVALID1 is %d \n\r", curStatus);
			TxData[0] = 0x80;
			mb4_write_registers(0xF4, TxData, 1);
			TxData[0] = 0x01;
			mb4_write_registers(0xF4, TxData, 1);//Restart AGS
		} else {  //If Status ok and SVALID flag is set, read single-cycle data
			mb4_read_registers(0x00, SG_Data, 4);
			printf(" BISS-C: bit SVALID1 Set Success! \n\r");
		}
	}
	else {
		printf("Data Not Ready! 0xF0 Status is 0x%x \n\r", StatusInformationF0);
	}
	//If Status not ok, check data channel configuration 
}

void HAL_CTLRegsWrite_Slave0(uint8_t reg_addr, uint8_t reg_data)
{
	uint8_t TxData = 0x00;
	uint8_t MasterRegisterValue = 0x00;
	
	mb4_read_registers(0xF0, &MasterRegisterValue, 1);
	printf("Acquire regs 0xF0: %2x", MasterRegisterValue);
	
	TxData = (0x01 << 7) | reg_addr;  
	mb4_write_registers(0xE2, &TxData, 1); //写操作 写的Slave传感器目的寄存器地址:
	TxData = 0x00;
	mb4_write_registers(0xE3, &TxData, 1); //REGNUM=0x00 (only 1 slave registers) 向0xE3写0代表传输单个寄存器的设置值 

	TxData = 0x01;
	mb4_write_registers(0xE4, &TxData, 1); //CHSEL(1)=1 (selects channel 1 for control communication)
	TxData = 0xC0;
	mb4_write_registers(0xE5, &TxData, 1); //CTS=1 (register access), REGVERS=1 (BiSS C), SLAVEID=0 这里就已经决定了是向 Slave0的寄存器进行控制数据传输
	mb4_write_registers(0x80, &reg_data, 1); //RDATA=BankAddress 从0x80开始到0xBF 一共64字节，是MB4芯片的临时数据存储区域

	//Start register communication with slave
	TxData = 0x09;
	mb4_write_registers(0xF4, &TxData, 1); //AGS=1, INSTR=4 (start control communication) instruction = 4, 传感器数据自动获取 AutoGetSensorData =1
	
	mb4_read_registers(0xF4, &MasterRegisterValue, 1);
	
	//wait until INSTR (address 0xF4) is reset (AGS remains 1) （等待INSTR置位，除错处理）
	while (MasterRegisterValue > 1) {
		mb4_read_registers(0xF4, &MasterRegisterValue, 1); // 写入后读取当前寄存器值以验证是否写入成功
	}

	//Read status register
	//Register communication successful if nREGERR=1 (bit3) and REGEND=1 (bit2)
	mb4_read_registers(0xF0, &MasterRegisterValue, 1);
	if (!(MasterRegisterValue & 0xC0)){
		printf(" BISS-C Write Regs Failed!\n\r");
	}
}

// 单通信周期内1个寄存器内容的实例
uint8_t HAL_CTLRegs_Read_Slave0(uint8_t readAddr)
{
	uint8_t SlaveRegValue = 0x00;
	uint8_t MasterRegisterValue = 0x00;
	uint8_t txData[8] = {0};

	//Configuration for reading slave register
	txData[0] = 0x7E;
	mb4_write_registers(0xE2, txData, 1); //WNR=0 / REGADR=0x7E 选择为读：
	txData[0] = 0x01;
	mb4_write_registers(0xE3, txData, 1); //REGNUM=0x01 (2 consecutive slave registers)
	txData[0] = 0x01;
	mb4_write_registers(0xE4, txData, 1); //CHSEL(1)=1 (selects channel 1 for control communication)
	txData[0] = 0xC0;
	mb4_write_registers(0xE5, txData, 1); //CTS=1 (register access), REGVERS=1 (BiSS C), SLAVEID=0

	//Start slave register communication
	txData[0] = 0x09;
	mb4_write_registers(0xF4, txData, 1); //AGS=1 / INSTR=4 (start control communication)

	mb4_read_registers(0xF4, &MasterRegisterValue, 1);
	while (MasterRegisterValue > 1) {//wait until INSTR (address 0xF4) is reset (AGS remains 1)
		mb4_read_registers(0xF4, &MasterRegisterValue, 1);
	} 

	//Read status register
	//Register communication successful if nREGERR=1 (bit3) and REGEND=1 (bit2)
	mb4_read_registers(0xF0, &MasterRegisterValue, 1);

	//Read slave register values
	mb4_read_registers(readAddr, &SlaveRegValue, 1);

	return SlaveRegValue;
}

uint8_t BISSC_F0Status_Display(uint8_t status)
{
	uint8_t ret = 0;
	switch(status) {



		default:
		break;
	}


	return ret;
}


#endif
