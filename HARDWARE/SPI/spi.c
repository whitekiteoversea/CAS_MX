#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "global_data.h"

#if HAL_DAC_ENABLE

/*DAC8653
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

		// 
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

#else

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

#endif



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
	uint8_t readAddr = 0x00;
	
	txData[0] = 0x01;
	mb4_write_registers(0xED, txData, 1); //CFGCH1=0x01 (BiSS C)
	txData[0] = 0x09; 
	mb4_write_registers(0xF5, txData, 1); //CFGIF=0x02  (RS422) + internal clock Source CLKIN =1

#if BISS_ENABLE_CRC
	//Single-Cycle Data: Data channel configuration
	mb4_write_registers(0xC0, txData, 1); // bit6:ENSCD1=1, bit5-0：SCD data len 26+2= 28, SCDLEN = 0x1C
	txData[0] = 0x06;
	mb4_write_registers(0xC1, txData, 1); //SELCRCS1=0x00, SCRCLEN1=0x06 (6-bit CRC polynomial 0x43)
	txData[0] = 0x00;
	mb4_write_registers(0xC2, txData, 1); //SCRCSTART1(7:0)=0x00 (CRC start value)
	txData[0] = 0x00;
	mb4_write_registers(0xC3, txData, 1); //SCRCSTART1(15:8)=0x00 (CRC start value)
#else
	//Single-Cycle Data: Data channel configuration
	txData[0] = 0x61;
	mb4_write_registers(0xC0, txData, 1); // bit6:ENSCD1=1, bit5-0：SCD data len 26+2+6= 34, SCDLEN = 34-1 = 0x21
	// disable CRC
	txData[0] = 0x00;
	mb4_write_registers(0xC1, txData, 1); //SELCRCS1=0x00, SCRCLEN1=0x00 
#endif

	//Frame Control: Master configuration
	txData[0] = 0x4;
	mb4_write_registers(0xE6, txData, 1); //FREQS=0x04 (2MHz) SPI Bandwidth
	txData[0] = 0x63;
	mb4_write_registers(0xE8, txData, 1); //FREQAGS=10KHz 控制RS422的最小循环周期 

	//Reset SVALID flags
	txData[0] = 0x00;
	mb4_write_registers(0xF1, txData, 1);
}

void HAL_BISSC_StartAGS(void) 
{
	uint8_t txData= 0x01;
	mb4_write_registers(0xF4, &txData, 1);
}

void HAL_BISSC_reStartAGS(void)
{
	uint8_t txData= 0;
	txData = 0x80;
	mb4_write_registers(0xF4, &txData, 1);// BREAK INSTR
	txData = 0x01;
	mb4_write_registers(0xF4, &txData, 1);// AGS RESET
}

/*
 * iC-MB4 独立调试：
 *   - 不启动 AGS，避免 2 kHz 业务轮询和自动恢复覆盖故障现场；
 *   - 先验证 SPI 主机接口，再按 AN3/AN4 建议清理并配置寄存器；
 *   - 使用项目原有的 2 MHz MA 时钟执行单次 BiSS-C 采集。
 */
static void IC_MB4_DebugHostInterface(void)
{
	uint8_t id[2] = {0};
	uint8_t writeValue = 0xA5;
	uint8_t readValue = 0;

	mb4_read_registers(0xEA, id, 2);
	mb4_write_registers(0x00, &writeValue, 1);
	mb4_read_registers(0x00, &readValue, 1);

	printf("MB4 HOST: REV(EA)=0x%02X VER(EB)=0x%02X RAM00 write/read=0x%02X/0x%02X %s\r\n",
		id[0], id[1], writeValue, readValue,
		((id[1] == 0x84U) && (readValue == writeValue)) ? "PASS" : "FAIL");
}

void IC_MB4_DebugSetup(void)
{
	uint8_t zeroSCD[64] = {0};
	uint8_t zeroSlaveCfg[32] = {0};
	uint8_t zeroControl[6] = {0};
	uint8_t zeroChannel[3] = {0};
	uint8_t value = 0;
	uint8_t instruction = 0x80;
	uint8_t cfg[9] = {0};

	IC_MB4_DebugHostInterface();

	/* RS-422 interface + internal 20 MHz oscillator. */
	value = 0x09;
	mb4_write_registers(0xF5, &value, 1);

	/* BiSS BREAK; protocol requires at least 40 us before the next frame. */
	mb4_write_instruction(&instruction, 1);
	HAL_Delay(1);

	/* Clear volatile data/configuration areas recommended by AN3. */
	mb4_write_registers(0x00, zeroSCD, sizeof(zeroSCD));
	mb4_write_registers(0xC0, zeroSlaveCfg, sizeof(zeroSlaveCfg));
	mb4_write_registers(0xE0, zeroControl, sizeof(zeroControl));
	mb4_write_registers(0xEC, zeroChannel, sizeof(zeroChannel));

	/* Channel 1 = BiSS-C. */
	value = 0x01;
	mb4_write_registers(0xED, &value, 1);

	/*
	 * AMG2000: 26-bit position + 2 status bits + 6 CRC bits.
	 * CRC is deliberately treated as normal data because the existing sensor
	 * and MB4 CRC result are known not to agree.
	 */
	value = 0x61;
	mb4_write_registers(0xC0, &value, 1);
	value = 0x00;
	mb4_write_registers(0xC1, &value, 1);

	/* Restore the known 2 MHz sensor clock; AGS remains disabled in debug mode. */
	value = 0x04;
	mb4_write_registers(0xE6, &value, 1);
	value = 0x00;
	mb4_write_registers(0xE7, &value, 1);
	value = 0x9F;
	mb4_write_registers(0xE8, &value, 1);

	/* Clear all SVALID flags before the first explicit single cycle. */
	value = 0x00;
	mb4_write_registers(0xF1, &value, 1);
	mb4_write_registers(0xF2, &value, 1);

	mb4_read_registers(0xEA, &cfg[0], 2); /* revision, version */
	mb4_read_registers(0xED, &cfg[2], 1); /* CFGCH */
	mb4_read_registers(0xF5, &cfg[3], 1); /* CFGIF/CLKENI */
	mb4_read_registers(0xC0, &cfg[4], 2); /* SCD configuration */
	mb4_read_registers(0xE6, &cfg[6], 3); /* FREQS/FREQR, SINGLEBANK, FREQAGS */

	printf("MB4 CFG : EA=%02X EB=%02X ED=%02X F5=%02X C0=%02X C1=%02X E6=%02X E7=%02X E8=%02X\r\n",
		cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5], cfg[6], cfg[7], cfg[8]);
}

void IC_MB4_DebugOneShot(void)
{
	uint8_t statusF0 = 0;
	uint8_t statusF1 = 0;
	uint8_t statusF4 = 0;
	uint8_t statusF8Before = 0;
	uint8_t statusF8 = 0;
	uint8_t recoverF0 = 0;
	uint8_t recoverF4 = 0;
	uint8_t recoverF8 = 0;
	uint8_t sg[5] = {0};
	uint8_t value = 0;
	uint8_t instruction = 0x04; /* INSTR=2: one sensor-data cycle, CDM=0. */
	uint32_t startTick = 0;
	uint32_t rawPosition = 0;

	/* Capture the idle level before triggering a new BiSS-C cycle. */
	mb4_read_registers(0xF8, &statusF8Before, 1);

	/* Clear the old SVALID result, then trigger exactly one BiSS-C cycle. */
	mb4_write_registers(0xF1, &value, 1);
	mb4_write_instruction(&instruction, 1);

	startTick = HAL_GetTick();
	do {
		mb4_read_status(&statusF0, 1);
	} while (((statusF0 & 0x01U) == 0U) &&
		 ((HAL_GetTick() - startTick) < 20U));

	mb4_read_registers(0xF1, &statusF1, 1);
	mb4_read_registers(0xF4, &statusF4, 1);
	mb4_read_registers(0xF8, &statusF8, 1);

	printf("MB4 STAT: F0=%02X F1=%02X F4=%02X F8=%02X | EOT=%u nERR=%u nAGSERR=%u nDELAYERR=%u nSCDERR=%u SVALID1=%u SL1pre=%u SL1=%u | PIN EOT=%u NER=%u\r\n",
		statusF0, statusF1, statusF4, statusF8,
		(statusF0 >> 0) & 1U, (statusF0 >> 7) & 1U,
		(statusF0 >> 6) & 1U, (statusF0 >> 5) & 1U,
		(statusF0 >> 4) & 1U, (statusF1 >> 1) & 1U,
		statusF8Before & 1U, statusF8 & 1U,
		(unsigned int)HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2),
		(unsigned int)HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_4));

	/* Single-cycle validity does not depend on the AGS status bit. */
	if (((statusF0 & 0x11U) != 0x11U) || ((statusF1 & 0x02U) == 0U)) {
		printf("MB4 DATA: invalid; snapshot saved above, issuing BREAK for next independent test\r\n");

		/*
		 * A manual SCD command can wait indefinitely for ACK/START.  Abort it
		 * only after the diagnostic snapshot so the next call starts a new frame.
		 */
		instruction = 0x80;
		mb4_write_instruction(&instruction, 1);
		HAL_Delay(1); /* More than the BiSS minimum BREAK/timeout interval. */

		mb4_read_status(&recoverF0, 1);
		mb4_read_registers(0xF4, &recoverF4, 1);
		mb4_read_registers(0xF8, &recoverF8, 1);
		printf("MB4 RECOVER: F0=%02X F4=%02X F8=%02X | EOT=%u SL1=%u\r\n",
			recoverF0, recoverF4, recoverF8,
			recoverF0 & 1U, recoverF8 & 1U);
		return;
	}

	mb4_read_registers(0x00, sg, sizeof(sg));
	rawPosition = (((uint32_t)sg[4] & 0x03U) << 24) |
		((uint32_t)sg[3] << 16) |
		((uint32_t)sg[2] << 8) |
		(uint32_t)sg[1];

	printf("MB4 DATA: SG[0..4]=%02X %02X %02X %02X %02X raw26=%lu statusBits=%u\r\n",
		sg[0], sg[1], sg[2], sg[3], sg[4],
		(unsigned long)rawPosition, (unsigned int)((sg[0] >> 6) & 0x03U));
}

// 获取传感器过程数据
uint8_t HAL_SG_SenSorAcquire(uint32_t *pSG_Data) 
{
	uint8_t cnt = 0;
	uint8_t StatusInformationF0 = 0x00;
	uint8_t StatusInformationF1 = 0x00;
	uint8_t SG_Data_Tmp[5];
	uint8_t ret = 0;
	uint64_t SGGData = 0;
	uint8_t txData = 0;

	//Read Status Information register 0xF0, wait for end of transmission EOT=1
	mb4_read_status(&StatusInformationF0, 1);
	if ((StatusInformationF0 & 0x01) == 0) { 
		goto __end;
	}

	mb4_read_registers(0xF1, &StatusInformationF1, 1);
	txData = 0;
	mb4_write_registers(0xF1, &txData, 1);

	if ((StatusInformationF0 & 0x70) != 0x70 ) { // SCDERR OR AGSERR
		printf("BISS-C: Step 2 ERR Occur! nAGSERR is %d nSCDERR is %d, reStart AGS! \n\r", ((StatusInformationF0 & 0x40) >> 6), ((StatusInformationF0 & 0x10) >> 4));
		// 检查数据通道设置
		HAL_BISSC_reStartAGS();
		goto __end;	
	}

	if ((StatusInformationF1 & 0x02) == 0) { // if SVALID1 != 1 ReStart AGS
		printf(" BISS-C:Step 5 Acquire SG Data Failed! StatusInformationF1 is %d \n\r", StatusInformationF1);
		HAL_BISSC_reStartAGS();
		goto __end;
	} 
	// 状态位检查均通过，则获取SGData结果
	// 关于关闭CRC前后的SGGData数据位置，有以下结论：
	// 1、IC-MB4对关闭CRC的处理方式是将对应CRC6作为数据位接收，
	// 这会导致实际SG[0] bit5:0 是CRC6 bit 7:6 是CRC状态位，因此只倒序读SG4 bit 1-0 + SG[3-1]反而是 26bit 数据位
	// 2、如果开启CRC校验，则26bit数据位会处于 SG[2] bit 3-0 + SG[1]+SG[0] bit 7-2 而CRC 状态位为 SG[0] bit1-0
	// CRC6结果为 SG[7] bit 7-2

	// 2024.07.02 确认AMG2000发送的CRC6计算结果有问题，IC-MB4的CRC校验不能开
	// 这里是关闭CRC6的数据获取，如果开启CRC6则不能这样获取
	for (cnt = 0; cnt<5; cnt++) {
		mb4_read_registers(cnt, &SG_Data_Tmp[cnt], 1);
	}
	SGGData = (SG_Data_Tmp[4] & 0x03);
	SGGData <<= 8;
	for (cnt = 3;cnt>0; cnt--) {
		SGGData += SG_Data_Tmp[cnt];
		if (cnt > 1) {
			SGGData <<= 8;
		}
	}
	*pSG_Data = (uint32_t)SGGData;// 数据合规性校验在外面做，逻辑解耦
	// printf("BISS-C: Acquire Posi is %d \r\n", *pSG_Data);
	
__end:
	return ret;
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

#endif
