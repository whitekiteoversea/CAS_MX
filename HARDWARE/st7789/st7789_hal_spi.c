#include "st7789_hal_spi.h"

#if USE_DMA_SPI
// #include "hal_spi_dma.h"
#endif // USE_DMA_SPI

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t hal_spi_rw_data(SPI_HandleTypeDef *hspi, uint16_t TxData)
{
	HAL_StatusTypeDef retStatus;
	uint8_t sendByte = (uint8_t)TxData;
    uint32_t timeout_ms = 50; // 50ms ��ʱ
    
    retStatus = HAL_SPI_Transmit(hspi, &sendByte,  1,  timeout_ms);
    if (retStatus != HAL_OK) {
        return retStatus;
    }

	//HAL_SPI_TransmitReceive(hspi, &sendByte, &retData, 1,  timeout_ms);
    return 0; //����ͨ��SPIx������յ�����
}
/******************************************************************************************/
//����Ļдһ��8λָ��
void hal_spi_write_cmd(SPI_HandleTypeDef *hspi, uint8_t cmd)
{
	//SPI д����ʱ��ʼ
    GPIO_SPI_CS_CLR;
    GPIO_SPI_DC_CLR;
    hal_spi_rw_data(hspi, cmd);
    GPIO_SPI_CS_SET;
}

//��SPI���ߴ���һ��8λ����
void hal_spi_write_8bit_data(SPI_HandleTypeDef *hspi, uint8_t data)
{
	//SPI д����ʱ��ʼ
    GPIO_SPI_CS_CLR;
    GPIO_SPI_DC_SET;
	hal_spi_rw_data(hspi, data);
	GPIO_SPI_CS_SET;
}

//����Ļдһ��16λ����
void hal_spi_write_16bit_data(SPI_HandleTypeDef *hspi, uint16_t data)
{	
     GPIO_SPI_CS_CLR;
     GPIO_SPI_DC_SET;

	//д���8λ����
	hal_spi_rw_data(hspi, data>>8);
	//д���8λ����
	hal_spi_rw_data(hspi, data);
	
     GPIO_SPI_CS_SET;
}

//��Ļ��λ
void hal_spi_hard_reset(void)
{
    GPIO_SPI_RST_CLR;//RST�������Ϊ��
    HAL_Delay(1000);
    GPIO_SPI_RST_SET;//RST�������Ϊ��
    HAL_Delay(100);
}

/**********************************************************************************/
/*************************************************
��������LCD_Set_Region
���ܣ�����lcd��ʾ�����ڴ�����д�������Զ�����
��ڲ�����xy�����յ�,Y_IncMode��ʾ������y������x
����ֵ����
*************************************************/
void hal_spi_setregion(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    hal_spi_write_cmd(&hspi3, 0x2a);
    hal_spi_write_8bit_data(&hspi3, x_start>>8);
    hal_spi_write_8bit_data(&hspi3, x_start);
    hal_spi_write_8bit_data(&hspi3, x_end>>8);
    hal_spi_write_8bit_data(&hspi3, x_end);

    hal_spi_write_cmd(&hspi3, 0x2b);
    hal_spi_write_8bit_data(&hspi3, y_start>>8);
    hal_spi_write_8bit_data(&hspi3, y_start);
    hal_spi_write_8bit_data(&hspi3, y_end>>8);
    hal_spi_write_8bit_data(&hspi3, y_end);

    hal_spi_write_cmd(&hspi3, 0x2c); //��ʼд��GRAM
}

// ��Ļ����
void hal_spi_clear(uint16_t color)
{
	uint16_t row,column;

	hal_spi_setregion(0, 0, LCD_WIDTH, LCD_HEIGHT);
	
    GPIO_SPI_CS_CLR;  // 重启屏幕
    GPIO_SPI_DC_SET;  // 传输数据
	for(row = 0;row < LCD_WIDTH; row++)             //ROW loop
	{
		for(column = 0;column < LCD_HEIGHT; column++) //column loop
		{
			hal_spi_write_16bit_data(&hspi3, color); 
		}
	}
	GPIO_SPI_CS_SET;
}

void hal_spi_lcd_init(void)
{
	/********************************************************/
    HAL_Delay(1000);
    hal_spi_write_cmd(&hspi3, ST7789_CMD_SLPOUT); 	//Sleep Out
    HAL_Delay(120);
    //-----------------------ST7789V Frame rate setting-----------------//
    //************************************************
    hal_spi_write_cmd(&hspi3, ST7789_CMD_COLMOD);       //65k mode
    hal_spi_write_8bit_data(&hspi3, 0x05);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_VCMOFSET); 	//VCOM
    hal_spi_write_8bit_data(&hspi3, 0x1A);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_MADCTL);       // ��Ļ��ʾ��������
    hal_spi_write_8bit_data(&hspi3, 0x00);
    //-------------ST7789V Frame rate setting-----------//
    hal_spi_write_cmd(&hspi3, ST7789_CMD_PORCTRL);		//Porch Setting
    hal_spi_write_8bit_data(&hspi3, 0x05);
    hal_spi_write_8bit_data(&hspi3, 0x05);
    hal_spi_write_8bit_data(&hspi3, 0x00);
    hal_spi_write_8bit_data(&hspi3, 0x33);
    hal_spi_write_8bit_data(&hspi3, 0x33);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_GCTRL);		//Gate Control
    hal_spi_write_8bit_data(&hspi3, 0x05);			//12.2v   -10.43v
    //--------------ST7789V Power setting---------------//
    hal_spi_write_cmd(&hspi3, ST7789_CMD_VCOMS);//VCOM
    hal_spi_write_8bit_data(&hspi3, 0x3F);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_LCMCTRL); //Power control
    hal_spi_write_8bit_data(&hspi3, 0x2c);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_VDVVRHEN);	//VDV and VRH Command Enable
    hal_spi_write_8bit_data(&hspi3, 0x01);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_VRHS);		//VRH Set
    hal_spi_write_8bit_data(&hspi3, 0x0F);		//4.3+( vcom+vcom offset+vdv)

    hal_spi_write_cmd(&hspi3, ST7789_CMD_VDVSET);		//VDV Set
    hal_spi_write_8bit_data(&hspi3, 0x20);				//0v

    hal_spi_write_cmd(&hspi3, ST7789_CMD_FRCTR2);		//Frame Rate Control in Normal Mode
    hal_spi_write_8bit_data(&hspi3, 0X01);			     //111Hz

    hal_spi_write_cmd(&hspi3, ST7789_CMD_PWCTRL1);		//Power Control 1
    hal_spi_write_8bit_data(&hspi3, 0xa4);
    hal_spi_write_8bit_data(&hspi3, 0xa1);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_PWCTRL2);		//Power Control 1
    hal_spi_write_8bit_data(&hspi3, 0x03);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_EQCTRL);		//Equalize time control
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x08);
    //---------------ST7789V gamma setting-------------//
    hal_spi_write_cmd(&hspi3, ST7789_CMD_PVGAMCTRL); //Set Gamma
    hal_spi_write_8bit_data(&hspi3, 0xD0);
    hal_spi_write_8bit_data(&hspi3, 0x05);
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x08);
    hal_spi_write_8bit_data(&hspi3, 0x14);
    hal_spi_write_8bit_data(&hspi3, 0x28);
    hal_spi_write_8bit_data(&hspi3, 0x33);
    hal_spi_write_8bit_data(&hspi3, 0x3F);
    hal_spi_write_8bit_data(&hspi3, 0x07);
    hal_spi_write_8bit_data(&hspi3, 0x13);
    hal_spi_write_8bit_data(&hspi3, 0x14);
    hal_spi_write_8bit_data(&hspi3, 0x28);
    hal_spi_write_8bit_data(&hspi3, 0x30);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_NVGAMCTRL); //Set Gamma
    hal_spi_write_8bit_data(&hspi3, 0xD0);
    hal_spi_write_8bit_data(&hspi3, 0x05);
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x09);
    hal_spi_write_8bit_data(&hspi3, 0x08);
    hal_spi_write_8bit_data(&hspi3, 0x03);
    hal_spi_write_8bit_data(&hspi3, 0x24);
    hal_spi_write_8bit_data(&hspi3, 0x32);
    hal_spi_write_8bit_data(&hspi3, 0x32);
    hal_spi_write_8bit_data(&hspi3, 0x3B);
    hal_spi_write_8bit_data(&hspi3, 0x14);
    hal_spi_write_8bit_data(&hspi3, 0x13);
    hal_spi_write_8bit_data(&hspi3, 0x28);
    hal_spi_write_8bit_data(&hspi3, 0x2F);

    hal_spi_write_cmd(&hspi3, ST7789_CMD_INVON);		//����

    hal_spi_write_cmd(&hspi3, ST7789_CMD_DISPON);		//������ʾ
}
/*********************************************END OF FILE********************************************/

