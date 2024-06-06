#include "userAPP.h"

// Variable

GLOBALTIME gTime;
GLOBALSTATUS gStatus;
MOTIONVAR motionStatus;
GLOBAL_ETH_UDP_VAR w5500_udp_var;
GLOBAL_CAN_VAR can_var;
MODBUSVARS modbusPosi;
SDRAM_STO_VAR sdram_var;

#if HAL_W5500_ENABLE

uint8_t gDATABUF[DATA_BUF_SIZE];  
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0x11},
                            .ip = {192, 168, 20, 11},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 20, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

volatile uint32_t last_timeMS = 0;
volatile uint32_t tim3_timeBaseCnt_10US = 0;
#endif

volatile uint32_t tim4_timeBaseCnt_1MS = 0;

uint8_t flagStatus = 0; 																	
int32_t avgPosiErr[2] = {0}; 		

// all pdo disable
uint16_t msg_allpdoDisable[16][9] = {
  {0x608,0x23,0x00,0x14,0x01,0x88,0x01,0x00,0x80},  // 失能tpdo1, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x01,0x14,0x01,0x88,0x01,0x00,0x80},  // 失能tpdo2, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x01,0x16,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x02,0x14,0x01,0x88,0x01,0x00,0x80},  // 失能tpdo3, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x02,0x16,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x03,0x14,0x01,0x88,0x01,0x00,0x80},  // 失能tpdo4, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x03,0x16,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x00,0x18,0x01,0x88,0x01,0x00,0x80},  // 失能rpdo1, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x01,0x18,0x01,0x88,0x01,0x00,0x80},  // 失能rpdo2, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x02,0x18,0x01,0x88,0x01,0x00,0x80},  // 失能rpdo3, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x02,0x1A,0x00,0x00,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 

  {0x608,0x23,0x03,0x18,0x01,0x88,0x01,0x00,0x80},  // 失能rpdo4, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x03,0x1A,0x00,0x00,0x00,0x00,0x00}  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 
};

// disable all PDO， only use 1
uint16_t message_sdo[MAX_PRESET_SDO_NUM][9] = {
  {0x608,0x23,0x00,0x18,0x01,0x88,0x01,0x00,0x80},  // 失能tpdo1, 改写COD-ID：608,23 00 14 01 01 00 00 80
  {0x608,0x2f,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},  // rpdo1映射个数清零: 608,23 00 16 00 00 00 00 00 
  {0x608,0x23,0x00,0x1A,0x01,0x20,0x19,0x0B,0x20},  // 映射相电流有效值到tdpo1: 4Byte 0.01A
  {0x608,0x23,0x00,0x1A,0x01,0x20,0x00,0x6C,0x60},  // 映射速度实际值到tpdo1: 4Byte rpm
  {0x608,0x23,0x00,0x18,0x01,0x88,0x01,0x00,0x00},  // 使能tpdo1:  608,23 00 14 01 01 02 00 00 

  {0x608,0x23,0x00,0x14,0x01,0x08,0x02,0x00,0x80},  // 失能rpdo1：
  {0x608,0x2f,0x00,0x16,0x00,0x01,0x00,0x00,0x00},  // rpdo1映射个数设置: 1
  {0x608,0x23,0x00,0x16,0x01,0x20,0x00,0xff,0x60},  // 映射速度设定值到rdpo1:
  {0x608,0x23,0x00,0x14,0x01,0x08,0x02,0x00,0x00},  // 使能rpdo1: 

  {0x608,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00},  // 切换从机工作模式为速度模式 1Byte
  {0x608,0x2b,0x40,0x60,0x01,0x01,0x00,0x00,0x80},  // 主回路供电启动 2Byte
};

// Function
#if HAL_W5500_ENABLE
// ETH initial
void network_init(void)
{
    uint8_t tmpstr[6];
    ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
    ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);

    // Display Network Information
    ctlwizchip(CW_GET_ID,(void*)tmpstr);
    printf("\r\n === %d ms %s NET CONF ===\r\n", gTime.l_time_ms, (char*)tmpstr);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],
        gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
    printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
    printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
    printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
    printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
    printf("======================\r\n");
}

void network_register(void)
{
	uint8_t tmp;
	int16_t ret = 0;
  uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
		
  reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);	
  /* Chip selection call back */
  #if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
    reg_wizchip_cs_cbfunc(SPI4_CS_Select, SPI4_CS_Deselect);
  #elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
    reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
  #else
    #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
        #error "Unknown _WIZCHIP_IO_MODE_"
    #else
        reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    #endif
  #endif
    /* SPI Read & Write callback function */
    reg_wizchip_spi_cbfunc(HAL_SPI4_ReadByte, HAL_SPI4_WriteByte);	//ע���д����?????

    /* WIZCHIP SOCKET Buffer initialize */
    if (ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
      printf("%d ms WIZCHIP Initialized fail.\r\n", gTime.l_time_ms);
      while(1);
    }

    /* PHY link status check */
    while (tmp == PHY_LINK_OFF) {
      if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){
          printf("W5500: Unknown PHY Link stauts.\r\n");
      }
    }
}

#endif

void systemParaInit(void)
{
	unsigned char ret = 0x00;	
    gStatus.telmode = IDLEMODE; 
    gStatus.workmode = RECVSPEEDMODE; 
	
    motionStatus.g_Distance = 0; // target Posi_um
    motionStatus.g_Speed = 0;

    can_var.NodeID = 0x01; // local node ID
    can_var.slaveCANID = SLAVECANID; // driver can ID

    w5500_udp_var.DstHostIP[0] = 192;
    w5500_udp_var.DstHostIP[1] = 168;
    w5500_udp_var.DstHostIP[2] = 20;
    w5500_udp_var.DstHostIP[3] = 33;
    w5500_udp_var.DstHostPort = 8888;

    w5500_udp_var.SrcRecvIP[0] = 192;
    w5500_udp_var.SrcRecvIP[1] = 168;
    w5500_udp_var.SrcRecvIP[2] = 20;
    w5500_udp_var.SrcRecvIP[3] = 11;
    w5500_udp_var.SrcRecvPort = 8001;

#if HAL_EEPROM_ENABLE
    AT24CXX_Init();
    //EEPROM self_test
    ret = AT24CXX_ReadOneByte(0xFF);
    printf("EEPROM: ret is : %d \n\r", ret);
        
    if (0x28 != ret) {
        printf("EEPROM: %d ms Fisrt Time EEPROM Initialized!\r\n", gTime.l_time_ms);

        HAL_Delay(10);
        AT24CXX_WriteOneByte(0xFF, 0x28);
        
        HAL_Delay(10);
        ret = AT24CXX_ReadOneByte(0xFF);

        if (0x28 != ret) {
            printf("EEPROM: %d ms Second EEPROM Initialize Failed Plz check link!\r\n", gTime.l_time_ms);
        } else {
            AT24CXX_WriteOneByte(0x01, 0x44); // first initial CAN ID = 0x01
        }
    } else {
        can_var.NodeID = AT24CXX_ReadOneByte(0x01);
        printf("EEPROM: can_var.NodeID is : 0x%x \n\r", can_var.NodeID);
    } 
 #endif   
}

int32_t avgErrCollect(uint8_t node, int32_t sampleData) 
{
	int32_t duss_result = 0;
	uint8_t snddata[8] = {0};

	if (node >= 3) {
		printf("UTC:%d ms CAS: %d Recv Error Context Pack\n\r", gTime.g_time_ms, gTime.l_time_ms);
		return -1;
	}
	avgPosiErr[node] = sampleData;
	flagStatus |= (0x01 << (node-1));
	return duss_result;
}

void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode)
{
    UNUSED(phcan);
   
    volatile u16 lastNewestSpeed = 0;
    volatile u32 curLocalTimeStamp = 0;

    u8 snddata[8]={0};
    short curGivenSpeed = 0; //rpm
    u32 tempGivenVol = 0;    //mV
    u32 sendCnt = 0;
    u16 tempFrameCnt = 0;
		UNS32 writeinCnt = 0;
    int32_t tempPosiErr = 0;
    int32_t tempRecvPosi = 0;

    // Ext CAN ID
    CAN_ID_Union ext_ID;
    ext_ID.Value = 0;

    // recv Frame Number just for consective test
    tempFrameCnt = CAN1_RecData[0];
    tempFrameCnt <<= 8;
    tempFrameCnt |= CAN1_RecData[1];
    
    switch(CTRCode)
    {
        case CANSpeedCmd:
          curGivenSpeed = CAN1_RecData[4];
          curGivenSpeed <<= 8;
          curGivenSpeed |= CAN1_RecData[5];

        #if HAL_DAC_ENABLE 
          if((curGivenSpeed <= MAX_ALLOWED_SPEED_RPM) && (curGivenSpeed >= MIN_ALLOWED_SPEED_RPM))
          {	
            tempGivenVol =  RPM2Vol_CONVERSE_COFF *curGivenSpeed + spdDownLimitVol ;
          
            HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
            printf("UTC: %d ms CAS: %d, frameNum: %d, update Speed :%d rpm\n\r", gTime.g_time_ms,
                                                                                  gTime.l_time_ms,
                                                                                  tempFrameCnt,
                                                                                   (short)curGivenSpeed);
        #else // CANOpen
            if (((short)curGivenSpeed <= MAX_ALLOWED_SPEED_RPM) && ((short)curGivenSpeed >= MIN_ALLOWED_SPEED_RPM)) {
								writeinCnt = sizeof(curGivenSpeed);
                writeLocalDict(&masterObjdict_Data, SPEEDGIVEN_INDEX, 0x00, &curGivenSpeed, &writeinCnt, RW);

                sendPDOrequest(&masterObjdict_Data, 0x1400); // request to update remote dictionary RPDO1

                printf("UTC: %d ms CAS: %d ms, frameNum: %d, update Speed :%d rpm\n\r", gTime.g_time_ms,
                                                                                      gTime.l_time_ms,
                                                                                      tempFrameCnt,
                                                                                      (short)curGivenSpeed);
            } else {
                printf("CAS:  %d ms SpeedGiven OverFlow \n\r!", gTime.l_time_ms);
            }
        #endif
        break;
        
				// PreDictive Speed Mode 
        case CANSpeedPreCmd:
          curGivenSpeed = CAN1_RecData[5];
          curGivenSpeed <<= 8;
          curGivenSpeed |= CAN1_RecData[6];
        
          if (((short)curGivenSpeed <= MAX_ALLOWED_SPEED_RPM) && ((short)curGivenSpeed >= MIN_ALLOWED_SPEED_RPM)) {	
              tempGivenVol = RPM2Vol_CONVERSE_COFF *curGivenSpeed + spdDownLimitVol ;
                      
              HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
              printf("UTC: %d ms CAS: %d, update Speed :%d rpm, t\n\r",  gTime.g_time_ms,
                                                                        gTime.l_time_ms,
                                                                        curGivenSpeed);
          }
        break;
          
        case CANTimeSyncCmd:
            //update global timeStamp
            gTime.g_time_ms = CAN1_RecData[2];
            gTime.g_time_ms <<= 8;
            gTime.g_time_ms |= CAN1_RecData[3];
            gTime.g_time_ms <<= 8;
            gTime.g_time_ms |= CAN1_RecData[4];	
            gTime.g_time_ms = gTime.g_time_ms;
            memcpy(snddata, CAN1_RecData, 8);		

            ext_ID.CAN_Frame_Union.CTRCode = CANTimeSyncCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack slave
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node
            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 4, ext_ID.Value);

        break;
      
        case CANPisiAcquireCmd:
            memcpy(snddata, CAN1_RecData, 8);

            snddata[5] = (motionStatus.g_Distance & 0x00FF0000) >> 16;
            snddata[6] = (motionStatus.g_Distance & 0x0000FF00) >> 8;
            snddata[7] = (motionStatus.g_Distance & 0x000000FF);

            ext_ID.CAN_Frame_Union.CTRCode = CANPisiAcquireCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node

            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
            printf("UTC:%d ms CAS: %d CurPosi is %d \n\r", gTime.g_time_ms, gTime.l_time_ms, motionStatus.g_Distance);
        break;

        // 
        case CANTimeSyncErrorCalCmd: 
            tempRecvPosi = CAN1_RecData[5];
            tempRecvPosi <<= 8;
            tempRecvPosi |= CAN1_RecData[6];

            avgErrCollect(ext_ID.CAN_Frame_Union.NodeOrGroupID, tempRecvPosi);
            if (flagStatus == 7) {
                tempPosiErr = avgErrUpdate(avgPosiErr);
                flagStatus = 0;	

              if (can_var.NodeID == 0x01) {
                  snddata[2] = 0;
                  snddata[3] = 0;
                  snddata[4] = 0;
                  snddata[5] = (tempPosiErr & 0x00FF0000) >> 16;
                  snddata[6] = (tempPosiErr & 0x0000FF00) >> 8;
                  snddata[7] = (tempPosiErr & 0x000000FF);

                  ext_ID.CAN_Frame_Union.CTRCode = CANTimeSyncErrorCalCmd; // position acquire pack type
                  ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
                  ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node
								  HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
              }
          }
        break;
          
        case CANLocalPITestCmd:
          if (CAN1_RecData[0] == 1) {
            PIDController_Reset(&pid);
            gStatus.workmode = PIPOSIMODE;
            givenExecPosiVal += 20600;
          }
          else if (CAN1_RecData[0] == 2) {
            gStatus.workmode = PREPOSIMODE;
          }
          else {
            gStatus.workmode = RECVSPEEDMODE;
          }
          printf("UTC:%d ms CAS: %d Start PI Position Control, Initial posi is %d, givenPosiVal is %d\n\r", gTime.g_time_ms, 
                                                            gTime.l_time_ms,
                                                            recvPosiInitVal,
                                                            givenExecPosiVal);
        break;

        default:
          printf("UTC:%d ms CAS: %d Recv Error Context Pack\n\r", gTime.g_time_ms, gTime.l_time_ms);
        break;
    }
}

int32_t avgErrUpdate(int32_t *sampleData) 
{
	int32_t duss_result =0;
	
	
	return duss_result;
}

//SDRAM内存测试	    
void fsmc_sdram_test()
{  
	u32 i=0;  	  
	u32 temp=0;	   
	u32 sval=0;	//在地址0读到的数据	  				   
	//每隔16K字节,写入一个数据,总共写入2048个数据,刚好是32M字节
	for(i=0;i<32*1024*1024;i+=16*1024) {
		*(vu32*)(Bank5_SDRAM_ADDR+i)=temp; 
		temp++;
	}
	//依次读出之前写入的数据,进行校验		  
 	for(i=0;i<32*1024*1024;i+=16*1024) 
	{	
  		temp=*(vu32*)(Bank5_SDRAM_ADDR+i);
		if (i==0) {
      sval=temp;
    } else if (temp<=sval) {
      break;
    }//后面读出的数据一定要比第一次读到的数据大.	   		   
		printf("SDRAM Capacity:%dKB\r\n",(u16)(temp-sval+1)*16);//打印SDRAM容量
 	}					 
}	

// TIMER3/TIMER4
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
   static unsigned int heartbeatChangedMs = 0; 
   unsigned int cmpVal = POSI_CHECK_PERIOD_1MS;
	 if (htim == &htim3) {
		 #if HAL_W5500_ENABLE
				tim3_timeBaseCnt_10US++;
		 #endif
      // 10us timecnt
      if (gTime.l_time_cnt_10us < 360000000 ) { // 60min reset local timing
        gTime.l_time_cnt_10us++;
      } else {
        gTime.l_time_cnt_10us = 0;
        gTime.l_time_ms = 0;
        gStatus.l_time_overflow++;
      }
      // 1ms timecnt
      if (gTime.l_time_cnt_10us % 100 == 0 && (gTime.l_time_cnt_10us > 0)) {
        gTime.l_time_ms++;
				// 20ms  posi acquire
				if (gStatus.l_rs485_getposi_cnt >= 20) {
					gStatus.l_rs485_getposiEnable = 1;
					gStatus.l_rs485_getposi_cnt = 0;

          // CAN2 Send TEST
          #if CAN2_SENDTEST_ON
            gStatus.l_can2_send_flag = 1;
          #endif 

				}
				// NEW RTU Frame recv Start
				if (modbusPosi.g_RTU_Startflag == 1) {
					modbusPosi.g_10ms_Cnt++;
					cmpVal = MODBUS_INTERNAL_1MS;
					//消息间隔超过10ms
					if (modbusPosi.g_10ms_Cnt >= 10) {
						 modbusPosi.g_10ms_Cnt = 0;
						 modbusPosi.g_RTU_Startflag = 0;
					
						//RTU接收完成标识
						 modbusPosi.g_RTU_RcvFinishedflag = 1;
					}
				}
				gStatus.l_rs485_getposi_cnt++; // 1ms ++
      }

      // 1s update local time and Sensor
      if ((gTime.l_time_ms % 1000 == 0) && ((gTime.l_time_ms > 0)) && (0 != gTime.l_time_ms-heartbeatChangedMs)) {
        gStatus.l_time_heartbeat = 1;
        heartbeatChangedMs = gTime.l_time_ms;
      }
      
      // 10s test
      if (gTime.l_time_cnt_10us % 1000000 == 0) {
        gStatus.l_bissc_sensor_acquire = 1;
      }
    } else if (htim == &htim4) {
      tim4_timeBaseCnt_1MS++;
      TimeDispatch(); // canfestival software timer 
    } else {
      printf("%d ms Unknown TIMER Interupt! \r\n", gTime.l_time_ms);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *phcan)
{
    UNUSED(phcan);
    CAN_RxHeaderTypeDef RxMessage;
    Message RMessage;
    unsigned char rxbuf[8] = {0};
    unsigned char cnt = 0;
    static unsigned int consectiveCnt = 0;

	  // printf("CAN: %d ms recv New Frame \n\r", gTime.l_time_ms);
		
    if (phcan == &hcan1) {
        HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &RxMessage, rxbuf);
        CAN1RecvFrame.Value = RxMessage.ExtId;
        if ((CAN1RecvFrame.CAN_Frame_Union.NodeOrGroupID == can_var.NodeID) && (CAN1RecvFrame.CAN_Frame_Union.MasterOrSlave == 1)){
            for (cnt = 0; cnt < 8; cnt++) {
                CAN1_RecData[cnt] = rxbuf[cnt];
            }
            printf("%d ms CAS: recv New Frame Type is %d\n\r", gTime.l_time_ms, CAN1RecvFrame.CAN_Frame_Union.CTRCode);
            gStatus.l_can1_recv_flag = 1;
        } else {
            gStatus.l_can1_recv_flag = 0;
        }
    } else if (phcan == &hcan2) {
        HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &RxMessage, rxbuf);

        RMessage.cob_id = RxMessage.StdId;				// driver canid		               
        if (RxMessage.RTR == CAN_RTR_DATA) {
						RMessage.rtr = 0;
				} else {
						RMessage.rtr = 1;
				}
				
        RMessage.len = RxMessage.DLC;							                      
        memcpy(RMessage.data, rxbuf, RxMessage.DLC);		               

				/*CANOpen Timer*/
        canDispatch(&masterObjdict_Data, &RMessage);
        
				consectiveCnt++;
        
        #if TEST_CAN_STABLITY
          printf("%d CAN2 Recv New Frame! \n\r", consectiveCnt); 
        #endif
			} else {
        printf("%d recv error can frame \n\r", gTime.l_time_ms);
    }
}

uint32_t tim4_getCurrentTimeCnt(void)
{
    return tim4_timeBaseCnt_1MS;
}

// MS level nonblocking delay
uint8_t tim4_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt)
{
    // avoid initial problem
    if (*lastTimeMS == 0) {
        *lastTimeMS = tim4_getCurrentTimeCnt();
    }
    if ((tim4_timeBaseCnt_1MS - *lastTimeMS) >= delay1MS_cnt) {
        *lastTimeMS = 0;
        return 1;
    }
    return 0;
}

#if HAL_W5500_ENABLE
uint32_t tim3_getCurrentTimeCnt(void)
{
    return tim3_timeBaseCnt_10US;
}

// MS level nonblocking delay
uint8_t tim3_noblocked_1MS_delay(uint32_t *lastTimeMS, uint16_t delay1MS_cnt)
{
    // avoid initial problem
    if (*lastTimeMS == 0) {
        *lastTimeMS = tim3_getCurrentTimeCnt();
    }
    if ((tim3_timeBaseCnt_10US - *lastTimeMS)/100 >= delay1MS_cnt) {
        *lastTimeMS = 0;
        return 1;
    }
    return 0;
}

void w5500_stateMachineTask(void)
{
    uint8_t ret = 0;
    switch (getSn_SR(0)) {
			case SOCK_UDP:																							    
					if (1 == tim3_noblocked_1MS_delay(&last_timeMS, 1)) {
              if (getSn_IR(0) & Sn_IR_RECV) {
                setSn_IR(0, Sn_IR_RECV);															   
              }
              
              if ((ret = getSn_RX_RSR(0)) > 0) { 
                  memset(gDATABUF, 0, ret+1);
                  recvfrom(0, gDATABUF, ret, w5500_udp_var.DstHostIP, &w5500_udp_var.DstHostPort);			
                  printf(" %d ms %s\r\n", gTime.l_time_ms, gDATABUF);															  
                  sendto(0, gDATABUF,ret, w5500_udp_var.DstHostIP, w5500_udp_var.DstHostPort);		  	
              }
          }
			break;
			case SOCK_CLOSED:																						   
					socket(0, Sn_MR_UDP, w5500_udp_var.SrcRecvPort, 0x00);			
			break;
		}
}
#endif

void canOpen_Init(void)
{
    setNodeId(&masterObjdict_Data, can_var.slaveCANID);
    setState(&masterObjdict_Data, Initialisation);  // 000 01 08

    setState(&masterObjdict_Data, Pre_operational); //
    setState(&masterObjdict_Data, Operational);
    masterSendNMTstateChange(&masterObjdict_Data, can_var.slaveCANID, NMT_Start_Node);

    canOpenSDOConfig();
}

// SDO Transmit
void canopen_send_sdo(uint16_t message_sdo[])
{
	  unsigned long abortCode=0;
    uint8_t      nodeID=0;          /* ID      */
    uint16_t    index=0;           /* 索引    */
    uint8_t     subIndex=0;        /* 子索引   */
    uint8_t     dataType=uint8;    /* 数据类型 */
    uint32_t    count=4;           /* 数据长度 */
    uint8_t     data[4];    
    uint8_t     i=0;
    
    nodeID=(int8_t)(message_sdo[0] & 0x7f);  // get low 7 bit (0-255)  
    index=(uint16_t)((message_sdo[3]<<16)+message_sdo[2]);
    subIndex=(uint8_t)(message_sdo[4]);

    for (i=0; i<4; i++) {
        data[0+i]=(uint8_t)message_sdo[5+i];
    }

    //d; nodeId; index; subIndex; count; dataType; data[4]; useBlockMode;
    writeNetworkDict(&masterObjdict_Data, nodeID, index, subIndex, count, dataType, &data, 0);
    while (getWriteResultNetworkDict(&masterObjdict_Data, nodeID, &abortCode) != SDO_FINISHED) {
        break;
    }
}

// SDO config PDO
uint8_t canOpenSDOConfig(void)
{
    uint8_t cnt = 0;
    uint16_t msg[9] = {0x608,0x23,0x00,0x18,0x01,0x88,0x01,0x00,0x80};
    
    #if CANOPEN_NONBLOACK_DELAY_ENABLE
      for (cnt=0; cnt<MAX_PRESET_SDO_NUM; cnt++) {
          if (tim4_noblocked_1MS_delay(&(can_var.canDelayTime_MS[cnt]), 20) == 1) {
              canopen_send_sdo(message_sdo[cnt]);
          }
      }
    #else
    printf ("CANOpen: SDO Send Start! \n\r");
     for (cnt=0; cnt<16; cnt++) {
         canopen_send_sdo(msg_allpdoDisable[cnt]);
     }
    printf ("CANOpen: SDO Send END! \n\r");
    #endif

    return 0;
}