#include "userAPP.h"

// Variable
GLOBALTIME gTime;
GLOBALSTATUS gStatus;
MOTIONVAR motionStatus;
GLOBAL_ETH_UDP_VAR w5500_udp_var;
GLOBAL_CAN_VAR can_var;
MODBUSVARS modbusPosi;
SDRAM_STO_VAR sdram_var;


uint16_t canopenStopMachineAndTransMode(uint8_t targetOperationMode);


#if HAL_W5500_ENABLE
uint8_t gDATABUF[DATA_BUF_SIZE];  
uint8_t gSendBUF[DATA_BUF_SIZE];

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
    reg_wizchip_spi_cbfunc(HAL_SPI4_ReadByte, HAL_SPI4_WriteByte);	

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
    unsigned char cnt = 0;

    gStatus.telmode = ETHMODE;  // 初始化为以太网控制模式，CAN仅用于单机调试
    motionStatus.g_Distance = 0; // target Posi_um
    motionStatus.g_Speed = 0;

    can_var.CASNodeID = 0x01;
    // CANOpen相关参数初始化后不变
    can_var.CANOpenMasterID = 0x01; // master CAN ID
    can_var.slaveCANID = SLAVECANID; // driver can ID

	#if HAL_W5500_ENABLE
	
    // 上位机 IP 192.168.20.33 Port 8888
    // 本地 IP 192.168.20.11 + (CANNodeID-1) Port 8001+ (CANNodeID-1) MAC 也一致更改

    w5500_udp_var.DstHostIP[0] = 192;
    w5500_udp_var.DstHostIP[1] = 168;
    w5500_udp_var.DstHostIP[2] = 20;
    w5500_udp_var.DstHostIP[3] = 33;
    w5500_udp_var.DstHostPort = 8888;

    w5500_udp_var.SrcRecvIP[0] = 192;
    w5500_udp_var.SrcRecvIP[1] = 168;
    w5500_udp_var.SrcRecvIP[2] = 20;
    w5500_udp_var.SrcRecvIP[3] = 11+can_var.CASNodeID-1;
    w5500_udp_var.SrcRecvPort = 8001+can_var.CASNodeID-1;

    w5500_udp_var.SrcMAC[0] = 0x00;
    w5500_udp_var.SrcMAC[1] = 0x08;
    w5500_udp_var.SrcMAC[2] = 0xDC;
    w5500_udp_var.SrcMAC[3] = 0x11;
    w5500_udp_var.SrcMAC[4] = 0x11;
    w5500_udp_var.SrcMAC[5] = 0x11+can_var.CASNodeID-1;

    w5500_udp_var.SrcSendPort = 8000; // 统一用8000端口发送

    // Update W5500 Initial Paras
    for (cnt=0;cnt<4;cnt++) {
      gWIZNETINFO.ip[cnt] =  w5500_udp_var.SrcRecvIP[cnt];
    }

    for (cnt=0;cnt<6;cnt++) {
      gWIZNETINFO.mac[cnt] =  w5500_udp_var.SrcMAC[cnt];
    }

#endif

    // 预设工作模式初始化
    motionStatus.targetWorkmode = RECVSPEEDMODE; // 默认电机控制为速度模式


// 0x01 CASNodeID
// 0x28 EEP初始化标志

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
            AT24CXX_WriteOneByte(0x01, can_var.CASNodeID); // first initial CAN ID
        }
    } else {
        can_var.CASNodeID = AT24CXX_ReadOneByte(0x01);  // 设置CAN1 ID
        printf("EEPROM: can_var.CASNodeID is : 0x%x \n\r", can_var.CASNodeID);
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

// CAN1接收处理
void CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode)
{
    UNUSED(phcan);
   
    volatile u16 lastNewestSpeed = 0;
    volatile u32 curLocalTimeStamp = 0;

    u8 snddata[8]={0};
    INTEGER32 curGivenSpeed = 0; //rpm
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
    
    switch(CTRCode) {
        case CANTargetCmd:  // 0x01
            if (motionStatus.targetWorkmode == motionStatus.g_curOperationMode) {
                if (motionStatus.targetWorkmode == RECVSPEEDMODE) {
                    curGivenSpeed = CAN1_RecData[4];
                    curGivenSpeed <<= 8;
                    curGivenSpeed |= CAN1_RecData[5];
                    #if HAL_CANOPEN_ENABLE 
                            canopenDriverSpeedGive(curGivenSpeed);
                    #else 
                            DACDriverSpeedGive(curGivenSpeed);
                    #endif               
                } else if (motionStatus.targetWorkmode == TORQUEMODE) {
                  ;
                }
            } else {
                printf("CAN1: System OperationMode not eq real Motor Mode! \r\n");
            }
        break;
        
				// PreDictive Speed Mode 
        case CANSpeedPreCmd:  // 0x02
            curGivenSpeed = CAN1_RecData[5];
            curGivenSpeed <<= 8;
            curGivenSpeed |= CAN1_RecData[6];

#if HAL_CANOPEN_ENABLE 
        canopenDriverSpeedGive(curGivenSpeed);
#else 
        DACDriverSpeedGive(curGivenSpeed);
#endif
        break;
          
        case CANTimeSyncCmd:  //0x04
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
      
        case CANPisiAcquireCmd: // 0x05
            memcpy(snddata, CAN1_RecData, 8);

            snddata[5] = (motionStatus.g_Distance & 0x00FF0000) >> 16;
            snddata[6] = (motionStatus.g_Distance & 0x0000FF00) >> 8;
            snddata[7] = (motionStatus.g_Distance & 0x000000FF);

            ext_ID.CAN_Frame_Union.CTRCode = CANPisiAcquireCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node

            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
            printf("UTC:%d ms CAS: %d ms CAN1 CurPosi is %d \n\r", gTime.g_time_ms, gTime.l_time_ms, motionStatus.g_Distance);
        break;

        case CANTimeSyncErrorCalCmd: 
            tempRecvPosi = CAN1_RecData[5];
            tempRecvPosi <<= 8;
            tempRecvPosi |= CAN1_RecData[6];

            avgErrCollect(ext_ID.CAN_Frame_Union.NodeOrGroupID, tempRecvPosi);
            if (flagStatus == 7) {
                tempPosiErr = avgErrUpdate(avgPosiErr);
                flagStatus = 0;	

							// 时间隔得太久，忘了处理逻辑
              if (can_var.CASNodeID == 0x01) {
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
            motionStatus.targetWorkmode = PIPOSIMODE;
            givenExecPosiVal += 20600;
          }
          else if (CAN1_RecData[0] == 4) {
             motionStatus.targetWorkmode = TORQUEMODE;
          }
          else {
             motionStatus.targetWorkmode = RECVSPEEDMODE;
          }
          printf("UTC:%d ms CAS: %d ms CAN1 Start PI Position Control, Initial posi is %d, givenPosiVal is %d\n\r", gTime.g_time_ms, 
                                                            gTime.l_time_ms,
                                                            recvPosiInitVal,
                                                            givenExecPosiVal);
        break;

        default:
          // printf("UTC:%d ms CAS: %d ms CAN1 Recv Error Context Pack\n\r", gTime.g_time_ms, gTime.l_time_ms);
        break;
    }
}

int32_t avgErrUpdate(int32_t *sampleData) 
{
	int32_t duss_result =0;
	
	
	return duss_result;
}

// SDRAM内存测试	    
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
   volatile static unsigned int bissc_interval_cnt = 0;
   volatile static unsigned int bissc_interval_old = 0;

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
            }
            // NEW RTU Frame recv Start
            if (modbusPosi.g_RTU_Startflag == 1) {
                modbusPosi.g_10ms_Cnt++;
                //消息间隔超过10ms
                if (modbusPosi.g_10ms_Cnt >= 10) {
                    modbusPosi.g_10ms_Cnt = 0;
                    modbusPosi.g_RTU_Startflag = 0;
                    modbusPosi.g_RTU_RcvFinishedflag = 1;
                }
            }
            gStatus.l_rs485_getposi_cnt++; // 1ms ++
        }

        // 1ms
        if (bissc_interval_cnt >= 360000000) {
            bissc_interval_cnt = 0;
            bissc_interval_old = 0;
        }
        if (bissc_interval_cnt - bissc_interval_old >= 100) {
            bissc_interval_old = bissc_interval_cnt;
            if (gStatus.l_bissc_sw == 1) {
               gStatus.l_bissc_sensor_acquire = 1;
            }
        }
        bissc_interval_cnt++;

         // 1s update local time and Sensor
        if ((gTime.l_time_ms % 1000 == 0) && ((gTime.l_time_ms > 0)) && (0 != gTime.l_time_ms-heartbeatChangedMs)) {
            gStatus.l_time_heartbeat = 1;
            heartbeatChangedMs = gTime.l_time_ms;
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
        if ((CAN1RecvFrame.CAN_Frame_Union.NodeOrGroupID == can_var.CASNodeID) && (CAN1RecvFrame.CAN_Frame_Union.MasterOrSlave == 1)){
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
    EthControlFrameSingleCAS cmdFrame; 
		
    switch (getSn_SR(0)) {
			case SOCK_UDP:																							    
					if (1 == tim3_noblocked_1MS_delay(&last_timeMS, 1)) {
              if (getSn_IR(0) & Sn_IR_RECV) {
                setSn_IR(0, Sn_IR_RECV);															   
              }
              
              if ((ret = getSn_RX_RSR(0)) > 0) { 
                  memset(gDATABUF, 0, ret+1);
                  recvfrom(0, gDATABUF, ret, w5500_udp_var.DstHostIP, &(w5500_udp_var.DstHostPort));		
									
                  // 拷贝以防被覆盖
                  memcpy(&cmdFrame, gDATABUF, sizeof(cmdFrame));
									
                  // Decoder
                  w5500_Decoder(cmdFrame);
              }
          }
			break;
			case SOCK_CLOSED:																						   
					socket(0, Sn_MR_UDP, w5500_udp_var.SrcRecvPort, 0x00);			
			break;
		}

    switch (getSn_SR(1)) {
      	case SOCK_UDP:	
          ; // do nothing
        break;
        case SOCK_CLOSED:																						   
            socket(1, Sn_MR_UDP, w5500_udp_var.SrcSendPort, 0x00);	// 这里的绑定是接收端口，但由于socket1仅用于发送，所以实际没有生效		
        break;
    }
}

// W5500 Recv deal
uint8_t w5500_Decoder(EthControlFrameSingleCAS frame) 
{
    uint8_t ret = 0;
    short speedGivenRpm = 0;   // 单位rpm
    short torqueGivenCmd = 0;  // 上面发下来的单位是0.001Nm
    CASREPORTFRAME statusPack;
    uint8_t pcSetupOperationMode = 0;

    switch (frame.EType) {
        case CANTargetCmd: // 0x01

            if (motionStatus.targetWorkmode == motionStatus.g_curOperationMode) {
                if (motionStatus.targetWorkmode == RECVSPEEDMODE) {
                    speedGivenRpm = frame.canpack.CANData[4];
                    speedGivenRpm <<= 8;
                    speedGivenRpm |= frame.canpack.CANData[5];
                    #if HAL_CANOPEN_ENABLE
                        canopenDriverSpeedGive(speedGivenRpm);
                    #endif
                } else if (motionStatus.targetWorkmode == TORQUEMODE) {
                     torqueGivenCmd = frame.canpack.CANData[4];
                    torqueGivenCmd <<= 8;
                    torqueGivenCmd |= frame.canpack.CANData[5];
                    #if HAL_CANOPEN_ENABLE
                        canopenDriverTorqueGive(torqueGivenCmd);    
                    #endif             
                }
            } else {
                printf("CAN1: System OperationMode not eq real Motor Mode! \r\n");
            }
        break;
      
        case CANOperationModeCmd: // 0x3
            pcSetupOperationMode = frame.canpack.CANData[4];
            printf("ETH: Recv Frame to change OperationMode to %x\n\r", pcSetupOperationMode);
            if (motionStatus.g_curOperationMode != pcSetupOperationMode) {
                #if HAL_CANOPEN_ENABLE
                    canopenStopMachineAndTransMode(pcSetupOperationMode);
                #endif
            }
          break;

        case CANTimeSyncCmd: // 0x04
          ;
        break;

        case CANPisiAcquireCmd: // 0x05
            w5500_reportStatus(statusPack);
        break;
        
        case CANDriverInfoAcquire: // SDRAM数据获取


          break;
        default:
           printf("W5500: Recv Error ETHCAS Pack \n\r");
        break;
    }
    return ret;
}

// 借用 0x05 状态数据上报
uint32_t w5500_reportStatus(CASREPORTFRAME statusPack)
{
    uint32_t ret = 0;

    statusPack.EHeader = 0xAA55;
    statusPack.FrameTailer = 0x55AA;
    statusPack.EType = CANPisiAcquireCmd;

    statusPack.ELen = sizeof(CASREPORTFRAME);

    statusPack.CASNodeID = can_var.CASNodeID;
    statusPack.curWorkMode = motionStatus.g_curOperationMode;
    statusPack.localTimeMS = gTime.l_time_ms;
    statusPack.motorPosiUM = motionStatus.g_Distance;
    statusPack.motorRealTimeTorqueNM = motionStatus.g_Torque; // 0.001Nm
    statusPack.motorAveragePhaseAmp = motionStatus.g_phaseAmp;
    statusPack.statusWord = motionStatus.motorStatusWord.Value;

    // printf ("Now motorPosi is %d um \r\n", motionStatus.g_Distance);
    memcpy(gSendBUF, &statusPack, sizeof(statusPack));
    ret = sendto(1, gSendBUF, sizeof(statusPack), w5500_udp_var.DstHostIP, 8888);		
    return ret;
}

#endif

/*  dataType:
define in objdictdef.h

#define boolean         0x01
#define int8            0x02
#define int16           0x03
#define int32           0x04
#define uint8           0x05
#define uint16          0x06
#define uint32          0x07
#define real32          0x08
#define visible_string  0x09
#define octet_string    0x0A
#define unicode_string  0x0B
#define time_of_day     0x0C
#define time_difference 0x0D
*/

#if HAL_CANOPEN_ENABLE

#define PRESETSDOLENG    (38) // Slave SDO配置
#define SPEEDSETUPLENG   (9)  // 速度模式下参数设置
#define TORQUESETUPLENG  (10) // 转矩模式下参数设置

// 这里的SDO都是配置参数，所以数据类型都是UNS8, 如果是SDO发送实时速度，才会变更为INT等
uint16_t message_sdo[PRESETSDOLENG][10] = {
    // RPDO1
    {0x608, 0x23, 0x00, 0x14, 0x01, 0x08, 0x02, 0x00, 0x80, uint8}, // RPDO1 失能 类型：
    {0x608, 0x2f, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO1 传输类型 值变更触发
    {0x608, 0x2f, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // 清除原有映射内容
    {0x608, 0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60, uint8}, // 映射为控制字 0x6040
    {0x608, 0x23, 0x00, 0x16, 0x02, 0x08, 0x00, 0x60, 0x60, uint8}, // 映射为运动模式 0x6060  
    {0x608, 0x2f, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00, uint8}, // 映射数量改为2
    {0x608, 0x23, 0x00, 0x14, 0x01, 0x08, 0x02, 0x00, 0x00, uint8}, // RPDO1 使能
    // RPDO2-4
    {0x608, 0x23, 0x01, 0x14, 0x01, 0x08, 0x03, 0x00, 0x80, uint8}, // RPDO2 失能
    {0x608, 0x2f, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO2 传输类型
    {0x608, 0x2f, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // 清除原有映射内容
    {0x608, 0x23, 0x01, 0x16, 0x01, 0x20, 0x00, 0xFF, 0x60, uint8}, // 映射为给定速度 0x60FF
    {0x608, 0x23, 0x01, 0x16, 0x02, 0x10, 0x00, 0x71, 0x60, uint8}, // 映射为给定转矩 0x6071
    {0x608, 0x2f, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00, uint8}, // 映射数量改为2
    {0x608, 0x23, 0x01, 0x14, 0x01, 0x08, 0x03, 0x00, 0x00, uint8}, // RPDO2 使能

    {0x608, 0x23, 0x02, 0x14, 0x01, 0x08, 0x04, 0x00, 0x80, uint8}, // RPDO3 失能
    {0x608, 0x2f, 0x02, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO3 传输类型
    {0x608, 0x23, 0x03, 0x14, 0x01, 0x08, 0x05, 0x00, 0x80, uint8}, // RPDO4 失能
    {0x608, 0x2f, 0x03, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO4 传输类型
    // TPDO1
    {0x608, 0x23, 0x00, 0x18, 0x01, 0x88, 0x01, 0x00, 0x80, uint8}, // TPDO1 失能
    {0x608, 0x2f, 0x00, 0x18, 0x02, 0x64, 0x00, 0x00, 0x00, uint8}, // TPDO1 传输类型 周期触发 100SYNC 1s
    {0x608, 0x2f, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // 清除原有映射内容
    {0x608, 0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60, uint8}, // 映射为状态字 0x6041
    {0x608, 0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x6C, 0x60, uint8}, // 映射为实时速度指令 0x606C
    {0x608, 0x23, 0x00, 0x1A, 0x03, 0x10, 0x19, 0x0B, 0x20, uint8}, // 映射为相电流有效值 0x200B-19H
    {0x608, 0x2f, 0x00, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, uint8}, // 映射数量改为3
    {0x608, 0x23, 0x00, 0x18, 0x01, 0x88, 0x01, 0x00, 0x00, uint8}, // TPDO1 使能      
    // TDO2 
    {0x608, 0x23, 0x01, 0x18, 0x01, 0x88, 0x02, 0x00, 0x80, uint8}, // TPDO2 失能
    {0x608, 0x2f, 0x01, 0x18, 0x02, 0x64, 0x00, 0x00, 0x00, uint8}, // TPDO2 传输类型 周期触发 100 SYNC 1s
    {0x608, 0x2f, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // TPDO2 映射清零
    {0x608, 0x23, 0x01, 0x1A, 0x01, 0x08, 0x00, 0x61, 0x60, uint8}, // 0x6061 当前运动模式显示 速度/位置/转矩
    {0x608, 0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60, uint8}, // 0x6064 编码器绝对位置
    {0x608, 0x23, 0x01, 0x1A, 0x03, 0x10, 0x00, 0x77, 0x60, uint8}, // 0x6077 实时转矩(1000为额定转矩)       
    {0x608, 0x2f, 0x01, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, uint8}, // TPDO2 映射为3
    {0x608, 0x23, 0x01, 0x18, 0x01, 0x88, 0x02, 0x00, 0x00, uint8}, // TPDO2 使能

    // TPDO3-4
    {0x608, 0x23, 0x02, 0x18, 0x01, 0x88, 0x03, 0x00, 0x80, uint8}, // TPDO3 失能
    {0x608, 0x2f, 0x02, 0x18, 0x02, 0x64, 0x00, 0x00, 0x00, uint8}, // TPDO3 传输类型 
    {0x608, 0x23, 0x03, 0x18, 0x01, 0x88, 0x04, 0x00, 0x80, uint8}, // TPDO4 失能
    {0x608, 0x2f, 0x03, 0x18, 0x02, 0x64, 0x00, 0x00, 0x00, uint8}, // TPDO4 传输类型 
};

uint16_t message_speedMode_sdo[SPEEDSETUPLENG][10] = {
    {0x608, 0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, int32},  // 给定速度为0
    {0x608, 0x2f, 0x7E, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, uint8},  // 给定指令极性为向上 0
    {0x608, 0x23, 0x83, 0x60, 0x00, 0xAA, 0xAA, 0x2A, 0x04, uint32},  // 轮廓加速度 500rpm/s 
    {0x608, 0x23, 0x84, 0x60, 0x00, 0xAA, 0xAA, 0x2A, 0x04, uint32},  // 轮廓加速度 500rpm/s 
    {0x608, 0x23, 0x85, 0x60, 0x00, 0xAA, 0xAA, 0xAA, 0x10, uint32},  // 快速停机 加速度2000rpm/s 
    // H08.00 速度还带宽 100Hz 默认 40Hz
    // 积分时间常数：9.89ms 默认：19.89ms
    // 转矩指令滤波时间常数 默认 0.5ms
    // 转矩限幅 60E0  60E1 默认是 350%额定值转矩
    {0x608, 0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00, uint8},  // 速度模式 0x03
    {0x608, 0x23, 0x7F, 0x60, 0x00, 0x0D, 0x8D, 0xA1, 0x08, uint32}, // 速度模式下最大运行速度 1000rpm
    {0x608, 0x2b, 0x6F, 0x60, 0x00, 0x0A, 0x00, 0x00, 0x00, uint16}, // 零速检测速度阈值 10rpm
    {0x608, 0x2b, 0x70, 0x60, 0x00, 0x64, 0x00, 0x00, 0x00, uint16} // 零速信号窗口时间(我认为是持续时间 100ms)
 };

// 停机并切换模式 
uint16_t canopenStopMachineAndTransMode(uint8_t targetOperationMode)
{
    uint16_t ret =0; 
    uint8_t cnt = 0;
    uint16_t message_torqueMode_sdo[TORQUESETUPLENG][10] = {
        {0x608, 0x2b, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, int16},  // 给定转矩为0
        {0x608, 0x2f, 0x7E, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, uint8},  // 给定指令极性为默认向上 0
        {0x608, 0x23, 0xC5, 0x60, 0x00, 0xAA, 0xAA, 0x2A, 0x04, uint32},  // 轮廓加速度限制 500rpm/s 
        {0x608, 0x23, 0xC6, 0x60, 0x00, 0xAA, 0xAA, 0x2A, 0x04, uint32},  // 轮廓加速度 500rpm/s 
        {0x608, 0x23, 0x85, 0x60, 0x00, 0xAA, 0xAA, 0xAA, 0x10, uint32},  // 快速停机 加速度2000rpm/s 
        // 转矩斜坡默认最大值
        // 默认转矩限幅 3.5倍
        {0x608, 0x2b, 0x72, 0x60, 0x00, 0xAC, 0x0D, 0x00, 0x00, uint16},  // 转矩限幅 
        {0x608, 0x2b, 0xE0, 0x60, 0x00, 0xAC, 0x0D, 0x00, 0x00, uint16},  // 正向转矩最大值
        {0x608, 0x2b, 0xE1, 0x60, 0x00, 0xAC, 0x0D, 0x00, 0x00, uint16},  // 反向转矩最大值
        // H08.00 速度还带宽 100Hz 默认 40Hz
        // 积分时间常数：9.89ms 默认：19.89ms
        // 转矩指令滤波时间常数 默认 0.5ms
        // 转矩限幅 60E0  60E1 默认是 350%额定值转矩
        {0x608, 0x2f, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00, uint8},  // 转矩模式 0x04
        {0x608, 0x23, 0x7F, 0x60, 0x00, 0x0D, 0x8D, 0xA1, 0x08, uint32}  // 转矩模式下最大运行速度 1000rpm
        // {0x608, 0x2b, 0x6F, 0x60, 0x00, 0x0A, 0x00, 0x00, 0x00, uint16}, // 零速检测速度阈值 10rpm
        // {0x608, 0x2b, 0x70, 0x60, 0x00, 0x64, 0x00, 0x00, 0x00, uint16}  // 零速信号窗口时间(我认为是持续时间 100ms)
    };

    // 考虑到默认启动模式是速度，因此在修改模式前需要先canopen状态机停机
    // 先停止状态机轮询切换
    gStatus.l_canopenSM_sw = 0;

    // 正在运行过程中,则快速停机
    if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0237) {
        Controlword = 0x0002;
        // Update OperationMode
        if (targetOperationMode >= RECVSPEEDMODE && targetOperationMode <= TORQUEMODE) {
            if ((motionStatus.targetWorkmode != targetOperationMode) && (Modes_of_operation != targetOperationMode)) {
                motionStatus.targetWorkmode = targetOperationMode;
                Modes_of_operation = targetOperationMode;
            }
            sendOnePDOevent(&masterObjdict_Data, 0); 
            HAL_Delay(20);

            // 初始化SDO
            stopSYNC(&masterObjdict_Data);	
            for (cnt =0;cnt<TORQUESETUPLENG; cnt++) {
              canopen_send_sdo(message_torqueMode_sdo[cnt]);
              closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
              HAL_Delay(20);     
            }
            // setState(&masterObjdict_Data, Operational);
            // masterSendNMTstateChange(&masterObjdict_Data, can_var.slaveCANID, NMT_Start_Node);
            startSYNC(&masterObjdict_Data);

            gStatus.l_canopenSM_sw = 1; // 都在while(1)里顺序执行，这个好像没啥用，先这样吧
        } else {
            ret = 0xFFFF; // 报错
        }
    } else {
        // 在其他模式下暂时不允许切换 没有时间完善了
    }
    return ret;
}

// 速度模式canopen初始化
void canOpenInit(void)
{
    UNS8 cnt = 0;
    UNS16 sdoIndex;
    UNS8 subIndex;	
    UNS32 size = sizeof(UNS32); 
    UNS8 Config_Code[4] = {0x88, 0x01, 0x00, 0x80};
		UNS32 abortCode =0x00;

    setNodeId(&masterObjdict_Data, can_var.CANOpenMasterID);  // Master ID
    setState(&masterObjdict_Data, Initialisation);  
    setState(&masterObjdict_Data, Pre_operational); 

    stopSYNC(&masterObjdict_Data);	

		for (cnt = 0; cnt<PRESETSDOLENG; cnt++) {
			canopen_send_sdo(message_sdo[cnt]);
      closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
      HAL_Delay(20);
		}

    // Speed Mode Var
    for (cnt =0;cnt<SPEEDSETUPLENG; cnt++) {
      canopen_send_sdo(message_speedMode_sdo[cnt]);
      closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
      HAL_Delay(20);     
    }

		setState(&masterObjdict_Data, Operational);
    masterSendNMTstateChange(&masterObjdict_Data, can_var.slaveCANID, NMT_Start_Node);

		startSYNC(&masterObjdict_Data);
}

// SDO Transmit 
uint8_t canopen_send_sdo(uint16_t *message_sdo)
{
	  unsigned long abortCode=0;
    uint8_t      nodeID=0;          /* ID      */
    uint16_t    index=0;           /* 索引    */
    uint8_t     subIndex=0;        /* 子索引   */
    uint8_t     dataType=message_sdo[9];    /* 数据类型 */
    uint32_t    count= 0;           /* 数据长度 */
    uint8_t     data[4] = {0};    
    uint8_t     i=0;
		uint8_t 		ret = 0;
    
    switch (message_sdo[1]) {
        case 0x2f:count = 1;break;
        case 0x2b:count = 2;break;
        case 0x27:count = 3;break;
        case 0x23:count = 4;break;
        default:
          printf("CANOpen: SDO Data Length Error! \r\n");
          ret = 0xFF;
        break;
    }

		if (ret == 0xFF) {
				goto __end_label;
		}
	
    nodeID=(uint8_t)(message_sdo[0] & 0x7f);  // get low 7 bit (0-255)  
    index = (message_sdo[3] << 8);
    index += message_sdo[2];
    subIndex= (uint8_t) (message_sdo[4]);
		printf("CANOpen: SDO Send nodeID: 0x%x, index: 0x%x, subIndex: 0x%x \n\r", nodeID, index, subIndex);
    for (i=0; i<count; i++) {
        data[0+i]= (uint8_t) message_sdo[5+i];
    }
    //d; nodeId; index; subIndex; count; dataType; data[4]; useBlockMode;
    writeNetworkDict(&masterObjdict_Data, nodeID, index, subIndex, count, dataType, &data, 0);
    while(getWriteResultNetworkDict(&masterObjdict_Data, nodeID, &abortCode) != SDO_FINISHED) {
      continue;
    }
__end_label:
    return ret;
}

//Only Use before RUN
uint8_t canOpenSDOSendWithDelay(CO_Data *d, uint8_t slaveNodeId, uint16_t sdoIndex, uint8_t subIndex, uint8_t sendNum, uint8_t sendType, uint32_t *sendContext) 
{
  uint8_t ret = 0;
  unsigned long abortCode=0;
  ret = writeNetworkDict(d, slaveNodeId, sdoIndex, subIndex, sendNum, sendType, (void *)sendContext, 0);
  while(getWriteResultNetworkDict(d, slaveNodeId, &abortCode) != SDO_FINISHED) {
      break;
  }
  return ret;
}

uint8_t canopenDriverSpeedGive(short speedCmdRpm)
{
    uint8_t ret =0;
    int64_t sendSpeed = 0;

    if ((motionStatus.g_curOperationMode == RECVSPEEDMODE) && (motionStatus.g_DS402_SMStatus == 4)) {
        if ((speedCmdRpm <= MAX_ALLOWED_SPEED_RPM) && (speedCmdRpm >= MIN_ALLOWED_SPEED_RPM)) {
            Controlword = 0x0F;
            sendSpeed = speedCmdRpm;
            sendSpeed *= MOTOR_ENCODER_IDENTIFYWIDTH;
            sendSpeed = sendSpeed /60;

            Target_velocity = (int)sendSpeed; // update speed instruction pulse per second
            Modes_of_operation = RECVSPEEDMODE; // 速度模式
            sendOnePDOevent(&masterObjdict_Data, 1);  // TPDO2
            printf("UTC: %d ms CAS: %d ms, ETH update Speed :%d rpm\n\r", gTime.g_time_ms, gTime.l_time_ms, speedCmdRpm);
        } else {
            printf("CAS:  %d ms SpeedGiven OverFlow, which is 0x%d rpm\n\r!", gTime.l_time_ms, speedCmdRpm);
        }
    } else {
        printf("CAS:  %d ms OperationMode 0x%x disMatched or SystemStatus Wrong! \n\r!", gTime.l_time_ms, \
                                                                                          motionStatus.g_curOperationMode);
    }
    return ret;
}

// 下发的实际Nm*1000，CAN发出是额定转矩的0.001倍，最小分辨率 2.80/1000 = 0.0028Nm 量化误差
uint8_t canopenDriverTorqueGive(short torqueCmd)
{
    uint8_t ret =0;
    int sendTorquePartial = 0; // 给定转矩相对于额定转矩的倍率*1000

    if ((motionStatus.g_curOperationMode == TORQUEMODE) && (motionStatus.g_DS402_SMStatus == 4)) {
        if ((torqueCmd <= (MAX_ALLOWED_TORQUE_NM*1000)) && (torqueCmd >= (MIN_ALLOWED_TORQUE_NM*1000))) {
            Controlword = 0x0F;
            sendTorquePartial = (torqueCmd/DesignedTorqueNM); //量化误差

            Target_velocity = 0; // update speed instruction pulse per second
            Target_Torque = (short)sendTorquePartial;
            Modes_of_operation = TORQUEMODE; // 转矩模式
            sendOnePDOevent(&masterObjdict_Data, 1);  // TPDO2
            printf("UTC: %d ms CAS: %d ms, ETH update Torque :%d Nm, sendTorquePartial is %d\n\r", gTime.g_time_ms, gTime.l_time_ms, (torqueCmd/1000), sendTorquePartial);
        } else {
            printf("CAS:  %d ms TorqueGiven OverFlow, which is 0x%d Nm\n\r!", gTime.l_time_ms, (torqueCmd/1000));
        }
    } else {
        printf("CAS:  %d ms OperationMode 0x%x disMatched or SystemStatus Wrong! \n\r!", gTime.l_time_ms, \
                                                                                          motionStatus.g_curOperationMode);
    }
    return ret;
}


// 只要目标工作模式正常，且状态机检测被允许开启，则目标都是驱动系统状态前往RUN
// 速度模式下的控制自与转矩模式状态值仅 bit12不同，且仅代表速度是否为0，目前没用到，因此状态字可以不做区分
// 且控制字其实也完全一致
uint8_t canopenStateMachine(void)
{
    uint8_t ret = 0;
    if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0250) {
      Controlword = 0x06;
      Target_velocity = 0x00;
      Modes_of_operation = motionStatus.targetWorkmode;
      sendOnePDOevent(&masterObjdict_Data, 0);
      motionStatus.g_DS402_SMStatus = 1;
      printf ("CANOpen: Status 1  servo No Fault plz send Controlword 0x06\r\n");
    }

    if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0231) {
      Controlword = 0x07;
      Target_velocity = 0x00;
      Modes_of_operation = motionStatus.targetWorkmode;
      sendOnePDOevent(&masterObjdict_Data, 0);
      motionStatus.g_DS402_SMStatus = 2;
      printf ("CANOpen: Status 2 Servo Ready, plz send Controlword 0x07\r\n");
    }

    if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0233) {
      Controlword = 0x0F;
      Target_velocity = 0x00;
      Modes_of_operation = motionStatus.targetWorkmode;
      sendOnePDOevent(&masterObjdict_Data, 0);
			motionStatus.g_DS402_SMStatus = 3;
      printf ("CANOpen: Status 3 Waiting For Enable Servo, plz send Controlword 0x0F \r\n");
    }

    if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0237) {
      motionStatus.g_DS402_SMStatus = 4;
      printf ("CANOpen: Status 4 Servo RUN \r\n");
    } 
    // 当前停机中，等待指令
		if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x0217){
        motionStatus.g_DS402_SMStatus = 0; 
        // 没啥用
        if (gStatus.l_canopenSM_sw == 1) {
            if (motionStatus.targetWorkmode == RECVSPEEDMODE) {
                Controlword = 0x0F;
                Target_velocity = 0x00;
                sendOnePDOevent(&masterObjdict_Data, 0);
                printf ("CANOpen: System Setup Speed Mode in Status 0 QuickStop \r\n"); 
            } else if (motionStatus.targetWorkmode == TORQUEMODE) {
                Controlword = 0x0F;
                Target_Torque = 0x00;
                sendOnePDOevent(&masterObjdict_Data, 0);
                printf ("CANOpen: System Setup Torque Mode in Status 0 QuickStop \r\n");
            } else { // idle模式下，canopen状态机不进行状态转换
                printf ("CANOpen: System Setup Idle/POSI Mode in Status 0 QuickStop \r\n"); 
            }
        }
    }

		if ((motionStatus.motorStatusWord.Value & 0x3FF) == 0x021F){
      motionStatus.g_DS402_SMStatus = 5; 
      printf ("CANOpen: Status 5 Now in Fault! Plz check ERRORCODE \r\n");   
    } 

    return ret;
}

void canopenStatusMonitor(void) 
{
      motionStatus.g_curOperationMode = Modes_of_operation_display; 
      motionStatus.motorStatusWord.Value = Statusword;
      if (motionStatus.g_curOperationMode == RECVSPEEDMODE) {// speedMode
          motionStatus.motorCMD_speed.Value = Controlword;
          printf("CANOpen: SlaveNode OperationMode is 0x%x, Controlword is 0x%x, Statusword is 0x%x  \r\n", motionStatus.g_curOperationMode, \
                                                                                                    motionStatus.motorCMD_speed.Value, \
                                                                                                    motionStatus.motorStatusWord.Value);

      } else if (motionStatus.g_curOperationMode == TORQUEMODE) {
          motionStatus.motorCMD_torque.Value = Controlword;
          printf("CANOpen: SlaveNode OperationMode is 0x%x, Controlword is 0x%x, Statusword is 0x%x \r\n", motionStatus.g_curOperationMode, \
                                                                                                    motionStatus.motorCMD_torque.Value, \
                                                                                                    motionStatus.motorStatusWord.Value);
      }
      
      motionStatus.g_Speed = (Velocity_actual_value *60 / MOTOR_ENCODER_IDENTIFYWIDTH); //rpm
      // when motor is still, sensor will genarate Wrong Data of Speed 
      if ( motionStatus.g_Speed > MAX_ALLOWED_SPEED_RPM || motionStatus.g_Speed < MIN_ALLOWED_SPEED_RPM) {
            motionStatus.g_Speed = 0;
      }

      motionStatus.g_Torque =Torque_Actual_Value *DesignedTorqueNM; // 上报转矩值
      // 2024.06.29 和速度不同 转矩的允许波动范围很大 需要考虑真正堵转 和 临时 超过额定值的情况，先不做区分，全显示吧
      motionStatus.g_realTimeTorque = ((float)(Torque_Actual_Value *DesignedTorqueNM)/1000.0);

      // 不管什么模式，监视器里面都需要监视 实时速度 实时位置 实时转矩 平均相电流（后补）
      printf("%d ms: TPDO2: Current Work Operation is 0x%d, realTimefilterSpeed is : %d rpm, TargetSpeed is %d rpm, realTimeTorque is %fN.m \n\r", \
                                                                                      gTime.l_time_ms,  \
                                                                                      motionStatus.g_curOperationMode, \
                                                                                      motionStatus.g_Speed, \
                                                                                      (Target_velocity*MOTOR_ENCODER_IDENTIFYWIDTH/60),\
                                                                                      motionStatus.g_realTimeTorque);
}

#endif

uint8_t DACDriverSpeedGive(short speedCmdRpm)
{
    uint8_t ret =0;
    int32_t tempGivenVol = 0;
    if ((speedCmdRpm <= MAX_ALLOWED_SPEED_RPM) && (speedCmdRpm >= MIN_ALLOWED_SPEED_RPM)) {	
        tempGivenVol =  RPM2Vol_CONVERSE_COFF *speedCmdRpm + spdDownLimitVol ;
        HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
        printf("UTC: %d ms CAS: %d, update Speed :%d rpm\n\r", gTime.g_time_ms, gTime.l_time_ms, speedCmdRpm);
    }
    return ret;
}

uint32_t bissc_processDataAcquire(void)
{
    uint32_t retPosi = 0;
    static uint8_t errorCnt = 0;
    static uint32_t sensorCnt = 0;
    uint32_t sensorData = 0;
    //uint32_t primask = 0;

    //primask = enter_critical();
    HAL_SG_SenSorAcquire(&sensorData);
    //exit_critical(primask);

    if (can_var.CASNodeID == 0x01) {
        if ((sensorData >= POSIRANGESTART_LEFT) && (sensorData <= POSIRANGEEND_LEFT)) {
            retPosi = sensorData;
            gStatus.effectCnt++;
            //printf("BISS-C: %d ms %d Frame Acquire PosiData %d um \n\r", gTime.l_time_ms, sensorCnt, sensorData);
        } else {
            errorCnt++;
            gStatus.noeffectCnt++;
            BISSC_ReStore(&errorCnt); // 1ms内可以完成
            printf("BISS-C: %d ms %d Frame Acquire PosiData Error! \n\r", gTime.l_time_ms, sensorCnt);
        }
    } else if (can_var.CASNodeID == 0x02) { //右侧电机
        if ((sensorData >= POSIRANGESTART_RIGHT) && (sensorData <= POSIRANGEEND_RIGHT)) {
            retPosi = sensorData;
            //printf("BISS-C: %d ms %d Frame Acquire PosiData %d um \n\r", gTime.l_time_ms, sensorCnt, sensorData);
        } else {
            errorCnt++;
           BISSC_ReStore(&errorCnt); 
            printf("BISS-C: %d ms %d Frame Acquire PosiData Error! \n\r", gTime.l_time_ms, sensorCnt);
        }
    } else if (can_var.CASNodeID == 0x03) {  //横梁电机
        ; // idle
    } else {
        ; // idle
    }
    sensorCnt++;
    return retPosi; 
}

uint32_t enter_critical(void)
{
    // 保存当前 PRIMASK 值
    uint32_t regPrimask = __get_PRIMASK();
    // 关闭系统全局中断（其实就是将 PRIMASK 设为 1）
    __disable_irq();

    return regPrimask;
}

void exit_critical(uint32_t primask)
{
    // 恢复 PRIMASK
    __set_PRIMASK(primask);
}

/*
//////////////////////////////////////////////////////
// Keil 环境下实现（见 cmsis_armclang.h 文件）
__STATIC_FORCEINLINE void __set_PRIMASK(uint32_t priMask)
{
  __ASM volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

__STATIC_FORCEINLINE uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __ASM volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}

*/

