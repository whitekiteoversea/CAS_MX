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


#if HAL_CANOPEN_ENABLE

#define PRESETSDOLENG    (37) // Slave SDO配置
#define SPEEDSETUPLENG   (9)  // 速度模式下参数设置
#define TORQUESETUPLENG  (7)  // 转矩模式下参数设置

// 这里的SDO都是配置参数，所以数据类型都是UNS8, 如果是SDO发送实时速度，才会变更为INT等
uint16_t message_sdo[PRESETSDOLENG][10] = {
    // RPDO1
    {0x608, 0x23, 0x00, 0x14, 0x01, 0x08, 0x02, 0x00, 0x80, uint8}, // RPDO1 失能 类型：
    {0x608, 0x2f, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO1 传输类型 值变更触发
    {0x608, 0x2f, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // 清除原有映射内容
    {0x608, 0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60, uint8}, // 映射为控制字 0x6040
    //{0x608, 0x23, 0x00, 0x16, 0x02, 0x20, 0x00, 0xFF, 0x60, uint8}, // 映射为给定速度 0x60FF
    {0x608, 0x23, 0x00, 0x16, 0x02, 0x08, 0x00, 0x60, 0x60, uint8}, // 映射为运动模式 0x6060  无法由PDO动态运行更改，只能初始化SDO设置   
    {0x608, 0x2f, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00, uint8}, // 映射数量改为2
    {0x608, 0x23, 0x00, 0x14, 0x01, 0x08, 0x02, 0x00, 0x00, uint8}, // RPDO1 使能
    // RPDO2-4
    {0x608, 0x23, 0x01, 0x14, 0x01, 0x08, 0x03, 0x00, 0x80, uint8}, // RPDO2 失能
    {0x608, 0x2f, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00, uint8}, // RPDO2 传输类型
    {0x608, 0x2f, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, uint8}, // 清除原有映射内容
    {0x608, 0x23, 0x01, 0x16, 0x01, 0x20, 0x00, 0xFF, 0x60, uint8}, // 映射为给定速度 0x60FF
    {0x608, 0x2f, 0x01, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00, uint8}, // 映射数量改为1
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
#endif

void systemParaInit(void)
{
	unsigned char ret = 0x00;	
    gStatus.telmode = IDLEMODE; 
	
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

    // 预设工作模式初始化
    motionStatus.targetWorkmode = RECVSPEEDMODE; // 默认电机控制为速度模式

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

uint8_t CANRecvMsgDeal(CAN_HandleTypeDef *phcan, uint8_t CTRCode)
{
    UNUSED(phcan);
    uint8_t ret = 0;
   
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

    uint8_t targetOperationMode = 0;

    // Ext CAN ID
    CAN_ID_Union ext_ID;
    ext_ID.Value = 0;

    // recv Frame Number just for consective test
    tempFrameCnt = CAN1_RecData[0];
    tempFrameCnt <<= 8;
    tempFrameCnt |= CAN1_RecData[1];
    
    switch(CTRCode) {
        // SpeedGiven
        case CANSpeedCmd:
          curGivenSpeed = CAN1_RecData[4];
          curGivenSpeed <<= 8;
          curGivenSpeed |= CAN1_RecData[5];

// 电机运动指令给定方式 DAC or CANOpen
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
            if ((motionStatus.g_curOperationMode == RECVSPEEDMODE) && (motionStatus.g_DS402_SMStatus == 4)) {
                if (((short)curGivenSpeed <= MAX_ALLOWED_SPEED_RPM) && ((short)curGivenSpeed >= MIN_ALLOWED_SPEED_RPM)) {
                    Controlword = 0x0F;
                    Target_velocity = curGivenSpeed *MOTOR_ENCODER_IDENTIFYWIDTH /60; // update speed instruction pulse per second
                    Modes_of_operation = RECVSPEEDMODE;       // 速度模式
                    sendOnePDOevent(&masterObjdict_Data, 1);  // TPDO2
                    printf("UTC: %d ms CAS: %d ms, CAN1 Recv frameNum: %d, update Speed :%d rpm\n\r", gTime.g_time_ms,
                                                                                                      gTime.l_time_ms,
                                                                                                      tempFrameCnt,
                                                                                                      (short)curGivenSpeed);
                } else {
                    printf("CAS:  %d ms SpeedGiven OverFlow \n\r!", gTime.l_time_ms);
                }
            } else {
                printf("CAS:  %d ms OperationMode 0x%x disMatched or SystemStatus Wrong! \n\r!", gTime.l_time_ms, \
                                                                                                 motionStatus.g_curOperationMode);
            }
#endif
        break;
        
				// PreDictive Speed Mode 
        case CANSpeedPreCmd:
            curGivenSpeed = CAN1_RecData[5];
            curGivenSpeed <<= 8;
            curGivenSpeed |= CAN1_RecData[6];
        
#if HAL_DAC_ENABLE 
          if (((short)curGivenSpeed <= MAX_ALLOWED_SPEED_RPM) && ((short)curGivenSpeed >= MIN_ALLOWED_SPEED_RPM)) {	
              tempGivenVol = RPM2Vol_CONVERSE_COFF *curGivenSpeed + spdDownLimitVol ;
                      
              HAL_DAC8563_cmd_Write(3, 0, tempGivenVol);
              printf("UTC: %d ms CAS: %d, update Speed :%d rpm, t\n\r",  gTime.g_time_ms,
                                                                        gTime.l_time_ms,
                                                                        (short)curGivenSpeed);
          }
#else
          ;
#endif
        break;
        // 电机工作模式
        case CANOperationModeCmd:
            targetOperationMode = CAN1_RecData[4];
            if (targetOperationMode > 4) {
              printf ("UTC: %d ms, CAS: %d ms CAN1 Recv Error OperationMode! \r\n", gTime.g_time_ms, gTime.l_time_ms);
                goto __recv_end;
            }
            // 模式变更条件
            if (targetOperationMode != motionStatus.targetWorkmode) {
                // 先停止，开启抱闸，再重设工作模式
                Controlword = 0x04; // 自由停机，保持静止
                motionStatus.targetWorkmode = targetOperationMode;
                Modes_of_operation = motionStatus.targetWorkmode;
                sendOnePDOevent(&masterObjdict_Data, 0);  // TPDO1

                // 理论上应该设置一个延时检查点 推个标志位给主线程 or 主线程一直循环在检查 目标工作模式和实际工作模式是否匹配
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

            snddata[5] = (motionStatus.g_Distance & 0x00FF0000) >> 16; //um 26bit
            snddata[6] = (motionStatus.g_Distance & 0x0000FF00) >> 8;
            snddata[7] = (motionStatus.g_Distance & 0x000000FF);

            ext_ID.CAN_Frame_Union.CTRCode = CANPisiAcquireCmd; // position acquire pack type
            ext_ID.CAN_Frame_Union.MasterOrSlave = 0x00; // reply pack
            ext_ID.CAN_Frame_Union.NodeOrGroupID = PCNODEID;  // Send to PC Node

            HAL_CAN_Ext_Transmit(phcan, (void *)snddata, 8, ext_ID.Value);
            printf("UTC:%d ms CAS: %d ms CAN1 CurPosi is %d \n\r", gTime.g_time_ms, gTime.l_time_ms, motionStatus.g_Distance);
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

        break;
    }

__recv_end:
    return ret;   
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
}s
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
void canOpenInit(void)
{
    UNS8 cnt = 0;
    UNS16 sdoIndex;
    UNS8 subIndex;	
    UNS32 size = sizeof(UNS32); 
    UNS8 Config_Code[4] = {0x88, 0x01, 0x00, 0x80};
		UNS32 abortCode =0x00;

    setNodeId(&masterObjdict_Data, can_var.NodeID);  // Master ID
    setState(&masterObjdict_Data, Initialisation);  
    setState(&masterObjdict_Data, Pre_operational); 

    stopSYNC(&masterObjdict_Data);	

		for (cnt = 0; cnt<PRESETSDOLENG; cnt++) {
			canopen_send_sdo(message_sdo[cnt]);
      closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
      HAL_Delay(20);
		}

    // Speed Mode Var
    if (motionStatus.targetWorkmode == RECVSPEEDMODE) {
        for (cnt = 0;cnt<SPEEDSETUPLENG; cnt++) {
          canopen_send_sdo(message_speedMode_sdo[cnt]);
          closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
          HAL_Delay(20);     
        }
    } else if (motionStatus.targetWorkmode == TORQUEMODE) {
          for (cnt = 0;cnt<TORQUESETUPLENG; cnt++) {
          // canopen_send_sdo(message_speedMode_sdo[cnt]);
          closeSDOtransfer(&masterObjdict_Data, can_var.slaveCANID, SDO_CLIENT);
          HAL_Delay(20);     
        }
    } else {
      printf("CANOpen Init WorkMode Wrong! Plz check targetWorkMode \n\r");
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

// LCD监视参数初始化
void LCD_dispParaInit(void)
{
  uint8_t *str[] = {"Mode:", "rMode:", "Net: ", "tSpeed: ", 
                    "tTorque: ", "Status: ","Time(ms): ",
                    "rSpeed: ", "rTorque: "};
  uint8_t *statusStr = "Linked!";

  LCD_ShowString(0, 12, str[0], WHITE, BLACK, 12, 1); // 当前运行模式
  LCD_ShowString(0, 28, str[1], WHITE, BLACK, 12, 1); // 反馈运行模式
  LCD_ShowString(0, 44, str[2], WHITE, BLACK, 12, 1); // ETH连接状态
  LCD_ShowString(0, 60, str[3], WHITE, BLACK, 12, 1); // 当前给定速度
  LCD_ShowString(0, 76, str[4], WHITE, BLACK, 12, 1); // 当前给定转矩
  LCD_ShowString(0, 92, str[6], WHITE, BLACK, 12, 1); // 当前电机状态字
  LCD_ShowString(0, 135, str[7], WHITE, BLACK, 12, 1); // 本次开机运行时间

  LCD_ShowString(120, 60, str[8], WHITE, BLACK, 12, 1); // 当前运行速度
  LCD_ShowString(120, 76, str[9], WHITE, BLACK, 12, 1); // 当前给定转矩
}

void LCD_MonitorPara_Update(void)
{
    LCD_ShowChar(90, 12, motionStatus.targetWorkmode, WHITE, BLACK, 12, 0);      // 预设工作模式
    LCD_ShowChar(90, 28, motionStatus.g_curOperationMode, WHITE, BLACK, 12, 0);  // 反馈工作模式
    // LCD_ShowChar(50, 44, gStatus.telmode, WHITE, BLACK, 12, 0); // ETH连接状态 通过时间同步报文来监控

    LCD_ShowIntNum(90, 60, motionStatus.g_SpeedCmd_rpm, WHITE, BLACK, 12, 0);       // 预设速度 rpm
    // LCD_ShowInt(, 60, motionStatus.g_SpeedCmd_rpm, WHITE, BLACK, 12, 0);    

    LCD_ShowFloatNum1(90, 76, motionStatus.g_TorqueCmd_NM, WHITE, BLACK, 12, 0); // 预设转矩 N.m

    LCD_ShowIntNum(90, 92, motionStatus.motorStatusWord.Value, WHITE, BLACK, 12, 0);       // 状态字
    LCD_ShowIntNum(90, 135, gTime.l_time_ms, WHITE, BLACK, 12, 0);                  // 启动后本地时间 ms

}
