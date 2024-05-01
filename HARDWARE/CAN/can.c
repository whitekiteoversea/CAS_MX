#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"

//CAN1
uint8_t can1RecvFlag = 0;
CAN_ID_Union CANRecvFrame;
uint8_t CAN1_RecData[8] = {0}; //接收数据段

//CAN2(未使用)
uint8_t can2RecvFlag = 0;
CAN_ID_Union CAN2RecvFrame;
u8 can2NodeNum =0x1E; //传感器节点 CAN ID
uint8_t CAN2_RecData[8] = {0}; //接收数据段

u8 canlocalCharacNode = 0x01; //本地节点号

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为45M,如果设置CAN1_Mode_Init(1,5,9,6,1);
//则波特率为:45M/((5+9+1)*12)=250Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	u16 i=0;
	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1;//先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	RCC->AHB1ENR|=1<<0;  	//使能PORTA口时钟 
	GPIO_Set(GPIOA,PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA11,PA12,复用功能,上拉输出
	GPIO_AF_Set(GPIOA,11,9);//PA11,AF9
	GPIO_AF_Set(GPIOA,12,9);//PA12,AF9 	   

	RCC->AHB1ENR|=1<<1;  	//使能PORTB口时钟 
	GPIO_Set(GPIOB,PIN5|PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB5,PB6,复用功能,上拉输出
	GPIO_AF_Set(GPIOB,5,9);//PB5,AF9
	GPIO_AF_Set(GPIOB,6,9);//PB6,AF9 	   
	
	RCC->APB1ENR|=1<<25;//使能CAN1时钟 CAN1使用的是APB1的时钟(max:48M)
	CAN1->MCR=0x0000;	//退出睡眠模式(同时设置所有位为0)
	CAN1->MCR|=1<<0;		//请求CAN进入初始化模式
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100){
			return 2;//进入初始化模式失败
		}
	}
	CAN1->MCR|=0<<7;		//非时间触发通信模式
	CAN1->MCR|=0<<6;		//软件自动离线管理
	CAN1->MCR|=0<<5;		//睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)
	CAN1->MCR|=1<<4;		//禁止报文自动传送
	CAN1->MCR|=0<<3;		//报文不锁定,新的覆盖旧的
	CAN1->MCR|=0<<2;		//优先级由报文标识符决定
	CAN1->BTR=0x00000000;	//清除原来的设置.
	CAN1->BTR|=mode<<30;	//模式设置 0,普通模式;1,回环模式;
	CAN1->BTR|=tsjw<<24; 	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN1->BTR|=tbs2<<20; 	//Tbs2=tbs2+1个时间单位
	CAN1->BTR|=tbs1<<16;	//Tbs1=tbs1+1个时间单位
	CAN1->BTR|=brp<<0;  	//分频系数(Fdiv)为brp+1
	//波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->MCR&=~(1<<0);		//请求CAN退出初始化模式
	while((CAN1->MSR&1<<0)==1)
	{
			i++;
			if(i>0XFFF0)return 3;//退出初始化模式失败
	}

	RCC->APB1ENR|=1<<26;	//使能CAN2时钟 CAN2使用的是APB1的时钟(max:45M)
	CAN2->MCR=0x0000;			//退出睡眠模式(同时设置所有位为0)
	CAN2->MCR|=1<<0;			//请求CAN2进入初始化模式
	while((CAN2->MSR&1<<0)==0)
	{
			i++;
			if(i>100)return 2;//进入初始化模式失败
	}
	CAN2->MCR|=0<<7;		//非时间触发通信模式
	CAN2->MCR|=0<<6;		//软件自动离线管理
	CAN2->MCR|=0<<5;		//睡眠模式通过软件唤醒(清除CAN2->MCR的SLEEP位)
	CAN2->MCR|=1<<4;		//禁止报文自动传送
	CAN2->MCR|=0<<3;		//报文不锁定,新的覆盖旧的
	CAN2->MCR|=0<<2;		//优先级由报文标识符决定
	CAN2->BTR=0x00000000;	//清除原来的设置.
	CAN2->BTR|=mode<<30;	//模式设置 0,普通模式;1,回环模式;
	CAN2->BTR|=tsjw<<24; 	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN2->BTR|=tbs2<<20; 	//Tbs2=tbs2+1个时间单位
	CAN2->BTR|=tbs1<<16;	//Tbs1=tbs1+1个时间单位
	CAN2->BTR|=brp<<0;  	//分频系数(Fdiv)为brp+1
							//波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN2->MCR&=~(1<<0);		//请求CAN2退出初始化模式
	while((CAN2->MSR&1<<0)==1)
	{
			i++;
			if(i>0XFFF0)
					return 3;//退出初始化模式失败
	}

	//这里注意，CAN2默认的滤波器是从14开始的
	//这里默认初始化 CAN->FMR 为0x2A1C 0E01对应为14 
	CAN1->FMR|=1<<0;		//过滤器组工作在初始化模式
	
	CAN1->FA1R&=~(1<<14);	//过滤器14不激活
	CAN1->FA1R&=~(1<<0);	//过滤器0不激活
	
	CAN1->FS1R|=1<<14; 		//过滤器位宽为32位
	CAN1->FS1R|=1<<0; 		//过滤器位宽为32位
	
	CAN1->FM1R|=0<<14;		//过滤器14工作在标识符屏蔽位模式
	CAN1->FM1R|=0<<0;			//过滤器0工作在标识符屏蔽位模式
	
	CAN1->FFA1R&= ~(1<<0);		//过滤器0关联到FIFO0
	CAN1->FFA1R&= ~(1<<14);			//过滤器14关联到FIFO0
	
	CAN1->sFilterRegister[0].FR1=0X00000000;//32位ID
	CAN1->sFilterRegister[0].FR2=0X00000000;//32位MASK
	
	CAN1->sFilterRegister[14].FR1=0X00000000;//32位ID
	CAN1->sFilterRegister[14].FR2=0X00000000;//32位MASK
	
	CAN1->FA1R|=1<<0;			//激活过滤器0
	CAN1->FA1R|=1<<14;		//激活过滤器14
	
	CAN1->FMR&=0<<0;			//过滤器组进入正常模式

#if CAN1_RX0_INT_ENABLE
	//使用中断接收
	CAN1->IER|=1<<1;		//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(1,0,CAN1_RX0_IRQn,2);//组2
#endif
	
#if CAN2_RX0_INT_ENABLE
	//使用中断接收
	CAN2->IER|=1<<1;		//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(1,3,CAN2_RX0_IRQn,2);//组2
#endif
	return 0;
}   
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//*dat:数据指针.
//返回值:0~3,邮箱编号.0XFF,无有效邮箱.
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;			//邮箱0为空
	else if(CAN1->TSR&(1<<27))mbox=1;	//邮箱1为空
	else if(CAN1->TSR&(1<<28))mbox=2;	//邮箱2为空
	else return 0XFF;					//无空邮箱,无法发送 
	CAN1->sTxMailBox[mbox].TIR=0;		//清除之前的设置
	if(ide==0)	//标准帧
	{
		id&=0x7ff;//取低11位stdid
		id<<=21;		  
	}else		//扩展帧
	{
		id&=0X1FFFFFFF;//取低32位extid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F;//得到低四位
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;		   //设置DLC.
	//待发送数据存入邮箱.
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //请求发送邮箱数据
	return mbox;
}
//获得发送状态.
//mbox:邮箱编号;
//返回值:发送状态. 0,挂起;0X05,发送失败;0X07,发送成功.
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			//RQCP0
			sta |= CAN1->TSR&(1<<1);			//TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24);	//TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		//RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		//TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25);	//TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	//RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	//TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26);	//TME2
			break;
		default:
			sta=0X05;//邮箱号不对,肯定失败.
		break;
	}
	return sta;
} 
//得到在FIFO0/FIFO1中接收到的报文个数.
//fifox:0/1.FIFO编号;
//返回值:FIFO0/FIFO1中的报文个数.
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)
			return CAN1->RF0R&0x03; 
	else if(fifox==1)
			return CAN1->RF1R&0x03; 
	else return 0;
}
//接收数据
//fifox:邮箱号
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//dat:数据缓存区
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//得到标识符选择位的值  
 	if(*ide==0)//标准标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	   //扩展标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//得到远程发送请求值.
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;//得到DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//得到FMI
	//接收数据
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)
			CAN1->RF0R|=0X20;//释放FIFO0邮箱
	else if(fifox==1)
			CAN1->RF1R|=0X20;//释放FIFO1邮箱	 
}

#if CAN1_RX0_INT_ENABLE	//使能RX1中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	u8 rxbuf[8];
	u32 id;
	u8 ide,rtr,len;
	u8 i =0;
	uint16_t tempFrameCnt = 0;

	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,rxbuf); //接收CAN帧
	CANRecvFrame.Value = id;

	//广播或单播符合本机节点号
	if(CANRecvFrame.CAN_Frame_Union.NodeOrGroupID == canlocalCharacNode)
	{
		for(i = 0;i < len; i++)
		{
			CAN1_RecData[i] = rxbuf[i];
		}
		tempFrameCnt = CAN1_RecData[0];
		tempFrameCnt <<= 8;
		tempFrameCnt |= CAN1_RecData[1];
		printf("CAS: recv New Frame Type is %d\n\r", CANRecvFrame.CAN_Frame_Union.CTRCode);
		
		can1RecvFlag =1;
	}
	else
	{
		can1RecvFlag =0;
	}
}
#endif

#if CAN2_RX0_INT_ENABLE	//使能RX1中断

//中断服务函数			    
void CAN2_RX0_IRQHandler(void)
{
		u8 rxbuf[8];
		u32 id;
		u8 ide,rtr,len;
		u8 i =0;
	
		CAN2_Rx_Msg(0,&id,&ide,&rtr,&len,rxbuf); //接收CAN2
		CAN2RecvFrame.Value = id;
	
		//单播符合本机SEC节点号
		if((CAN2RecvFrame.CAN_Frame_Union.NodeOrGroupID == can2NodeNum) && 
			(CAN2RecvFrame.CAN_Frame_Union.MasterOrSlave == 0x00))
		{
				for(i = 0;i < len; i++)
				{
						CAN2_RecData[i] = rxbuf[i];
				}
				can2RecvFlag =1;
		}
		else
		{
				can2RecvFlag =0;
		}
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
		u8 mbox;
		u16 i=0;	  	 						       
		mbox = CAN1_Tx_Msg(0x12,0,0,len,msg);
		while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
		{
			i++;//等待发送结束
		}
		if(i>=0XFFF)
		{
			return 1;
		}
		//发送失败?
		return 0;										//发送成功;
}

//can口接收数据查询(未使用)
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
		u32 id;
		u8 ide,rtr,len; 
		if(CAN1_Msg_Pend(0)==0)
		{
				return 0;		//没有接收到数据,直接退出 	 
		}
		CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//读取数据
		if(id!=0x12||ide!=0||rtr!=0)len=0;			//接收错误	   
		return len;	
}

//发送CAN回复报文
u8 canSendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
	mbox = CAN1_Tx_Msg(canSendFrameID.Value, 0, 0, len, msg);
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
	{
		i++;//等待发送结束
	}
	if(i>=0XFFF)
	{
			return 1;
	}
	//发送失败?
	return 0;										//发送成功;
}

//CAN2接收
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
		//标准帧 数据帧
		*ide=CAN2->sFIFOMailBox[fifox].RIR&0x04;//得到标识符选择位的值
  
		if(*ide==0)//标准标识符
		{
				*id=CAN2->sFIFOMailBox[fifox].RIR>>21;
		}
		else	   //扩展标识符
		{
				*id=CAN2->sFIFOMailBox[fifox].RIR>>3;
		}
		
		*rtr=CAN2->sFIFOMailBox[fifox].RIR&0x02;	//得到远程发送请求值.
		*len=CAN2->sFIFOMailBox[fifox].RDTR&0x0F;//得到DLC
		//*fmi=(CAN2->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//得到FMI
		
		//接收数据
		dat[0]=CAN2->sFIFOMailBox[fifox].RDLR&0XFF;
		dat[1]=(CAN2->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
		dat[2]=(CAN2->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
		dat[3]=(CAN2->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
		dat[4]=CAN2->sFIFOMailBox[fifox].RDHR&0XFF;
		dat[5]=(CAN2->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
		dat[6]=(CAN2->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
		dat[7]=(CAN2->sFIFOMailBox[fifox].RDHR>>24)&0XFF; 
		
		if(fifox==0)
				CAN2->RF0R|=0X20;//释放FIFO0邮箱
		else if(fifox==1)
				CAN2->RF1R|=0X20;//释放FIFO1邮箱	 
}

//CAN2发送
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
		u8 mbox;	  
		if(CAN2->TSR&(1<<26))mbox=0;			//邮箱0为空
		else if(CAN2->TSR&(1<<27))mbox=1;	//邮箱1为空
		else if(CAN2->TSR&(1<<28))mbox=2;	//邮箱2为空
		else return 0XFF;					//无空邮箱,无法发送 
		CAN2->sTxMailBox[mbox].TIR=0;		//清除之前的设置
	
		if(ide==0)	//标准帧
		{
				id&=0x7ff;//取低11位stdid
				id<<=21;		  
		}
		else		//扩展帧
		{
				id&=0X1FFFFFFF;//取低32位extid
				id<<=3;									   
		}
		CAN2->sTxMailBox[mbox].TIR|=id;		 
		CAN2->sTxMailBox[mbox].TIR|=ide<<2;	  
		CAN2->sTxMailBox[mbox].TIR|=rtr<<1;
		len&=0X0F;//得到低四位
		CAN2->sTxMailBox[mbox].TDTR&=~(0X0000000F);
		CAN2->sTxMailBox[mbox].TDTR|=len;		   //设置DLC.
		//待发送数据存入邮箱.
		CAN2->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
									((u32)dat[6]<<16)|
									((u32 )dat[5]<<8)|
									((u32)dat[4]));
		CAN2->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
									((u32)dat[2]<<16)|
									((u32)dat[1]<<8)|
									((u32)dat[0]));
		CAN2->sTxMailBox[mbox].TIR|=1<<0; //请求发送邮箱数据
		return mbox;
}

//发送CAN2回复报文
u8 can2SendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
	mbox = CAN2_Tx_Msg(canSendFrameID.Value, 0, 0, len, msg);
	while((CAN2_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
	{
			i++;//等待发送结束
	}
	if(i>=0XFFF)
	{
			return 1;
	}
	//发送失败?
	return 0;										//发送成功;
}

u8 CAN2_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN2->TSR&(1<<0);			//RQCP0
			sta |= CAN2->TSR&(1<<1);			//TXOK0
			sta |=((CAN2->TSR&(1<<26))>>24);	//TME0
			break;
		case 1: 
			sta |= CAN2->TSR&(1<<8)>>8;		//RQCP1
			sta |= CAN2->TSR&(1<<9)>>8;		//TXOK1
			sta |=((CAN2->TSR&(1<<27))>>25);	//TME1	   
			break;
		case 2: 
			sta |= CAN2->TSR&(1<<16)>>16;	//RQCP2
			sta |= CAN2->TSR&(1<<17)>>16;	//TXOK2
			sta |=((CAN2->TSR&(1<<28))>>26);	//TME2
			break;
		default:
			sta=0X05;//邮箱号不对,肯定失败.
		break;
	}
	return sta;
} 

//未使用
u8 CAN2_Receive_Msg(u8 *buf)							//接收数据
{
	u32 id;
	u8 ide,rtr,len; 
	if(CAN2_Msg_Pend(1)==0)
	{
			return 0;		//没有接收到数据,直接退出 	 
	}
	CAN2_Rx_Msg(1,&id,&ide,&rtr,&len,buf); 	//读取数据
	if(id!=0x12||ide!=0||rtr!=0)len=0;			//接收错误	   
	return len;
}

u8 CAN2_Msg_Pend(u8 fifox)
{
	if(fifox==0)
			return CAN2->RF0R&0x03; 
	else if(fifox==1)
			return CAN2->RF1R&0x03; 
	else 
			return 0;
}
