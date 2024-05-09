#include "can.h"
#include "led.h"
#include "usart.h"

CAN_ID_Union CAN1RecvFrame;
CAN_ID_Union CAN2RecvFrame;
uint8_t CAN1_RecData[8] = {0};
uint8_t CAN2_RecData[8] = {0}; 

uint8_t HAL_CAN_Ext_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Ext_ID)
{
    uint32_t txmailbox = 0;
    uint32_t offset = 0;
    CAN_TxHeaderTypeDef hdr;

    hdr.IDE = CAN_ID_EXT;													
    hdr.RTR = CAN_RTR_DATA;													
    hdr.StdId = 0;														
    hdr.ExtId = Ext_ID;									
    hdr.TransmitGlobalTime = DISABLE;

    while (len != 0)
    {
        hdr.DLC = len > 8 ? 8 : len;			// 数据长度
		if (HAL_CAN_AddTxMessage(hcan, &hdr, ((uint8_t *)buf) + offset, &txmailbox) != HAL_OK) {
			return 1;
		}
        offset += hdr.DLC;
        len -= hdr.DLC;
    }
    return 0;
}

uint8_t HAL_CAN_Std_Transmit(CAN_HandleTypeDef *hcan, const void* buf, uint32_t len, uint32_t Std_ID)
{
    uint32_t txmailbox = 0;
    uint32_t offset = 0;
    CAN_TxHeaderTypeDef hdr;

    hdr.IDE = CAN_ID_STD;													
    hdr.RTR = CAN_RTR_DATA;													
    hdr.StdId = 0;														
    hdr.ExtId = Std_ID;									
    hdr.TransmitGlobalTime = DISABLE;

    while (len != 0)
    {
        hdr.DLC = len > 8 ? 8 : len;			// 数据长度
		if (HAL_CAN_AddTxMessage(hcan, &hdr, ((uint8_t *)buf) + offset, &txmailbox) != HAL_OK) {
			return 1;
		}
        offset += hdr.DLC;
        len -= hdr.DLC;
    }
    return 0;
}

#if REG_CAN_ENABLE        

//CAN1
uint8_t can1RecvFlag = 0;
CAN_ID_Union CANRecvFrame;
uint8_t CAN1_RecData[8] = {0}; //�������ݶ�

//CAN2(δʹ��)
uint8_t can2RecvFlag = 0;
CAN_ID_Union CAN2RecvFrame;
u8 can2NodeNum =0x1E; //�������ڵ� CAN ID
uint8_t CAN2_RecData[8] = {0}; //�������ݶ�

u8 canlocalCharacNode = 0x01; //���ؽڵ��

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3;
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ45M,�������CAN1_Mode_Init(1,5,9,6,1);
//������Ϊ:45M/((5+9+1)*12)=250Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	u16 i=0;
	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1;//�ȼ�ȥ1.����������
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	RCC->AHB1ENR|=1<<0;  	//ʹ��PORTA��ʱ�� 
	GPIO_Set(GPIOA,PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA11,PA12,���ù���,�������
	GPIO_AF_Set(GPIOA,11,9);//PA11,AF9
	GPIO_AF_Set(GPIOA,12,9);//PA12,AF9 	   

	RCC->AHB1ENR|=1<<1;  	//ʹ��PORTB��ʱ�� 
	GPIO_Set(GPIOB,PIN5|PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB5,PB6,���ù���,�������
	GPIO_AF_Set(GPIOB,5,9);//PB5,AF9
	GPIO_AF_Set(GPIOB,6,9);//PB6,AF9 	   
	
	RCC->APB1ENR|=1<<25;//ʹ��CAN1ʱ�� CAN1ʹ�õ���APB1��ʱ��(max:48M)
	CAN1->MCR=0x0000;	//�˳�˯��ģʽ(ͬʱ��������λΪ0)
	CAN1->MCR|=1<<0;		//����CAN�����ʼ��ģʽ
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100){
			return 2;//�����ʼ��ģʽʧ��
		}
	}
	CAN1->MCR|=0<<7;		//��ʱ�䴥��ͨ��ģʽ
	CAN1->MCR|=0<<6;		//�����Զ����߹���
	CAN1->MCR|=0<<5;		//˯��ģʽͨ����������(���CAN1->MCR��SLEEPλ)
	CAN1->MCR|=1<<4;		//��ֹ�����Զ�����
	CAN1->MCR|=0<<3;		//���Ĳ�����,�µĸ��Ǿɵ�
	CAN1->MCR|=0<<2;		//���ȼ��ɱ��ı�ʶ������
	CAN1->BTR=0x00000000;	//���ԭ��������.
	CAN1->BTR|=mode<<30;	//ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
	CAN1->BTR|=tsjw<<24; 	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
	CAN1->BTR|=tbs2<<20; 	//Tbs2=tbs2+1��ʱ�䵥λ
	CAN1->BTR|=tbs1<<16;	//Tbs1=tbs1+1��ʱ�䵥λ
	CAN1->BTR|=brp<<0;  	//��Ƶϵ��(Fdiv)Ϊbrp+1
	//������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->MCR&=~(1<<0);		//����CAN�˳���ʼ��ģʽ
	while((CAN1->MSR&1<<0)==1)
	{
			i++;
			if(i>0XFFF0)return 3;//�˳���ʼ��ģʽʧ��
	}

	RCC->APB1ENR|=1<<26;	//ʹ��CAN2ʱ�� CAN2ʹ�õ���APB1��ʱ��(max:45M)
	CAN2->MCR=0x0000;			//�˳�˯��ģʽ(ͬʱ��������λΪ0)
	CAN2->MCR|=1<<0;			//����CAN2�����ʼ��ģʽ
	while((CAN2->MSR&1<<0)==0)
	{
			i++;
			if(i>100)return 2;//�����ʼ��ģʽʧ��
	}
	CAN2->MCR|=0<<7;		//��ʱ�䴥��ͨ��ģʽ
	CAN2->MCR|=0<<6;		//�����Զ����߹���
	CAN2->MCR|=0<<5;		//˯��ģʽͨ����������(���CAN2->MCR��SLEEPλ)
	CAN2->MCR|=1<<4;		//��ֹ�����Զ�����
	CAN2->MCR|=0<<3;		//���Ĳ�����,�µĸ��Ǿɵ�
	CAN2->MCR|=0<<2;		//���ȼ��ɱ��ı�ʶ������
	CAN2->BTR=0x00000000;	//���ԭ��������.
	CAN2->BTR|=mode<<30;	//ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
	CAN2->BTR|=tsjw<<24; 	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
	CAN2->BTR|=tbs2<<20; 	//Tbs2=tbs2+1��ʱ�䵥λ
	CAN2->BTR|=tbs1<<16;	//Tbs1=tbs1+1��ʱ�䵥λ
	CAN2->BTR|=brp<<0;  	//��Ƶϵ��(Fdiv)Ϊbrp+1
							//������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN2->MCR&=~(1<<0);		//����CAN2�˳���ʼ��ģʽ
	while((CAN2->MSR&1<<0)==1)
	{
			i++;
			if(i>0XFFF0)
					return 3;//�˳���ʼ��ģʽʧ��
	}

	//����ע�⣬CAN2Ĭ�ϵ��˲����Ǵ�14��ʼ��
	//����Ĭ�ϳ�ʼ�� CAN->FMR Ϊ0x2A1C 0E01��ӦΪ14 
	CAN1->FMR|=1<<0;		//�������鹤���ڳ�ʼ��ģʽ
	
	CAN1->FA1R&=~(1<<14);	//������14������
	CAN1->FA1R&=~(1<<0);	//������0������
	
	CAN1->FS1R|=1<<14; 		//������λ��Ϊ32λ
	CAN1->FS1R|=1<<0; 		//������λ��Ϊ32λ
	
	CAN1->FM1R|=0<<14;		//������14�����ڱ�ʶ������λģʽ
	CAN1->FM1R|=0<<0;			//������0�����ڱ�ʶ������λģʽ
	
	CAN1->FFA1R&= ~(1<<0);		//������0������FIFO0
	CAN1->FFA1R&= ~(1<<14);			//������14������FIFO0
	
	CAN1->sFilterRegister[0].FR1=0X00000000;//32λID
	CAN1->sFilterRegister[0].FR2=0X00000000;//32λMASK
	
	CAN1->sFilterRegister[14].FR1=0X00000000;//32λID
	CAN1->sFilterRegister[14].FR2=0X00000000;//32λMASK
	
	CAN1->FA1R|=1<<0;			//���������0
	CAN1->FA1R|=1<<14;		//���������14
	
	CAN1->FMR&=0<<0;			//���������������ģʽ

#if CAN1_RX0_INT_ENABLE
	//ʹ���жϽ���
	CAN1->IER|=1<<1;		//FIFO0��Ϣ�Һ��ж�����.	    
	MY_NVIC_Init(1,0,CAN1_RX0_IRQn,2);//��2
#endif
	
#if CAN2_RX0_INT_ENABLE
	//ʹ���жϽ���
	CAN2->IER|=1<<1;		//FIFO0��Ϣ�Һ��ж�����.	    
	MY_NVIC_Init(1,3,CAN2_RX0_IRQn,2);//��2
#endif
	return 0;
}   
//id:��׼ID(11λ)/��չID(11λ+18λ)	    
//ide:0,��׼֡;1,��չ֡
//rtr:0,����֡;1,Զ��֡
//len:Ҫ���͵����ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
//*dat:����ָ��.
//����ֵ:0~3,������.0XFF,����Ч����.
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;			//����0Ϊ��
	else if(CAN1->TSR&(1<<27))mbox=1;	//����1Ϊ��
	else if(CAN1->TSR&(1<<28))mbox=2;	//����2Ϊ��
	else return 0XFF;					//�޿�����,�޷����� 
	CAN1->sTxMailBox[mbox].TIR=0;		//���֮ǰ������
	if(ide==0)	//��׼֡
	{
		id&=0x7ff;//ȡ��11λstdid
		id<<=21;		  
	}else		//��չ֡
	{
		id&=0X1FFFFFFF;//ȡ��32λextid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F;//�õ�����λ
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;		   //����DLC.
	//���������ݴ�������.
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //��������������
	return mbox;
}
//��÷���״̬.
//mbox:������;
//����ֵ:����״̬. 0,����;0X05,����ʧ��;0X07,���ͳɹ�.
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
			sta=0X05;//����Ų���,�϶�ʧ��.
		break;
	}
	return sta;
} 
//�õ���FIFO0/FIFO1�н��յ��ı��ĸ���.
//fifox:0/1.FIFO���;
//����ֵ:FIFO0/FIFO1�еı��ĸ���.
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)
			return CAN1->RF0R&0x03; 
	else if(fifox==1)
			return CAN1->RF1R&0x03; 
	else return 0;
}
//��������
//fifox:�����
//id:��׼ID(11λ)/��չID(11λ+18λ)	    
//ide:0,��׼֡;1,��չ֡
//rtr:0,����֡;1,Զ��֡
//len:���յ������ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
//dat:���ݻ�����
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//�õ���ʶ��ѡ��λ��ֵ  
 	if(*ide==0)//��׼��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	   //��չ��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//�õ�Զ�̷�������ֵ.
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;//�õ�DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//�õ�FMI
	//��������
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)
			CAN1->RF0R|=0X20;//�ͷ�FIFO0����
	else if(fifox==1)
			CAN1->RF1R|=0X20;//�ͷ�FIFO1����	 
}

#if CAN1_RX0_INT_ENABLE	//ʹ��RX1�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
	u8 rxbuf[8];
	u32 id;
	u8 ide,rtr,len;
	u8 i =0;
	uint16_t tempFrameCnt = 0;

	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,rxbuf); //����CAN֡
	CANRecvFrame.Value = id;

	//�㲥�򵥲����ϱ����ڵ��
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

#if CAN2_RX1_INT_ENABLE	//ʹ��RX1�ж�

//�жϷ�����			    
void CAN2_RX1_IRQHandler(void)
{
		u8 rxbuf[8];
		u32 id;
		u8 ide,rtr,len;
		u8 i =0;
	
		HAL_CAN

		CAN2_Rx_Msg(0,&id,&ide,&rtr,&len,rxbuf); //����CAN2
		CAN2RecvFrame.Value = id;
	
		//�������ϱ���SEC�ڵ��
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

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
		u8 mbox;
		u16 i=0;	  	 						       
		mbox = CAN1_Tx_Msg(0x12,0,0,len,msg);
		while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
		{
			i++;//�ȴ����ͽ���
		}
		if(i>=0XFFF)
		{
			return 1;
		}
		//����ʧ��?
		return 0;										//���ͳɹ�;
}

//can�ڽ������ݲ�ѯ(δʹ��)
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
		u32 id;
		u8 ide,rtr,len; 
		if(CAN1_Msg_Pend(0)==0)
		{
				return 0;		//û�н��յ�����,ֱ���˳� 	 
		}
		CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//��ȡ����
		if(id!=0x12||ide!=0||rtr!=0)len=0;			//���մ���	   
		return len;	
}

//����CAN�ظ�����
u8 canSendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
	mbox = CAN1_Tx_Msg(canSendFrameID.Value, 0, 0, len, msg);
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
	{
		i++;//�ȴ����ͽ���
	}
	if(i>=0XFFF)
	{
			return 1;
	}
	//����ʧ��?
	return 0;										//���ͳɹ�;
}

//CAN2����
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
		//��׼֡ ����֡
		*ide=CAN2->sFIFOMailBox[fifox].RIR&0x04;//�õ���ʶ��ѡ��λ��ֵ
  
		if(*ide==0)//��׼��ʶ��
		{
				*id=CAN2->sFIFOMailBox[fifox].RIR>>21;
		}
		else	   //��չ��ʶ��
		{
				*id=CAN2->sFIFOMailBox[fifox].RIR>>3;
		}
		
		*rtr=CAN2->sFIFOMailBox[fifox].RIR&0x02;	//�õ�Զ�̷�������ֵ.
		*len=CAN2->sFIFOMailBox[fifox].RDTR&0x0F;//�õ�DLC
		//*fmi=(CAN2->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//�õ�FMI
		
		//��������
		dat[0]=CAN2->sFIFOMailBox[fifox].RDLR&0XFF;
		dat[1]=(CAN2->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
		dat[2]=(CAN2->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
		dat[3]=(CAN2->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
		dat[4]=CAN2->sFIFOMailBox[fifox].RDHR&0XFF;
		dat[5]=(CAN2->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
		dat[6]=(CAN2->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
		dat[7]=(CAN2->sFIFOMailBox[fifox].RDHR>>24)&0XFF; 
		
		if(fifox==0)
				CAN2->RF0R|=0X20;//�ͷ�FIFO0����
		else if(fifox==1)
				CAN2->RF1R|=0X20;//�ͷ�FIFO1����	 
}

//CAN2����
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
		u8 mbox;	  
		if(CAN2->TSR&(1<<26))mbox=0;			//����0Ϊ��
		else if(CAN2->TSR&(1<<27))mbox=1;	//����1Ϊ��
		else if(CAN2->TSR&(1<<28))mbox=2;	//����2Ϊ��
		else return 0XFF;					//�޿�����,�޷����� 
		CAN2->sTxMailBox[mbox].TIR=0;		//���֮ǰ������
	
		if(ide==0)	//��׼֡
		{
				id&=0x7ff;//ȡ��11λstdid
				id<<=21;		  
		}
		else		//��չ֡
		{
				id&=0X1FFFFFFF;//ȡ��32λextid
				id<<=3;									   
		}
		CAN2->sTxMailBox[mbox].TIR|=id;		 
		CAN2->sTxMailBox[mbox].TIR|=ide<<2;	  
		CAN2->sTxMailBox[mbox].TIR|=rtr<<1;
		len&=0X0F;//�õ�����λ
		CAN2->sTxMailBox[mbox].TDTR&=~(0X0000000F);
		CAN2->sTxMailBox[mbox].TDTR|=len;		   //����DLC.
		//���������ݴ�������.
		CAN2->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
									((u32)dat[6]<<16)|
									((u32 )dat[5]<<8)|
									((u32)dat[4]));
		CAN2->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
									((u32)dat[2]<<16)|
									((u32)dat[1]<<8)|
									((u32)dat[0]));
		CAN2->sTxMailBox[mbox].TIR|=1<<0; //��������������
		return mbox;
}

//����CAN2�ظ�����
u8 can2SendMsg(CAN_ID_Union canSendFrameID, u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
	mbox = CAN2_Tx_Msg(canSendFrameID.Value, 0, 0, len, msg);
	while((CAN2_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))
	{
			i++;//�ȴ����ͽ���
	}
	if(i>=0XFFF)
	{
			return 1;
	}
	//����ʧ��?
	return 0;										//���ͳɹ�;
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
			sta=0X05;//����Ų���,�϶�ʧ��.
		break;
	}
	return sta;
} 

//δʹ��
u8 CAN2_Receive_Msg(u8 *buf)							//��������
{
	u32 id;
	u8 ide,rtr,len; 
	if(CAN2_Msg_Pend(1)==0)
	{
			return 0;		//û�н��յ�����,ֱ���˳� 	 
	}
	CAN2_Rx_Msg(1,&id,&ide,&rtr,&len,buf); 	//��ȡ����
	if(id!=0x12||ide!=0||rtr!=0)len=0;			//���մ���	   
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

#endif
