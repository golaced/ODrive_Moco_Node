#include "can.h"
#include "delay.h"
#include "include.h"
#include "my_math.h"
#include "math.h"

_LEG_MOTOR leg_motor[4];
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ. @ref CAN_synchronisation_jump_width   ��Χ: ; CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   @ref CAN_time_quantum_in_bit_segment_2 ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   @refCAN_time_quantum_in_bit_segment_1  ��Χ: ;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//������=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
//mode: @ref CAN_operating_mode ��Χ��CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((1+6+7)*6)=500Kbps   0 7 12 4 
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;
uint32_t can1_rx_id;
u8 canbuft1[8];
u8 canbufr1[8];

#define USE_ID_CHECK1 0
u32  slave_id1 = 99 ; 
float cnt_rst1=0;
int can1_rx_cnt;


void CAN_motor_init(void){
	char i;
	
	for(i=0;i<4;i++){
		leg_motor[i].connect=0;
		leg_motor[i].motor_en=0;
		//leg_motor[i].motor_mode=MOTOR_MODE_CURRENT;	
		leg_motor[i].motor_mode=MOTOR_MODE_T;//ģʽѡ��
		
		reset_current_cmd(i);
		
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=1.8;//�������
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=12;//������
		
		leg_motor[0].q_reset[0]=0;//�����ƫ��λ�Ƕ�  +-
		leg_motor[0].q_reset[1]=270+8;//                0~360

		leg_motor[1].q_reset[0]=-90;//�����ƫ��λ�Ƕ�
		leg_motor[1].q_reset[1]=180+18;

		leg_motor[2].q_reset[0]=0;//�����ƫ��λ�Ƕ�
		leg_motor[2].q_reset[1]=270+8;

		leg_motor[3].q_reset[0]=-90;//�����ƫ��λ�Ƕ�
		leg_motor[3].q_reset[1]=180+18;
	}
}

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	  
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   

float can_dt[4]={0};
void CAN1_RX0_IRQHandler(void)
{
	static int cnt_rx,led_flag;
	char can_node_id=0;
	char temp;
	CanRxMsg RxMessage;
	int i=0,id=0;
	CAN_Receive(CAN1, 0, &RxMessage);
	cnt_rst1=0;
	if(cnt_rx++>500&&1){cnt_rx=0;
	led_flag=!led_flag;
	//LEDRGB_RED(led_flag);
	}
	for(i=0;i<8;i++)
		canbufr1[i]=0x00;
	for(i=0;i<RxMessage.DLC;i++)
		canbufr1[i]=RxMessage.Data[i];
	can1_rx_id=RxMessage.StdId;	
	
	if(RxMessage.DLC){
			 //LEG FR
		 if ( RxMessage.StdId == 0x300+0 ) 
			{
				can_dt[0] = Get_Cycle_T(10); 
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}
			if ( RxMessage.StdId == 0x310+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}
			if ( RxMessage.StdId == 0x400+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				leg_motor[id].temp[1]=*(canbufr1+7);
			}
			//LEG HR
			if ( RxMessage.StdId == 0x300+1 ) 
			{can_dt[1] = Get_Cycle_T(11); 
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}
			if ( RxMessage.StdId == 0x310+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}			
			if ( RxMessage.StdId == 0x400+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				leg_motor[id].temp[1]=*(canbufr1+7);
			}
				 //LEG FL
		 if ( RxMessage.StdId == 0x300+2 ) 
			{can_dt[2] = Get_Cycle_T(12); 
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}
			if ( RxMessage.StdId == 0x310+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}			
			if ( RxMessage.StdId == 0x400+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				leg_motor[id].temp[1]=*(canbufr1+7);
			}
			//LEG HL
			if ( RxMessage.StdId == 0x300+3) 
			{can_dt[3] = Get_Cycle_T(13); 
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}
			if ( RxMessage.StdId == 0x310+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=1;
			}			
			if ( RxMessage.StdId == 0x400+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				leg_motor[id].temp[1]=*(canbufr1+7);
			}
		}
	can1_rx_cnt++;
}


//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;//0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i]; 
    can1_rx_id=RxMessage.StdId;	
	return RxMessage.DLC;	
}


uint32_t can2_rx_id;
u8 canbuft2[8];
u8 canbufr2[8];

#define USE_ID_CHECK2 0
u32  slave_id2 = 99 ; 
float cnt_rst2;
int can2_rx_cnt;
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN2_RX0_INT_ENABLE
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN2_RX0_IRQHandler(void)
{
	static int cnt_rx,led_flag,id=0;
	CanRxMsg RxMessage;
	int i=0;
	char temp;
	CAN_Receive(CAN2, 0, &RxMessage);
	cnt_rst2=0;
	if(cnt_rx++>500&&1){cnt_rx=0;
	led_flag=!led_flag;
	//LEDRGB_BLUE(led_flag);
	}
	for(i=0;i<8;i++)
		canbufr2[i]=0x00;
	for(i=0;i<RxMessage.DLC;i++)
		canbufr2[i]=RxMessage.Data[i];
	can2_rx_id=RxMessage.StdId;	
	if(RxMessage.DLC==8){
				 //LEG FR
		 if ( RxMessage.StdId == 0x300+0 ) 
			{
				can_dt[0] = Get_Cycle_T(10); 	
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x310+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x400+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				leg_motor[id].temp[1]=*(canbufr2+7);
			}
			//LEG HR
			if ( RxMessage.StdId == 0x300+1 ) 
			{
				can_dt[1] = Get_Cycle_T(11); 
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x310+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x400+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				leg_motor[id].temp[1]=*(canbufr2+7);
			}
				 //LEG FL
		 if ( RxMessage.StdId == 0x300+2 ) 
			{
				can_dt[2] = Get_Cycle_T(12); 
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x310+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x400+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				leg_motor[id].temp[1]=*(canbufr2+7);
			}
			//LEG HL
			if ( RxMessage.StdId == 0x300+3) 
			{
				can_dt[3] = Get_Cycle_T(13); 
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}
			if ( RxMessage.StdId == 0x310+3 ) 
			{	
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/20.;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/20.;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/1000.;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/1000.;
				leg_motor[id].can_bus_id=2;
			}			
			if ( RxMessage.StdId == 0x400+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				leg_motor[id].temp[1]=*(canbufr2+7);
			}
	}
	can2_rx_cnt++;
}

#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	  can2_rx_id=RxMessage.StdId;	
	return RxMessage.DLC;	
}
//----------------------------------------------------------------------------------------------------------------------
void reset_current_cmd(char id)
{
	leg_motor[id].set_t[0]=leg_motor[id].set_t[1]=leg_motor[id].set_t[2]=0;
	leg_motor[id].set_i[0]=leg_motor[id].set_i[1]=leg_motor[id].set_i[2]=0;
}


char reset_err_flag=0;
void CAN_set_torque(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	leg_motor[id].set_t[0]=LIMIT(leg_motor[id].set_t[0],-leg_motor[id].max_t[0],leg_motor[id].max_t[0]);
	leg_motor[id].set_t[1]=LIMIT(leg_motor[id].set_t[1],-leg_motor[id].max_t[1],leg_motor[id].max_t[1]);

	_temp=leg_motor[id].set_t[0]*100*leg_motor[id].ready[0];
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].set_t[1]*100*leg_motor[id].ready[1];
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];//������
	canbuft1[5]=leg_motor[id].motor_en;//���ʹ��
	canbuft1[6]=leg_motor[id].reset_q;//�������
	canbuft1[7]=reset_err_flag;//��λ����
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,0x100+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,0x100+id);		
}

void CAN_set_zero_off(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	
	_temp=leg_motor[id].q_reset[0]*10;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[1]*10;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=0;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=reset_err_flag;
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,0x500+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,0x500+id);		
}

void CAN_set_current(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	leg_motor[id].set_i[0]=LIMIT(leg_motor[id].set_i[0],-leg_motor[id].max_i[0],leg_motor[id].max_i[0]);
	leg_motor[id].set_i[1]=LIMIT(leg_motor[id].set_i[1],-leg_motor[id].max_i[1],leg_motor[id].max_i[1]);

	_temp=leg_motor[id].set_i[0]*100*leg_motor[id].ready[0];
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].set_i[1]*100*leg_motor[id].ready[1];
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=0;
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,0x200+id);	
	else
	res=CAN2_Send_Msg(canbuft1,8,0x200+id);	
}

void CAN_set_current_zero(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};

	_temp=0;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=0;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=0;

	res=CAN2_Send_Msg(canbuft1,8,0x100+id);	
}

void CAN_reset_q(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_reset[0]*10;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[1]*10;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[2]*10;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA3);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB3);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA3);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB3);	
	break;
	}
}


void CAN_board(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_set[0]*10;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[1]*10;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[2]*10;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA6);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB6);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA6);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB6);	
	break;
	}
}

void CAN_set_q(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_set[0]*10;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[1]*10;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[2]*10;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA2);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB2);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA2);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB2);	
	break;
	}
}


void CAN_set_force(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].f_set[0]*10;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].f_set[1]*10;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].f_set[2]*10;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA4);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB4);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA4);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB4);	
	break;
	}
}

void CAN_motor_sm(float dt)
{
	static int cnt_[4]={0};
	static float timer_[4]={0};
	static int state_[4]={0};
	char i=0;
	for(i=0;i<4;i++){
	switch(state_[i]){
		case 0:
			CAN_set_current_zero(i);
			if(leg_motor[i].connect)
			{
				cnt_[i]=0;
				state_[i]++;
			}
		break;	
		case 1:	
			if(!leg_motor[i].connect)
				state_[i]=0;
			else{
				if(leg_motor[i].motor_mode==MOTOR_MODE_T)
					CAN_set_torque(i);
				else if(leg_motor[i].motor_mode==MOTOR_MODE_CURRENT)
					CAN_set_current(i);
				else if(leg_motor[i].motor_mode==MOTOR_MODE_POS)
					CAN_set_q(i);		
			}
			
			if(leg_motor[i].motor_mode!=leg_motor[i].motor_mode_reg&&1)
			{
				state_[i]=0;
				leg_motor[i].motor_en=0;
			}
		break;
		default:
			state_[i]=0;
		break;
	}
		
	leg_motor[i].motor_en_reg=leg_motor[i].motor_en;
	leg_motor[i].motor_mode_reg=leg_motor[i].motor_mode;
	}
}




