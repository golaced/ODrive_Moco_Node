#include "can.h"
#include "delay.h"
#include "include.h"
#include "my_math.h"
#include "math.h"

_LEG_MOTOR leg_motor[4];
//CAN初始化
//tsjw:重新同步跳跃时间单元. @ref CAN_synchronisation_jump_width   范围: ; CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   @ref CAN_time_quantum_in_bit_segment_2 范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   @refCAN_time_quantum_in_bit_segment_1  范围: ;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//波特率=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
//mode: @ref CAN_operating_mode 范围：CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((1+6+7)*6)=500Kbps   0 7 12 4 
//返回值:0,初始化OK;
//    其他,初始化失败;
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
		leg_motor[i].motor_mode=MOTOR_MODE_T;//模式选择
		
		reset_current_cmd(i);
		
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=1.8;//最大力矩
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=12;//最大电流
		
		leg_motor[0].q_reset[0]=0;//电机零偏复位角度  +-
		leg_motor[0].q_reset[1]=270+8;//                0~360

		leg_motor[1].q_reset[0]=-90;//电机零偏复位角度
		leg_motor[1].q_reset[1]=180+18;

		leg_motor[2].q_reset[0]=0;//电机零偏复位角度
		leg_motor[2].q_reset[1]=270+8;

		leg_motor[3].q_reset[0]=-90;//电机零偏复位角度
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
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	  
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
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


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;//0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
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
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN2_RX0_INT_ENABLE
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
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

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
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

	canbuft1[4]=leg_motor[id].max_i[0];//最大电流
	canbuft1[5]=leg_motor[id].motor_en;//电机使能
	canbuft1[6]=leg_motor[id].reset_q;//电机标零
	canbuft1[7]=reset_err_flag;//复位故障
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




