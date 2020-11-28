#ifndef __CAN1_H
#define __CAN1_H	 
#include "sys.h"	 
#include "stm32f4xx_can.h"
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1	//0,不使能;1,使能.								    
				 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft1[8];
extern u8 canbufr1[8];
extern uint32_t can1_rx_id;
extern float cnt_rst1;
extern int can1_rx_cnt;
void data_can_anal_master(u8 rx_data[8]);

//CAN1接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
								 							 				    
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN2_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft2[8];
extern u8 canbufr2[8];
extern uint32_t can2_rx_id;
extern float cnt_rst2;
extern int can2_rx_cnt;

#define MOTOR_MODE_POS 0
#define MOTOR_MODE_T	 1
#define MOTOR_MODE_CURRENT	2
typedef struct 
{
	char connect,connect_motor[2];
	char ready[2];
	int loss_cnt;
	char reset_q;
	char motor_en,motor_en_reg;
	char motor_mode,motor_mode_reg;
	char err_flag[2];
	float bat_v[2];
	float temp[2];
	float q_now[3],qd_now[3],qdd_now[3];
	float q_set[3],qd_set[3],qdd_set[3];
	float f_now[3];
	float f_set[3];
	float t_now[3];
	float q_reset[3];
	float set_t[3];
	float set_i[3];
	float max_t[3];
	float max_i[3];
	char can_bus_id;
}_LEG_MOTOR;
extern _LEG_MOTOR leg_motor[4];

void CAN_set_torque(char id);
void CAN_reset_q(char id);
void CAN_motor_sm(float dt);
void CAN_motor_init(void);
void CAN_set_zero_off(char id);
extern char reset_err_flag;
void reset_current_cmd(char id);
#endif

















