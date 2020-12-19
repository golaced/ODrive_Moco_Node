#include "include.h" 
#include "can.h"
#include "delay.h"
#include "time.h"
#include "mymath.h"
float tar_sita[4][2]={0,0};
float q_err[4][2]={0};
float sita_d[4][2]={0};
float tao_q_i[4][2]={0};

float pos_taod[4][2]={0,0};
float kp=0.003;
float ki=0.001;
float kd=0.00001;
void pos_control(float dt)
{
    int i=0;
		static float q_reg[4][2];
    for(i=0;i<4;i++){
			q_err[i][0]=limitw(To_180_degreesw(tar_sita[i][0]-leg_motor[i].q_now[0]),-66,66);
			q_err[i][1]=limitw(To_180_degreesw(tar_sita[i][1]-leg_motor[i].q_now[1]),-66,66);
			
			//积分控制
			if(leg_motor[i].connect&&leg_motor[i].connect_motor[i]&&leg_motor[i].ready[i]&&leg_motor[i].motor_en){
				tao_q_i[i][0]+=q_err[i][0]*ki*(dt+0.000001);
				tao_q_i[i][1]+=q_err[i][1]*ki*(dt+0.000001);
			}else{
				tao_q_i[i][0]=0;
				tao_q_i[i][1]=0;
			}
			tao_q_i[i][0]=limitw(tao_q_i[i][0],-leg_motor[i].max_t[0]*0.3,leg_motor[i].max_t[0]*0.3);
			tao_q_i[i][1]=limitw(tao_q_i[i][1],-leg_motor[i].max_t[1]*0.3,leg_motor[i].max_t[1]*0.3);
 
			sita_d[i][0]=leg_motor[i].q_now[0]-q_reg[i][0]/(dt+0.000001);
			sita_d[i][1]=leg_motor[i].q_now[1]-q_reg[i][1]/(dt+0.000001);
			
			pos_taod[i][0]= q_err[i][0]*kp-
			limitw(sita_d[i][0],-2000,2000)*kd+tao_q_i[i][0];

			pos_taod[i][1]= q_err[i][1]*kp-
			limitw(sita_d[i][1],-2000,2000)*kd+tao_q_i[i][1];
			
			pos_taod[i][0]=limitw(pos_taod[i][0],-leg_motor[i].max_t[0],leg_motor[i].max_t[0]);
			pos_taod[i][1]=limitw(pos_taod[i][1],-leg_motor[i].max_t[1],leg_motor[i].max_t[1]);
			
			leg_motor[i].set_t[0]=pos_taod[i][0];
			leg_motor[i].set_t[1]=pos_taod[i][1];
			
			q_reg[i][0]=leg_motor[i].q_now[0]*leg_motor[i].motor_en*leg_motor[i].connect*leg_motor[i].connect_motor[0];
			q_reg[i][1]=leg_motor[i].q_now[1]*leg_motor[i].motor_en*leg_motor[i].connect*leg_motor[i].connect_motor[1];
    }
}

char reset_q=0;//标0 设定2后再设定0  
char motor_en=0;
int main(void)
{
	char i;
	float dt;
	NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
	SysTick_Configuration(); 
	//--------------------CAN----------------------------1Mbps
	//Moco Drive
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps    
	Delay_ms(100);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps  
	Delay_ms(100);
	CAN_motor_init();
	while(1){
		dt = Get_Cycle_T(0); 
		leg_motor[0].reset_q=leg_motor[1].reset_q=leg_motor[2].reset_q=leg_motor[3].reset_q=reset_q;
		leg_motor[0].motor_en=leg_motor[1].motor_en=leg_motor[2].motor_en=leg_motor[3].motor_en=motor_en;
		
		pos_control(dt);
		CAN_motor_sm(dt);	
		delay_ms(5);
	}
 	return (1);
}


