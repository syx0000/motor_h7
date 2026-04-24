#include "uart4_VOFA+.h"
#include "main.h"
extern UART_HandleTypeDef huart4;

uint8_t VOFA_dma_tx_buf[CH_COUNT*4+4];
float VOFA_data[CH_COUNT] = {0.0f};
unsigned short VOFA_On = 1;       //启用VOFA观测
extern ControllerStruct controller;

//使用联合体将浮点数转换为4个u8类型数据
union
{
    uint8_t a[4];   //在单片机中浮点存储为小端模式，高字节在高位，低字节在低位。
    float b;
}temp;
//装载数据
void LoadData()
{	
//	VOFA_data[0] = p_motor_g->vbus;
//	VOFA_data[1] = p_motor_g->phase_a_current;
//	VOFA_data[2] = p_motor_g->phase_b_current;
//	VOFA_data[3] = p_motor_g->phase_c_current;
//	VOFA_data[4] = (float)p_encoder_g->mech_pos;
//	VOFA_data[5] = p_motor_g->phase_a_current_actual;
//	VOFA_data[6] = p_encoder_g->mech_vel;
//	VOFA_data[7] = ISR_time_us;
	
//	VOFA_data[0] = p_motor_g->vbus;
//	VOFA_data[1] = p_motor_g->phase_a_current;
//	VOFA_data[2] = p_motor_g->phase_b_current;
//	VOFA_data[3] = p_motor_g->phase_c_current;
//	VOFA_data[4] = (float)p_encoder_g->mech_pos;
//	VOFA_data[5] = p_motor_g->phase_a_current_actual;
//	VOFA_data[6] = p_encoder_g->mech_vel;
//	VOFA_data[7] = ISR_time_us;
//	VOFA_data[8] = p_encoder_g->pos_abs;
//	VOFA_data[9] = p_position_loop_g->target;
//	VOFA_data[10] = p_position_loop_g->err;
	//闭环测试//
	VOFA_data[0] = p_position_loop_g->target;
	VOFA_data[1] = p_encoder2_g->pos_abs;
	VOFA_data[2] = Motor_P;//p_position_loop_g->err;
	VOFA_data[3] = p_motor_g->controlMode;//p_motor_g->i_d_ref;
	VOFA_data[4] = FSMstate;//p_motor_g->i_q_ref;
	VOFA_data[5] = p_motor_g->D_axis_current;
	VOFA_data[6] = p_motor_g->Q_axis_current;
	VOFA_data[7] = p_velocity_loop_g->target;
	VOFA_data[8] = p_encoder_g->mech_vel;
	VOFA_data[9] = p_motor_g->vbus;
	VOFA_data[10] = p_motor_g->phase_a_current;
	VOFA_data[11] = p_motor_g->phase_b_current;
	VOFA_data[12] = p_motor_g->phase_c_current;
	VOFA_data[13] = p_motor_g->phase_a_current_actual;
	VOFA_data[14] = (float)p_encoder_g->mech_pos;
	VOFA_data[15] = ISR_time_us;
	VOFA_data[16] = TEMPERATURE_MOTOR;//controller.v_d;
	VOFA_data[17] = controller.v_q;
	VOFA_data[18] = (float)p_encoder2_g->mech_pos;
	
	/*扫频测试*/
//	VOFA_data[0] = p_motor_g->i_d_ref;
//	VOFA_data[1] = p_motor_g->D_axis_current;
////	VOFA_data[2] = (float)p_encoder_g->mech_pos;


//	VOFA_data[2] = p_motor_g->Q_axis_current;
//	VOFA_data[3] = p_motor_g->vbus;
//	VOFA_data[4] = p_motor_g->phase_a_current;
//	VOFA_data[5] = p_motor_g->phase_b_current;
//	VOFA_data[6] = p_motor_g->phase_c_current;
//	VOFA_data[7] = p_motor_g->phase_a_current_actual;
//	VOFA_data[8] = ISR_time_us;
//	VOFA_data[9] = controller.v_d;
//	VOFA_data[10] = controller.v_q;
	

//	VOFA_data[0] = p_motor_g->phase_a_current;
//	VOFA_data[1] = p_motor_g->phase_a_current_filt2;
//	VOFA_data[2] = p_motor_g->phase_b_current;
//	VOFA_data[3] = p_motor_g->phase_b_current_filt1;
//	VOFA_data[4] = p_motor_g->phase_b_current_filt2;
//	VOFA_data[5] = p_motor_g->phase_c_current;
//	VOFA_data[6] = p_motor_g->phase_c_current_filt1;
//	VOFA_data[7] = p_motor_g->phase_c_current_filt2;
//	VOFA_data[8] = p_motor_g->phase_a_current_actual;
//	VOFA_data[9] = p_motor_g->phase_a_current_filt;
//	VOFA_data[10] = p_motor_g->phase_b_current_filt;
//	VOFA_data[11] = p_motor_g->phase_c_current_filt;
	
//	VOFA_data[0] = (float)p_encoder_g->mech_pos;
//	VOFA_data[1] = p_encoder_g->mech_vel;
	
	for(uint16_t i = 0;i<CH_COUNT;i++)
	{
		temp.b = VOFA_data[i];
		for(uint16_t j = 0;j<4;j++) VOFA_dma_tx_buf[4*i+j]=temp.a[j];
	}
	
	/*帧尾*/
	VOFA_dma_tx_buf[4*CH_COUNT]   = 0x00;
	VOFA_dma_tx_buf[4*CH_COUNT+1] = 0x00;
	VOFA_dma_tx_buf[4*CH_COUNT+2] = 0x80;
	VOFA_dma_tx_buf[4*CH_COUNT+3] = 0x7f;
}
