#include "encoder.h"
#include <FOC.h>
#include "main.h"
#include "stdlib.h"

/*速度解算相关*/
#define V_WINDOW_N 2 // 滑窗滤波窗口大小
#define V_WINDOW_N_INNER 10 // 滑窗滤波窗口大小
float M_velVec[V_WINDOW_N] = {0}; // M法速度解算滑窗滤波速度数组
float M_velVec_Inner[V_WINDOW_N_INNER] = {0}; // M法速度解算滑窗滤波速度数组

volatile float encoder_theta;
Encoder_t encoder_g;
Encoder_t encoder2_g;
Torque_t torque_g;

Encoder_t *p_encoder_g = &encoder_g;
Encoder_t *p_encoder2_g = &encoder2_g;
Torque_t *p_torque_g = &torque_g;

uint8_t vel_loop_flag = 0;
uint8_t pos_loop_flag = 0;
float P_filter_k = 1.0f;//位置反馈一阶低通滤波系数
float v_filter_k = 0.1f;//速度反馈一阶低通滤波系数


/*T_Method*/
float speed_w = 0;
int32_t delta_time_CNT_T = 0;
int32_t delta_encoder_cnt_T = 0;
/*M_Method*/
int32_t delta_time_CNT_M = 0;
volatile int32_t delta_encoder_cnt_M = 0;
volatile int32_t delta_encoderInner_cnt_M = 0;

float test_H1 = 0;
float test_H2 = 0;
float test_H3 = 0;
float test_H4 = 0;
uint8_t vel_zero_cnt = 0;
void Encoder_Init(void)//初始化
{
	p_encoder_g->cpr = 16777216;
	p_encoder_g->cpr_div_two = (p_encoder_g->cpr) >> 1;//分辨率除2
	p_encoder_g->one_div_cpr = 1.0f / (float)p_encoder_g->cpr;//1除分辨率
	p_encoder_g->mech_pos = 0;
	p_encoder_g->mech_pos_several_times_before = 0;
	p_encoder_g->rotations = 0;
	p_encoder_g->last_mech_pos = 0;
	p_encoder_g->elec_pos = 0.0f;
	p_encoder_g->elec_vel = 0.0f;
	p_encoder_g->elec_offset = 1.719367;//1.090150
	p_encoder_g->mech_offset = 336.764;
	p_encoder_g->mech_vel = 0.0f;
	p_encoder_g->pos_abs = 0.0f;
	p_encoder_g->cali_num = 16;//分辨率一圈位数
	p_encoder_g->cali_bit = 17 - 4;//位数
	p_encoder_g->cali_finish = 0;

	p_encoder2_g->cpr = 16777216;
	p_encoder2_g->cpr_div_two = (p_encoder2_g->cpr) >> 1;
	p_encoder2_g->one_div_cpr = 1.0f / (float)p_encoder2_g->cpr;
	p_encoder2_g->mech_pos = 0;
	p_encoder2_g->rotations = 0;
	p_encoder2_g->last_mech_pos = 0;
	p_encoder2_g->elec_pos = 0.0f;
	p_encoder2_g->elec_vel = 0.0f;
	p_encoder2_g->elec_offset = 0.0f;
	p_encoder2_g->mech_vel = 0.0f;
	p_encoder2_g->pos_abs = 0.0f;
	p_encoder2_g->cali_num = 0;
	p_encoder2_g->cali_bit = 17 - 4;
	p_encoder2_g->cali_finish = 0;

	p_torque_g->deform = 0.0f;
	p_torque_g->current2torque = 0.0f;
	p_torque_g->deform2torque = 0.0f;
	p_torque_g->relative_encoder_pos = 0.0f;
	p_torque_g->relative_encoder2_pos = 0.0f;
	p_torque_g->encoder_order_ratio = -1.0f;
	p_torque_g->reduction_ratio = 50.0f;
	p_torque_g->torque_current_ratio = 3.3925f;
	p_torque_g->torque_deform_ratio = 369.729;

	p_encoder_g->mech_pos = angleOutter;
	p_encoder_g->mech_pos_several_times_before = p_encoder_g->mech_pos;
	p_encoder_g->last_mech_pos = p_encoder_g->mech_pos;

	p_encoder2_g->mech_pos = angleInner;
	p_encoder2_g->mech_pos_several_times_before = p_encoder2_g->mech_pos;
	p_encoder2_g->last_mech_pos = p_encoder2_g->mech_pos;
}

void EncoderSample(void)//电机电角度标定后，位置加上偏移的角度
{
	p_encoder_g->last_mech_pos = p_encoder_g->mech_pos;
	p_encoder_g->mech_pos = angleOutter;
	p_encoder_g->mech_abs = PI_TIMES_2 * (float)p_encoder_g->mech_pos * p_encoder_g->one_div_cpr;
	p_encoder_g->delta_mech_pos = p_encoder_g->mech_pos - p_encoder_g->last_mech_pos;
	// judge rotations
	if (p_encoder_g->delta_mech_pos > p_encoder_g->cpr_div_two)//正常情况下差值很小，除非是从0到cpr或者从cpr到0的跳变会导致差值大于cpr_div_two
	{
		p_encoder_g->rotations -= 1;
		p_encoder_g->delta_mech_pos -= p_encoder_g->cpr;
	}
	else if (p_encoder_g->delta_mech_pos < -p_encoder_g->cpr_div_two)
	{
		p_encoder_g->rotations += 1;
		p_encoder_g->delta_mech_pos += p_encoder_g->cpr;
	}
	// 计算上电后的绝对位置（含电子圈数）
	p_encoder_g->pos_abs = PI_TIMES_2 * ((float)p_encoder_g->rotations + (float)p_encoder_g->mech_pos * p_encoder_g->one_div_cpr);

	// 计算电角度（减去电角度偏移）
	p_encoder_g->elec_pos = ((float)p_encoder_g->mech_pos * p_encoder_g->one_div_cpr)*(float)p_motor_g->pole_pairs*PI_TIMES_2 - p_encoder_g->elec_offset;
	p_encoder_g->elec_pos = fmodf(p_encoder_g->elec_pos, PI_TIMES_2);
	if (p_encoder_g->elec_pos < 0) p_encoder_g->elec_pos += PI_TIMES_2;
	
	p_encoder2_g->last_mech_pos = p_encoder2_g->mech_pos;
	p_encoder2_g->mech_pos = angleInner;
	p_encoder2_g->mech_abs = PI_TIMES_2 * (float)p_encoder2_g->mech_pos * p_encoder2_g->one_div_cpr;
	p_encoder2_g->delta_mech_pos = p_encoder2_g->mech_pos - p_encoder2_g->last_mech_pos;
	// judge rotations
	if (p_encoder2_g->delta_mech_pos > p_encoder2_g->cpr_div_two)//正常情况下差值很小，除非是从0到cpr或者从cpr到0的跳变会导致差值大于cpr_div_two
	{
		p_encoder2_g->rotations -= 1;
		p_encoder2_g->delta_mech_pos -= p_encoder2_g->cpr;
	}
	else if (p_encoder2_g->delta_mech_pos < -p_encoder2_g->cpr_div_two)
	{
		p_encoder2_g->rotations += 1;
		p_encoder2_g->delta_mech_pos += p_encoder2_g->cpr;
	}
	// 计算上电后的绝对位置（含电子圈数）
	p_encoder2_g->pos_abs = PI_TIMES_2 * ((float)p_encoder2_g->rotations + (float)p_encoder2_g->mech_pos * p_encoder2_g->one_div_cpr);
	// 从源头减去机械零位偏移，使 pos_abs 直接表示相对于零点的位置
	p_encoder2_g->pos_abs -= p_encoder2_g->mech_offset;

	static uint8_t vel_calc_count = 0;//vel_calc_count为静态局部变量
	vel_calc_count++;
	if (vel_calc_count >= VEL_CALC_PERIOD)
	{
		vel_calc_count = 0;
		vel_loop_flag = 1;
		prvCalcVelocity();//计算速度
	}
	
	static uint8_t pos_calc_count = 0;
	pos_calc_count++;
	if (pos_calc_count >= POS_CALC_PERIOD)
	{
		pos_calc_count = 0;
		pos_loop_flag = 1;
	}	
}
///**
//  * @brief  计算电机机械及电速度.
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
static void prvCalcVelocity(void)
{
	delta_encoder_cnt_M = p_encoder_g->mech_pos-p_encoder_g->mech_pos_several_times_before;
	if (delta_encoder_cnt_M > p_encoder_g->cpr_div_two) delta_encoder_cnt_M -= p_encoder_g->cpr;
	else if (delta_encoder_cnt_M < -p_encoder_g->cpr_div_two) delta_encoder_cnt_M += p_encoder_g->cpr;
	
	p_encoder_g->mech_vel = delta_encoder_cnt_M * PI_TIMES_2 * PWM_FREQUENCY_DEFAULT / p_encoder_g->cpr / (float)VEL_CALC_PERIOD;

	/*M法滑窗滤波*/
	float sum_M = p_encoder_g->mech_vel;
	for (uint16_t i = 1; i < V_WINDOW_N; i++)
	{
			M_velVec[V_WINDOW_N - i] = M_velVec[V_WINDOW_N - i - 1];
			sum_M += M_velVec[V_WINDOW_N - i];
	}
	M_velVec[0] = p_encoder_g->mech_vel;
	p_encoder_g->mech_vel =  sum_M/(float)V_WINDOW_N;
	if ((p_encoder_g->mech_vel == 0) && (FSMstate == MOTOR_MODE)) vel_zero_cnt++;

	// 一阶低通滤波
	p_encoder_g->mech_vel_filt += v_filter_k * (p_encoder_g->mech_vel - p_encoder_g->mech_vel_filt);
	p_encoder_g->elec_vel = p_encoder_g->mech_vel * (float)p_motor_g->pole_pairs;
	p_encoder_g->mech_pos_several_times_before = p_encoder_g->mech_pos;
	
	
	delta_encoderInner_cnt_M = p_encoder2_g->mech_pos-p_encoder2_g->mech_pos_several_times_before;
	if (delta_encoderInner_cnt_M > p_encoder2_g->cpr_div_two) delta_encoderInner_cnt_M -= p_encoder2_g->cpr;
	else if (delta_encoderInner_cnt_M < -p_encoder2_g->cpr_div_two) delta_encoderInner_cnt_M += p_encoder2_g->cpr;
	
	p_encoder2_g->mech_vel = delta_encoderInner_cnt_M * PI_TIMES_2 * PWM_FREQUENCY_DEFAULT / p_encoder2_g->cpr / (float)VEL_CALC_PERIOD;

	/*M法滑窗滤波*/
	float sum_M_Inner = p_encoder2_g->mech_vel;
	for (uint16_t j = 1; j < V_WINDOW_N_INNER; j++)
	{
			M_velVec_Inner[V_WINDOW_N_INNER - j] = M_velVec_Inner[V_WINDOW_N_INNER - j - 1];
			sum_M_Inner += M_velVec_Inner[V_WINDOW_N_INNER - j];
	}
	M_velVec_Inner[0] = p_encoder2_g->mech_vel;
	p_encoder2_g->mech_vel =  sum_M_Inner/(float)V_WINDOW_N_INNER;

	// 一阶低通滤波
	p_encoder2_g->mech_vel_filt += v_filter_k * (p_encoder2_g->mech_vel - p_encoder2_g->mech_vel_filt);
	p_encoder2_g->mech_pos_several_times_before = p_encoder2_g->mech_pos;
}
///**
//  * @brief  brief.
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
//static void prvCalcTorque(void)//计算力矩
//{
//	//current to torque
//	p_torque_g->current2torque = p_torque_g->torque_current_ratio * p_motor_g->Q_axis_current;
//	//deform to torque
//	p_torque_g->deform = p_encoder2_g->pos_abs - ((p_encoder_g->pos_abs - p_torque_g->relative_encoder_pos)/p_torque_g->reduction_ratio*p_torque_g->encoder_order_ratio + p_torque_g->relative_encoder2_pos);
//	p_torque_g->deform2torque = p_motor_g->phase_order * p_torque_g->torque_deform_ratio * p_torque_g->deform;
//}

///**
//  * @brief  brief.
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
//static void prvSetDAxisAngle(float theta)//旋转一定的角度
//{
////	encoder_theta = Mod(theta,0,PI_TIMES_2);
////	osDelay(10);
//	theta = Mod(theta,0,PI_TIMES_2);//限制theta在0到2Pi
//	//Foc.ApplyVoltDQToSVPWM(p_motor_g->cali_voltage, 0.0f, theta);
//	Foc.ApplyVoltDQToSVPWM(1.5f, 0.0f, theta);//d轴电压1.0V q轴电压0V

//}


///* ------------------------------ Manager Declaration ------------------------------ */
//const EncoderManager_typedef Encoder =
//{
//  Init,
//  Sample,
//  Calibrate
//};
