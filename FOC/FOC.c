#include <FOC.h>
#include <main.h>
#include <math.h>
#include <stdint.h>

volatile char cmd_val[8] = {0};
volatile char cmd_id = 0;
volatile char char_count = 0;


extern volatile uint16_t state_change;
extern ControllerStruct controller;
/*位置插补*/
extern Trajectory_t trajectory_g;
extern Trajectory_t *p_trajectory_g;
volatile char is_PPMode = 0;//默认直通

volatile float I_BW_set;
volatile float I_SWOver_set;
volatile float Motor_Iq_set;
volatile float Motor_W_set;
volatile float Motor_P_set;
volatile float Velocity_P_set;
volatile float Velocity_I_set;
volatile float Position_P_set;
volatile float Position_I_set;
volatile float Current_P_set;
volatile float Current_I_set;
volatile float FOC_velAccDec_set;

volatile uint16_t FSMstate = REST_MODE;
uint8_t caliOn_flag = 0;
// 控制位定义
#define DEM_CR_TRCENA       (1 << 24) // 启用DWT和ITM等调试模块
#define DWT_CR_CYCCNTENA    (1 << 0)  // 启用CYCCNT计数器
#define DWT_LAR_KEY         0xC5ACCE55 // Cortex-M7 DWT访问解锁密钥[3](@ref)
uint32_t ISR_start, ISR_end;
float ISR_time_us = 0;
/*DMA直接搬运到数组*/
uint16_t ADC_Temp_Value[2];
uint16_t ADC_Cur_vbus_Value[ADC2_CHANNELS*ADC2_CHANNELS_WINDOW];

const float sample_time = 400.0f; // remain some time for ADC sample
uint8_t svpwm_on = 0;
#define OVERMODULATION  1.15f       // 1.0 = no overmodulation

ControllerStruct controller;

uint16_t Ta = 0, Tb = 0, Tc = 0;
 
static void SVPWM(float alpha, float beta)//SVPWM算法 具体可见https://blog.csdn.net/Alex497259/article/details/125441988
{
	static const float SQRT3_TIMES_TimerPeriod = SQRT3 * PWM_PERIOD_DEFAULT;
	static const float TimerPeriod_MINUS_sample_time = PWM_PERIOD_DEFAULT - ADC_sample_time;
	uint8_t JudgeSextant = 0; // a variable related to sextant and not equals to sextant
	float timeX = 0.0f, timeY = 0.0f, timeZ = 0.0f;
	float Tx = 0.0f, Ty = 0.0f, Tsum = 0.0f; // book table 2-5
	//uint16_t Ta = 0, Tb = 0, Tc = 0;
	//Avoid repeated calculation and reduce calculation time
	float fast_calculate_1 = SQRT3_TIMES_TimerPeriod / p_motor_g->vbus;
	float fast_calculate_2_1 = SQRT3_BY_2 * alpha;
	float fast_calculate_2_2 = 0.5f * beta;
	float fast_calculate_2 = fast_calculate_2_1 + fast_calculate_2_2;
	float fast_calculate_3 = -fast_calculate_2_1 + fast_calculate_2_2;

	JudgeSextant = (beta > 0.0f) + ((fast_calculate_3 < 0.0f) << 1) + ((fast_calculate_2 < 0.0f) << 2);

	timeX = fast_calculate_1 * beta;
	timeY = fast_calculate_1 * fast_calculate_2;
	timeZ = fast_calculate_1 * fast_calculate_3;
	
	//judge the sextant and calculate the time x,y and z
	switch (JudgeSextant)
	{
		case 3: // Sextant = 1;
			Tx = -timeZ;
			Ty = timeX;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			break;
		case 1: // Sextant = 2;
			Tx = timeZ;
			Ty = timeY;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			break;
		case 5: // Sextant = 3;
			Tx = timeX;
			Ty = -timeY;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			break;
		case 4: // Sextant = 4;
			Tx = -timeX;
			Ty = timeZ;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			break;
		case 6: // Sextant = 5;
			Tx = -timeY;
			Ty = -timeZ;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			break;
		case 2: // Sextant = 6;
			Tx = timeY;
			Ty = -timeX;
			Tsum = Tx + Ty;
			if (Tsum > TimerPeriod_MINUS_sample_time)
			{
				Tx = Tx * TimerPeriod_MINUS_sample_time / Tsum;
				Ty = TimerPeriod_MINUS_sample_time - Tx;
			}
			Ta = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
			Tb = (uint16_t)(PWM_PERIOD_DEFAULT + Tx + Ty) >> 2;
			Tc = (uint16_t)(PWM_PERIOD_DEFAULT + Tx - Ty) >> 2;
			break;		
	}
	//calculate the tim1's turning time
	__asm volatile("cpsid i");//关闭中断的汇编指令（C语言中嵌入汇编）
	if (p_motor_g->phase_order == POSITIVE_PHASE_ORDER)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tc);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tb);
	}
	__asm volatile("cpsie i");//开启IRQ中断
}

/**
 * @brief  turn volt_d, volt_q and theta to svpwm
 * @note   InverseParkTransform and SVPWM
 * @param  voltage_d and vlotage_q and the ele_theta
 * @retval None
 */
void ApplyVoltDQToSVPWM(float volt_d, float volt_q, float theta)
{
	float volt_alpha, volt_beta;
	__asm volatile("cpsid i");

	InverseParkTransform(&volt_d, &volt_q, theta, &volt_alpha, &volt_beta);
	SVPWM(volt_alpha, volt_beta);
	__asm volatile("cpsie i");
}
/*********************************************坐标变换*********************************************/
void ClarkTransform(volatile float *current_a, volatile float *current_b, volatile float *current_c, volatile float *current_alpha, volatile float *current_beta)
{	//abc→αβClark等幅值变换 需要在计算磁场分量结束后 乘上2/3的系数
	*current_alpha = _2_BY_3 * *current_a - _1_BY_3 * (*current_b + *current_c);
	*current_beta = SQRT3_BY_3 * (*current_b - *current_c);
}

void InverseClarkTransform(volatile float *current_alpha, volatile float *current_beta, volatile float *current_a, volatile float *current_b, volatile float *current_c)//反Clark变换
{	//αβ→abc
	*current_a = *current_alpha;
	float fast_1 = -0.5f * (*current_alpha);
	float fast_2 = SQRT3_BY_2 * (*current_beta);
	*current_b = fast_1 + fast_2;
	*current_c = fast_1 - fast_2;
}

void ParkTransform(volatile float *current_alpha, volatile float *current_beta, volatile float theta, volatile float *current_d, volatile float *current_q)//Park变换
{	//αβ→dq
	theta = PrvMod2PI(theta);
	float cos_theta = arm_cos_f32(theta);
	float sin_theta = arm_sin_f32(theta);
	*current_d = cos_theta * *current_alpha + sin_theta * *current_beta;
	*current_q = -sin_theta * *current_alpha + cos_theta * *current_beta;
}

void InverseParkTransform(volatile float *current_d, volatile float *current_q, volatile float theta, volatile float *current_alpha, volatile float *current_beta)//反Park变换
{	//dq→αβ
	theta = PrvMod2PI(theta);
	float cos_theta = arm_cos_f32(theta);
	float sin_theta = arm_sin_f32(theta);
	*current_alpha = cos_theta * *current_d - sin_theta * *current_q;
	*current_beta = sin_theta * *current_d + cos_theta * *current_q;
}

void InitControllerParams(ControllerStruct *controller)
{
	controller->v_d  = 0;
	controller->v_q  = 0;	
	p_motor_g->i_d_ref = 0;
	p_motor_g->i_q_ref = 0;
	controller->d_int = 0;
	controller->q_int = 0;
	/*手调PI参数初始化*/
	controller->ki_d = Current_I;
	controller->ki_q = Current_I;
	controller->k_d = Current_P;
	controller->k_q = Current_P;
	/*经验公式计算PI参数初始化*/
	//controller->ki_d = I_BW*2.0f*PI*p_motor_g->phase_resistance*(1/PWM_FREQUENCY_DEFAULT);//积分参数：电流环带宽*相电阻*电流环周期
	//controller->ki_q = I_BW*2.0f*PI*p_motor_g->phase_resistance*(1/PWM_FREQUENCY_DEFAULT);
	//controller->k_d = I_BW*2.0f*PI*p_motor_g->phase_inductance;//比例参数：电流环带宽*相电感
	//controller->k_q = I_BW*2.0f*PI*p_motor_g->phase_inductance;
	/*PD控制参数*/
	controller->kp = 0;
	controller->kd = 0;
}

void Homing(void)//回零
{
	p_position_loop_g->feedforward = 0.0f;
	Pid.DoPidCalc(p_position_loop_g,p_encoder_g->mech_abs);//输出端HALL编码器绝对位置(单位：rad/GR) *GR是为了位置环PI参数通用
  
	Pid.SetTarget(p_velocity_loop_g, p_position_loop_g->output);
}

void PositionLoop(void)//位置环 PI运算得到速度给定
{
	p_position_loop_g->feedforward = 0.0f;
//	Pid.DoPidCalc(p_position_loop_g, p_encoder_g->pos_abs);//（含上电后转动圈数）电机绝对机械位置
	Pid.DoPidCalc(p_position_loop_g, p_encoder2_g->pos_abs);//（含上电后转动圈数）电机绝对机械位置
	Pid.SetTarget(p_velocity_loop_g, p_position_loop_g->output);
}

void VelocityLoop(void)//速度环 PI运算得到q轴电流给定 p_motor_g->i_d_ref和p_motor_g->i_q_ref
{
	/*速度斜坡规划*/
	if (p_motor_g->controlMode == FOC_VELOCITY_LOOP)//速度闭环添加规划
	{
		float max_step_size = fabsf(FOC_velAccDec * VEL_CALC_PERIOD / PWM_FREQUENCY_DEFAULT);
		float full_step = p_velocity_loop_g->targetend - p_velocity_loop_g->target;
		float step = CLAMP(full_step, -max_step_size, max_step_size);
		p_velocity_loop_g->target += step;
	}
	/*位置闭环下摆臂实验中增加前馈控制，提高速度环响应速度*/
//	p_velocity_loop_g->feedforward_ratio = 1;
//	if (p_motor_g->controlMode == FOC_POSITION_LOOP) p_velocity_loop_g->feedforward = p_encoder2_g->compSin*1.75f;//重力补偿
	/*速度闭环下摆臂实验中增加前馈控制，提高速度环响应速度*/
//	p_velocity_loop_g->feedforward_ratio = 1;
//	if (p_motor_g->controlMode == FOC_VELOCITY_LOOP) p_velocity_loop_g->feedforward = p_encoder_g->compSin*1.75f;//重力补偿 42.236,37.14 (43.48,35.805)
//	float FF;
//	FF = -Position_P*arm_cos_f32(encoder1_raw/16384.0f*4*PI-PI/7.0f);
//	p_velocity_loop_g->feedforward_ratio = 1;
//	if (p_motor_g->controlMode == FOC_VELOCITY_LOOP) p_velocity_loop_g->feedforward = FF;

	
//	Pid.DoPidCalc(p_velocity_loop_g, vel_estimate_);//PLL:vel_estimate_(电机端机械转速 rad/s) 发散后vel_estimate_为一个很大的值
	Pid.DoPidCalc(p_velocity_loop_g, p_encoder_g->mech_vel);//滑窗滤波：p_encoder_g->mech_vel/10.0f(输出端机械转速 rad/s)
//	Pid.DoPidCalc(p_velocity_loop_g, speed_w);//M/T法解算转速
	p_motor_g->i_q_ref = p_velocity_loop_g->output;
//	Pid.SetTarget(p_Q_current_loop_g, p_velocity_loop_g->output);
	p_motor_g->i_d_ref = 0;
//	Pid.SetTarget(p_D_current_loop_g, 0.0f);
}

void CurrentLoop()//电流环 CLARK和PARK变换+PI+SVPWM
{
//	static const float voltage_lim_ratio = SQRT3 * PWM_PERIOD_DEFAULT / (PWM_PERIOD_DEFAULT - sample_time);//给电流采样时间！！！
	ClarkTransform(&p_motor_g->phase_a_current, &p_motor_g->phase_b_current, &p_motor_g->phase_c_current, &p_motor_g->alpha_axis_current, &p_motor_g->beta_axis_current);
	float sin_theta, cos_theta;
	sin_theta = arm_sin_f32(p_encoder_g->elec_pos);
	cos_theta = arm_cos_f32(p_encoder_g->elec_pos);
	/*计算dq轴电流*/
	p_motor_g->D_axis_current = cos_theta * p_motor_g->alpha_axis_current + sin_theta * p_motor_g->beta_axis_current;
	p_motor_g->Q_axis_current = -sin_theta * p_motor_g->alpha_axis_current + cos_theta * p_motor_g->beta_axis_current;
//	DQ First order low pass filter
	p_motor_g->Q_axis_current_filt = (0.4f * p_motor_g->Q_axis_current_filt) + (0.6f * p_motor_g->Q_axis_current);
	p_motor_g->D_axis_current_filt = (0.4f * p_motor_g->D_axis_current_filt) + (0.6f * p_motor_g->D_axis_current);
	
	//电流限制
	LimitNorm(&p_motor_g->i_d_ref, &p_motor_g->i_q_ref,p_motor_g->IMax);

	/// PI Controller ///
	float i_d_error = p_motor_g->i_d_ref - p_motor_g->D_axis_current;
	float i_q_error = p_motor_g->i_q_ref - p_motor_g->Q_axis_current;
//启动dq轴电流滤波
//	float i_d_error = p_motor_g->i_d_ref - p_motor_g->D_axis_current_filt;
//	float i_q_error = p_motor_g->i_q_ref - p_motor_g->Q_axis_current_filt;
	// Integrate Error（冻结策略：输出饱和时停止积分累积）
	float v_bus_limit = OVERMODULATION * p_motor_g->vbus;
	float d_int_new = controller.d_int + controller.ki_d * i_d_error;
	float q_int_new = controller.q_int + controller.ki_q * i_q_error;

	// D轴：仅在积分不会导致输出饱和时才更新
	if (fabsf(controller.k_d * i_d_error + d_int_new) < v_bus_limit)
		controller.d_int = d_int_new;
	// 积分限幅（防止极端情况）
	controller.d_int = fmaxf(-v_bus_limit, fminf(v_bus_limit, controller.d_int));

	// Q轴：同上
	if (fabsf(controller.k_q * i_q_error + q_int_new) < v_bus_limit)
		controller.q_int = q_int_new;
	controller.q_int = fmaxf(-v_bus_limit, fminf(v_bus_limit, controller.q_int));
	
	/*使用给定电角速度计算前馈*/
	controller.v_d_ff = -p_velocity_loop_g->target*p_motor_g->pole_pairs*p_motor_g->phase_inductance*p_motor_g->i_q_ref;
	controller.v_q_ff = p_velocity_loop_g->target*p_motor_g->pole_pairs*p_motor_g->phase_inductance*p_motor_g->i_d_ref + 0.005619f*p_velocity_loop_g->target*p_motor_g->pole_pairs;
	/*使用实际电角速度计算前馈*/
//	controller.v_d_ff = -p_encoder_g->elec_vel*p_motor_g->phase_inductance*p_motor_g->i_q_ref;
//	controller.v_q_ff = p_encoder_g->elec_vel*p_motor_g->phase_inductance*p_motor_g->i_d_ref + 0.005619f*p_velocity_loop_g->target*p_motor_g->pole_pairs;
	controller.v_d = controller.k_d*i_d_error + controller.d_int;
	controller.v_q = controller.k_q*i_q_error + controller.q_int;
	/*转速闭环前馈控制*/
//	controller.v_d += controller.v_d_ff;
//	controller.v_q += controller.v_q_ff;
	//电压限制
	LimitNorm(&controller.v_d, &controller.v_q,1.15f*p_motor_g->vbus);       // Normalize voltage vector to lie within curcle of radius v_bus
//	limit_normlization(&controller->v_d, &controller->v_q, OVERMODULATION*controller->v_bus);       // Normalize voltage vector to lie within curcle of radius v_bus

	if (svpwm_on == 1)
		SVPWM(cos_theta * controller.v_d - sin_theta * controller.v_q, sin_theta * controller.v_d + cos_theta * controller.v_q);
}

void TorqueControl(ControllerStruct *controller)
{
	controller->dtheta_mech = p_encoder2_g->mech_vel;//输出端速度（rad/s）
	controller->theta_mech = p_encoder2_g->pos_abs;//输出端位置（rad）
	
	float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + 
		controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
	
	p_motor_g->i_q_ref = torque_ref/KT_OUT;
	p_motor_g->i_d_ref = 0.0f;
}

void PD_FOC_clear(void)
{	
	/*FOC相关变量清零*/
	controller.v_d_ff = 0;
	controller.v_q_ff = 0;
	controller.i_d_filt = 0;
	controller.i_q_filt = 0;
	controller.d_int = 0;
	controller.q_int = 0;
	controller.v_d = 0;
	controller.v_q = 0;
	/*PD控制参数清零*/
	controller.kp = 0;
	controller.kd = 0;
	controller.t_ff = 0;
	controller.p_des = p_encoder2_g->pos_abs;
	controller.v_des = 0;
	if (FSMstate == MOTOR_MODE)  printf("\n\r Entering Motor Mode \n\r");
	else if (FSMstate == HOMING_MODE)  printf("\n\r Entering Homing Mode \n\r");
}

void EnablePWM(void)
{
	htim1.Instance->CCER |= 0x0555;//Enable channel output
}
void DisablePWM(void)
{
	htim1.Instance->CCER &= ~0x0555;//Disable channel output
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0xffff);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0xffff);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0xffff);
	traj_complete = 0;//以免位置模式下报故，再进入电机模式电机运行
}

/*数学运算*/
// Same as fmodf but result is positive and y must be positive
float FmodfPos(float x, float y) {
	float res = WrapPm(x, y);
	if (res < 0) res += y;
	return res;
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
float WrapPm(float x, float y) {

	float intval = nearbyintf(x / y);//用于将一个浮点数四舍五入到最接近的整数值（float）
	return x - intval * y;
}

/* Scales the lenght of vector (x, y) to be <= limit */
void LimitNorm(float *x, float *y, float limit)
{
//    float norm = sqrt(*x * *x + *y * *y);//双精度（避免使用，STM32F446无硬件FPU双精度支持）
	float norm = sqrtf(*x * *x + *y * *y);//单精度
    if (norm > limit)
    {
        *x = *x * limit/norm;
        *y = *y * limit/norm;
    }
}

/* Converts a float to an unsigned int, given range and number of bits */
uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;//10
    float offset = x_min;//-5
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/* converts unsigned int to float, given range and number of bits */  
float Uint32ToFloat(uint32_t x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
float PrvMod2PI(volatile float theta) //限制角度在0-2pi之间
{
	while (theta < 0.0f)
		theta += PI_TIMES_2;
	while (theta > PI_TIMES_2)
		theta -= PI_TIMES_2;
	return theta;
}


// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
int EncoderMod(const int dividend, const int divisor)
{
	int r = dividend % divisor;
	if (r < 0) r += divisor;
	return r;
}

float Clamp(volatile float num, volatile float low, volatile float high)
{
	float output = num;
	if (num<low) output = low;
	else if (num>high) output = high;
	return output;
}

float Mod(volatile float num, volatile float mod_range_min, volatile float mod_range_max)//将num限制在mod_range_min和mod_range_max之间（float类型数）
{
  float cycle = mod_range_max - mod_range_min;
  float output = num;

  while (output < mod_range_min)
    output += cycle;
  while (output > mod_range_max)
    output -= cycle;

  return output;
}

uint16_t UintMod(volatile uint16_t output,volatile uint16_t mod_range_min,volatile uint16_t mod_range_max)//将output限制在mod_range_min和mod_range_max之间（uint16_t类型数）
{
	uint16_t cycle = mod_range_max - mod_range_min;

  while (output < mod_range_min)
    output += cycle;
  while (output > mod_range_max)
    output -= cycle;

  return output;
}

/**
 * @brief  a function to estimate the ln(a), from https://blog.csdn.net/sduyyy/article/details/78973581
 * @param  a：the variable to be calculated
 * @param  order：Order of Taylor formula expansion
 * @retval ln(a)
 */
float IterationLn(volatile float a, int order)
{
  int k, nk;
  float x, xx, y;
  x = (a - 1) / (a + 1);
  xx = x * x;
  nk = 2 * order + 1;
  y = 1.0 / nk;
  for (k = order; k > 0; k--)
  {
    nk = nk - 2;
    y = 1.0 / nk + xx * y;
  }
  return 2.0f * x * y;
}

/**
  * @brief  change uint8_t to float
  * @note   note.
  * @param  param brief
  * @retval None
  */
float Uint2Float(volatile uint8_t *p_raw)
{
	union { uint8_t b[4]; float f; } conv;
	conv.b[0] = *p_raw;
	conv.b[1] = *(p_raw + 1);
	conv.b[2] = *(p_raw + 2);
	conv.b[3] = *(p_raw + 3);
	return conv.f;
}

bool least_square_method(float *data, uint8_t num, float *K)
{
	int32_t sum_num = num * (num + 1) / 2;
	int32_t sum_num_2 = num * (num + 1) * (2 * num + 1) / 6;
	float sum_data = 0.0f, sum_multi = 0.0f;
	for (int i = 0; i < num; i++)
	{
		sum_data += *(data + i);
		sum_multi += (*(data + i)) * (i + 1); 
	}
	float sum_1 = 0.0f, sum_2 = 0.0f, sum_3 = 0.0f;
	float ave_num = (float)(num + 1) / 2.0f, ave_data = sum_data / (float)num;
	for (int k = 0; k < num; k++)
	{
		sum_1 += (k + 1 - ave_num) * (*(data + k) - ave_data);
		sum_2 += (k + 1 - ave_num) * (k + 1 - ave_num);
		sum_3 += (*(data + k) - ave_data) * (*(data + k) - ave_data);
	}
	float r_2 = sum_1 * sum_1 / (sum_2 * sum_3);
	*K = (num * sum_multi - sum_num * sum_data) / (num * sum_num_2 - sum_num * sum_num);
	if (r_2 > 0.98f)
		return 1;
	else
	{
		printf("R = %f, least_square_method error\r\n", r_2);
		return 0;
	}
}
bool least_square_method_flux(float *xdata, float *ydata, uint8_t num, float *K)
{
	float sum_xdata = 0.0f, sum_xdata_2 = 0.0f, sum_ydata = 0.0f, sum_multi = 0.0f;
	for (int i = 0; i < num; i++)
	{
		sum_xdata += *(xdata + i);
		sum_xdata_2 += (*(xdata + i)) * (*(xdata + i));
		sum_ydata += *(ydata + i);
		sum_multi += (*(xdata + i)) * (*(ydata + i)); 
	}
	float sum_1 = 0.0f, sum_2 = 0.0f, sum_3 = 0.0f;
	float ave_xdata = sum_xdata / (float)num;
	float ave_ydata = sum_ydata / (float)num;
	for (int i = 0; i < num; i++)
	{
		sum_1 += (*(xdata + i) - ave_xdata) * (*(ydata + i) - ave_ydata);
		sum_2 += (*(xdata + i) - ave_xdata) * (*(xdata + i) - ave_xdata);
		sum_3 += (*(ydata + i) - ave_ydata) * (*(ydata + i) - ave_ydata);
	}
	float r_2 = sum_1 * sum_1 / (sum_2 * sum_3);
	*K = (num * sum_multi - sum_xdata * sum_ydata) / (num * sum_xdata_2 - sum_xdata * sum_xdata);
	if (r_2 > 0.98f)
		return 1;
	else
	{
//		printf("R = %f, least_square_method_flux error\r\n", r_2);
		return 0;
	}
}
uint8_t crcTable [256] = 
{0x00,0x97,0xB9,0x2E,0xE5,0x72,0x5C,0xCB,
0x5D,0xCA,0xE4,0x73,0xB8,0x2F,0x01,0x96,
0xBA,0x2D,0x03,0x94,0x5F,0xC8,0xE6,0x71,
0xE7,0x70,0x5E,0xC9,0x02,0x95,0xBB,0x2C,
0xE3,0x74,0x5A,0xCD,0x06,0x91,0xBF,0x28,
0xBE,0x29,0x07,0x90,0x5B,0xCC,0xE2,0x75,
0x59,0xCE,0xE0,0x77,0xBC,0x2B,0x05,0x92,
0x04,0x93,0xBD,0x2A,0xE1,0x76,0x58,0xCF,
0x51,0xC6,0xE8,0x7F,0xB4,0x23,0x0D,0x9A,
0x0C,0x9B,0xB5,0x22,0xE9,0x7E,0x50,0xC7,
0xEB,0x7C,0x52,0xC5,0x0E,0x99,0xB7,0x20,
0xB6,0x21,0x0F,0x98,0x53,0xC4,0xEA,0x7D,
0xB2,0x25,0x0B,0x9C,0x57,0xC0,0xEE,0x79,
0xEF,0x78,0x56,0xC1,0x0A,0x9D,0xB3,0x24,
0x08,0x9F,0xB1,0x26,0xED,0x7A,0x54,0xC3,
0x55,0xC2,0xEC,0x7B,0xB0,0x27,0x09,0x9E,
0xA2,0x35,0x1B,0x8C,0x47,0xD0,0xFE,0x69,
0xFF,0x68,0x46,0xD1,0x1A,0x8D,0xA3,0x34,
0x18,0x8F,0xA1,0x36,0xFD,0x6A,0x44,0xD3,
0x45,0xD2,0xFC,0x6B,0xA0,0x37,0x19,0x8E,
0x41,0xD6,0xF8,0x6F,0xA4,0x33,0x1D,0x8A,
0x1C,0x8B,0xA5,0x32,0xF9,0x6E,0x40,0xD7,
0xFB,0x6C,0x42,0xD5,0x1E,0x89,0xA7,0x30,
0xA6,0x31,0x1F,0x88,0x43,0xD4,0xFA,0x6D,
0xF3,0x64,0x4A,0xDD,0x16,0x81,0xAF,0x38,
0xAE,0x39,0x17,0x80,0x4B,0xDC,0xF2,0x65,
0x49,0xDE,0xF0,0x67,0xAC,0x3B,0x15,0x82,
0x14,0x83,0xAD,0x3A,0xF1,0x66,0x48,0xDF,
0x10,0x87,0xA9,0x3E,0xF5,0x62,0x4C,0xDB,
0x4D,0xDA,0xF4,0x63,0xA8,0x3F,0x11,0x86,
0xAA,0x3D,0x13,0x84,0x4F,0xD8,0xF6,0x61,
0xF7,0x60,0x4E,0xD9,0x12,0x85,0xAB,0x3C};

uint8_t CalcCRC(uint8_t * buffer, uint8_t length){
	uint8_t temp = *buffer++;
	while (--length){
	temp = *buffer++ ^ crcTable[temp];
	}
	return crcTable[temp];
}
void DelayUs(uint16_t nus)
{

	htim6.Instance->CNT = 0;   										// set the counter value 0
	htim6.Instance->CR1|=TIM_CR1_CEN;
//	HAL_TIM_Base_Start(&htim6);
/*wait for the counter to reach the us input in the parameter*/
	while (htim6.Instance->CNT < nus);
	htim6.Instance->CR1 &= ~TIM_CR1_CEN;       		// stop the counter
	
	// Set timer period for desired delay in microseconds
//	htim6.Instance->CNT = 0; 
//  __HAL_TIM_SET_AUTORELOAD(&htim6, nus - 1);//定时器响应时间为period*定时器频率
//	HAL_TIM_Base_Start(&htim6);//start the timer
//	//通过轮询的方式等待定时器的更新事件
//	//当定时器溢出并计数器更新时，TIM_FLAG_UPDATE标志会被置位。
//	while (__HAL_TIM_GET_FLAG(&htim6,TIM_FLAG_UPDATE)==RESET);
//	__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);//清楚更新标志位
//		HAL_TIM_Base_Stop(&htim6);//Stop the timer
}

/**
  * @brief  初始化并使能DWT CYCCNT计数器
  * @retval 1: 成功; 0: 失败 (可能芯片不支持)
  */
uint8_t DWT_Init(void)
{
    // 1. 启用DWT模块的访问权限
    DEM_CR |= DEM_CR_TRCENA;
    
    // 2. (针对Cortex-M7内核) 解锁DWT寄存器的写访问
    DWT->LAR = DWT_LAR_KEY; // 或者使用 *(volatile uint32_t *)0xE0001FB0 = 0xC5ACCE55;
    
    // 3. 清零周期计数器
    DWT_CYCCNT = 0;
    
    // 4. 启用周期计数器
    DWT_CR |= DWT_CR_CYCCNTENA;
    
    // 检查计数器是否已启用并开始计数
    if (DWT_CYCCNT) {
        return 1; // 成功
    } else {
        return 0; // 失败
    }
}
