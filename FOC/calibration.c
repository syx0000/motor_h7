#include "calibration.h"
#include <FOC.h>
#include "main.h"

float error_f[2688]={0};                        // error vector rotating forwards  2688=128*p_motor_g->pole_pairs
float error_b[2688]={0};											  // error vector rotating backwards 2688=128*p_motor_g->pole_pairs
float error[2688]={0};                          //2688=128*p_motor_g->pole_pairs
uint32_t   raw_f[2688]={0};											//2688=128*p_motor_g->pole_pairs
uint32_t   raw_b[2688]={0};											//2688=128*p_motor_g->pole_pairs

void OrderPhases()
{
	encoder_calibrating = 1;
	///Checks phase order, to ensure that positive Q current produces
	///torque in the positive direction wrt the position sensor.
	p_encoder_g->rotations = 0;//否则电机长时运行后，再整定会出现问题
	p_motor_g->phase_order = POSITIVE_PHASE_ORDER;
	printf("\n\r Checking phase ordering\n\r");
	float theta_ref = 0;
	float theta_actual = 0;
	float v_d = V_CAL;                                                             //Put all volts on the D-Axis
	float v_q = 0.0f;
	uint16_t sample_counter = 0;

	// 计算步进角度：目标旋转 4π 电角度，约 800 步（与 Calibrate 一致的时序）
	const uint32_t n_steps = 800;
	float delta = 4*PI / n_steps;

	///Set voltage angle to zero, wait for rotor position to settle
	ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
	HAL_Delay(1000);

	float theta_start;

	/// Rotate voltage angle
	while (theta_ref < 4*PI)//rotate for 2 electrical cycles
	{
		ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
		HAL_Delay(0);
		__disable_irq();  // 临界区保护：防止与 TIM1 ISR 中的 EncoderSample 重入
		EncoderSample();
		theta_actual = p_encoder_g->pos_abs;
		__enable_irq();
		if (theta_ref < 0.001f)  // 使用容差判断而非直接相等比较
		{
			theta_start = theta_actual;
		}
		if (sample_counter > 50)
		{
			sample_counter = 0 ;
			printf("%.4f   %.4f\n\r", theta_ref/p_motor_g->pole_pairs, theta_actual);
		}
		sample_counter++;
		theta_ref += delta;  // 使用计算的 delta
    }
    
	float theta_end = p_encoder_g->pos_abs;
	// 卡死检测：若转动量小于 0.1 rad（约 6°），认为电机未正常转动
	if (fabsf(theta_end - theta_start) < 0.1f)
	{
		printf("\n\rERROR: Motor stalled during phase ordering (moved %.4f rad)\n\r", fabsf(theta_end - theta_start));
		encoder_calibrating = 0;
		return;
	}
	int direction = ((theta_end - theta_start) > 0);//编码器增加
	printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
	printf("Direction:  %d\n\r", direction);
	if (direction)
	{
		printf("Encoder&Motor in same order\n\r");
	}
	else if (!direction)
	{
		printf("Encoder&Motor in different order\n\r");
	}
	p_motor_g->phase_order = direction;
}

/// Measures the electrical angle offset of the position sensor
/// and (in the future) corrects nonlinearity due to position sensor eccentricity（偏心）
void Calibrate()
{
	OrderPhases();//相序检测
	HAL_Delay(500);

	const 	uint32_t n = 128*21;              			// number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
	const 	uint32_t n2 = 1;                  			// increments between saved samples (原设计 n2=40 用于平滑运动，当前简化为 1)
	float 	delta = 2*PI*p_motor_g->pole_pairs/(n*n2);      // change in angle between samples

	float theta_ref = 0;
	float theta_actual = 0;
	float v_d = V_CAL;              						    // Put volts on the D-Axis
	float v_q = 0.0f;

	HAL_Delay(0);
	//Set voltage angle to zero, wait for rotor position to settle
	ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
	HAL_Delay(2000);

	printf("rotations Current Angle  Rotor Angle  error  Raw Encoder\n\r");
	uint16_t print_counter = 0;
	for (int i = 0; i<n; i++)//n=128*NPP
	{// rotate forwards
		for (int j = 0; j<n2; j++)//n2=1（原设计 40）
		{
			theta_ref += delta;//2*PI*NPP/(n*n2)
			ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
			HAL_Delay(1);//2ms
		}
		__disable_irq();  // 临界区保护
		EncoderSample();
		theta_actual = p_encoder_g->pos_abs;
		__enable_irq();
		error_f[i] = theta_actual - theta_ref/(float)p_motor_g->pole_pairs;//实际角度-开环定位角度
		raw_f[i] = p_encoder_g->mech_pos;//编码器原始值
//		printf("%.4f,%.4f,%.4f\n\r",p_motor_g->phase_a_current,p_motor_g->phase_b_current,p_motor_g->phase_c_current);

		// 降低打印频率：每 200 次打印一次，减少 UART 阻塞（原来每次都打印）
		if (print_counter > 200)
		{
			print_counter = 0;
			printf("%d   %.4f   %.4f   %.4f   %d\n\r", p_encoder_g->rotations, theta_ref/(float)p_motor_g->pole_pairs, theta_actual, error_f[i], raw_f[i]);
		}
		print_counter++;
	} 
    
	print_counter = 0;
	for (int i = 0; i<n; i++)
	{	// rotate backwards
		for (int j = 0; j<n2; j++)
		{
			theta_ref -= delta;//从2*PI*NPP开始减
			ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
			HAL_Delay(1);//2ms
		}
		__disable_irq();  // 临界区保护
		EncoderSample();
		theta_actual = p_encoder_g->pos_abs;
		__enable_irq();
		error_b[i] = theta_actual - theta_ref/(float)p_motor_g->pole_pairs;
		raw_b[i] = p_encoder_g->mech_pos;

		if (print_counter > 200)
		{
			print_counter = 0;
			printf("%d   %.4f   %.4f   %.4f   %d\n\r", p_encoder_g->rotations, theta_ref/p_motor_g->pole_pairs, theta_actual, error_b[i], raw_b[i]);
		}
		print_counter++;
	}    
        
	float offset = 0;                                  
	for (int i = 0; i<n; i++)
	{
		offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
	}                         
	offset = Mod(offset*p_motor_g->pole_pairs,0,PI_TIMES_2);								// convert mechanical angle to electrical angle

	// 范围检查：offset 必须在 [0, 2π] 内
	if (offset < 0 || offset > PI_TIMES_2)
	{
		printf("\n\rERROR: Invalid offset %.4f (out of range [0, 2π])\n\r", offset);
		p_encoder_g->cali_finish = 0;
		encoder_calibrating = 0;
		return;
	}

	HAL_Delay(10);

	printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
	p_encoder_g->elec_offset = offset;
	p_encoder_g->cali_finish = 1;
	encoder_calibrating = 0;
}

void PositionSensor_WriteLUT(int32_t new_lut[128])
{
}
