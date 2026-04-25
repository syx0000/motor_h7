#include "motor.h"
#include <FOC.h>
#include "main.h"
#include "stdlib.h"
/* ------------------------------ Public Function Declarations ------------------------------ */
//static void Init(void);
static void MeasureResistance(void);
static void MeasureInductance(void);
//static void MeasureFluxAndInertia(void);
/* ------------------------------ Variable Declarations ------------------------------ */
Motor_t motor_g;
Motor_t *p_motor_g = &motor_g;
volatile static uint8_t measure_time = 0;
const uint8_t measure_induct_num = 15; // the number of data that we use when measure inductance in one period
const float theta_test = 0.0f;
const float voltage_test = V_CAL;//整定电阻和电感时的测试电压
float current_a[30] = {0.0f};
float current_sum[30] = {0.0f};
const uint8_t measure_flux_time = 20;
//float prvelec_vel[measure_flux_time] = {0.0f}, Q_axis_volt[measure_flux_time]={0.0f};
float prvmech_vel[5] = {0.0f};
uint8_t count_test = 0;
const float one_by_sample_resistance = 250.0f; // 1/0.004
const float sample_resistance = 0.0025f;//采样电阻

//const float voltage_coefficient = 0.016193848;//1M 910k 100k
//static const float voltage_coefficient = 0.0169189453125;//1M 1M 100k
//static const float voltage_coefficient = VOLTAGE_RES_DIVIDE*ADC_supply/ADC_resolution;
static const float voltage_coefficient = ADC_supply/ADC_resolution;//1M 1M 100k
/*电流采样*/
#define HALL_SENSITIVITY 0.044f // 44mV/A
/*温度采样*/
#define B_CONST_MOS 3950.0f
#define RES_DIVIDE_MOS 10.0f // kΩ
#define NOMINAL_RES_MOS 10.0f // kΩ

#define B_CONST_MOTOR 3435.0f
#define RES_DIVIDE_MOTOR 10.0f // kΩ
#define NOMINAL_RES_MOTOR 10.0f // kΩ

#define ABSOLUTE_ZERO 273.15f // K

float TEMP_MOTOR = 0.0f;
float TEMP_MOS = 0.0f;
float TEMP_MOTOR_filter1 = 0.0f;
float TEMP_MOTOR_filter2 = 0.0f;
float TEMP_MOS_filter1 = 0.0f;
uint8_t Current_errorCount = 0;
void Motor_Init(void)
{
	p_motor_g->vbus = 48;
	p_motor_g->phase_order = NEGATIVE_PHASE_ORDER;//反相序
	p_motor_g->pole_pairs = 8;     // @todo flash
	p_motor_g->IMax = 60.0f * CURRENT_COMPENSATION_RATIO;//电流限制
	p_motor_g->current_loop_bandwidth = PWM_FREQUENCY_DEFAULT / 10.0f; // @todo flash 10k/10 = 1k
	p_motor_g->velocity_loop_bandwidth = 100.0f;
	p_motor_g->direct_current = 0.6f;
	p_motor_g->flux_current = 0.6f;
	p_motor_g->cali_voltage = 0.6f;
	p_motor_g->phase_resistance = 0.08813667f;
	p_motor_g->phase_inductance = 0.00006076f;
	p_motor_g->sta_flux = 0.0f;
	/* set calibrated as 0, motor won't run if motor_calibrated is not 1 */
	p_motor_g->motor_calibrated = 0;
	p_motor_g->flux_measured = 0;
	p_motor_g->inertia_measured = 0;
	/* initiate variables as 0 */
	p_motor_g->phase_a_current_offset = 32768.0f;
	p_motor_g->phase_b_current_offset = 32768.0f;
	p_motor_g->phase_c_current_offset = 32768.0f;
	p_motor_g->phase_a_current = 0.0f;
	p_motor_g->phase_b_current = 0.0f;
	p_motor_g->phase_c_current = 0.0f;
	p_motor_g->alpha_axis_current = 0.0f;
	p_motor_g->beta_axis_current = 0.0f;
	p_motor_g->D_axis_current = 0.0f;
	p_motor_g->Q_axis_current = 0.0f;
	p_motor_g->max_current_ever = 0.0f;
	p_motor_g->max_current_now = 0.0f;
	
	p_motor_g->error = Normal;
	p_motor_g->Err1 = MotorErr1_Nomal;
	p_motor_g->Err2 = MotorErr1_Nomal;
	p_motor_g->Warning = MotorWarning_Nomal;
//	voltage_coefficient = VOLTAGE_RES_DIVIDE * 3.3f / 4096.0f;
	
	p_motor_g->controlMode = FOC_POSITION_LOOP;//FOC_VELOCITY_LOOP;//默认FOC速度模式
	p_motor_g->i_d_ref = 0;
	p_motor_g->i_q_ref = 0;
	
	p_motor_g->lastError = 0;

	// 电机温度初始化（防止ADC = 4095除零）
	float adc1_voltage = (float)ADC1->DR * 3.3f / 4095.0f;
	if (adc1_voltage >= 3.2f) // 接近满量程，分母接近0
	{
		TEMP_MOTOR_filter1 = 25.0f; // 默认室温
	}
	else
	{
		float r1_ntc = 10.0f * adc1_voltage / (3.3f - adc1_voltage);
		if (r1_ntc > 0.01f) // 防止log(0)
			TEMP_MOTOR_filter1 = 1.0f / ((1.0f / 298.15f) + (logf(r1_ntc / 10.0f) / 3950.0f)) - 273.15f;
		else
			TEMP_MOTOR_filter1 = 25.0f;
	}
	TEMP_MOTOR_filter2 = TEMP_MOTOR_filter1;

	// MOS温度初始化（防止ADC = 4095除零）
	float adc2_voltage = (float)ADC2->DR * 3.3f / 4095.0f;
	if (adc2_voltage >= 3.2f)
	{
		TEMP_MOS_filter1 = 25.0f;
	}
	else
	{
		float r2_ntc = 3.3f * adc2_voltage / (3.3f - adc2_voltage);
		if (r2_ntc > 0.01f)
			TEMP_MOS_filter1 = 1.0f / ((1.0f / 298.15f) + (logf(r2_ntc / 10.0f) / 3380.0f)) - 273.15f;
		else
			TEMP_MOS_filter1 = 25.0f;
	}
  /* set volt to amp conversion rate */
}


float pos_estimate_counts_ = 0.0f;  //当前估算的位置值，单位[count]  
float pos_cpr_counts_ = 0.0f;       //当前约束在cpr范围内的位置值，单位[count]
float current_meas_period = 0.0001;
float vel_estimate_counts_ = 0.0f;  //当前估算转速，单位[count/s]
//float pll_kp_ = 2000.0f;   // [count/s / count] Kp = 2ζωn，Ki = ωn2
//float pll_ki_ = 1000000.0f;   // [(count/s^2) / count]  0.25*pll_kp_^2
//float pll_kp_ = 2000.0f;   // [count/s / count] Kp = 2ζωn，Ki = ωn2
//float pll_ki_ = 2040816.3f;   // [(count/s^2) / count]
float pll_kp_ = 1000.0f;   // [count/s / count] Kp = 2ζωn，Ki = ωn2
float pll_ki_ = 510204.1f;   // [(count/s^2) / count]
//float pll_kp_ = 500.0f;   // [count/s / count] Kp = 2ζωn，Ki = ωn2
//float pll_ki_ = 127551.0f;   // [(count/s^2) / count]
//float pll_kp_ = 140.0f;   // [count/s / count]
//float pll_ki_ = 10000.0f;   // [(count/s^2) / count]  0.25*pll_kp_^2
float pos_estimate_ = 0.0f; //当前估算的位置值，单位[rad]
float vel_estimate_ = 0.0f; //当前估算关节输出端转速，单位[rad/s]

int32_t shadow_count_ = 0;   //编码器累计计数。
int32_t count_in_cpr_ = 0;   //编码器当前计数值。

void RunPllVelocity()
{
//	pos_cpr_counts_ = WrapPm(2, 3);
	
	int32_t delta_enc = 0;
	int32_t pos_abs_latched = p_encoder_g->mech_pos;
	
	delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
	delta_enc = EncoderMod(delta_enc,16384);
	if (delta_enc > 8192)delta_enc -= 16384;
	
	shadow_count_ += delta_enc;
	count_in_cpr_ += delta_enc;//count_in_cpr_=pos_abs_latched=pos_abs_
	count_in_cpr_ = EncoderMod(count_in_cpr_,16384);
	count_in_cpr_ = pos_abs_latched;

	// run pll (for now pll is in units of encoder counts)
	// Predict current pos
	pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
	pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
	
	// discrete phase detector
	float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
	float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
	delta_pos_cpr_counts = WrapPm(delta_pos_cpr_counts, 21);//不能用p_motor_g->pole_pairs？
	// delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
	// pll feedback
	pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
	pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
	pos_cpr_counts_ = FmodfPos(pos_cpr_counts_, 16384);//不能用p_encoder_g->cpr？
	vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
//	uint8_t snap_to_zero_vel = false;
//	if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
//	{
//		vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
//		snap_to_zero_vel = true;
//	}
	
	// Outputs from Encoder for Controller
	pos_estimate_ = (pos_estimate_counts_ / (float)p_encoder_g->cpr)*2*PI;
//	pos_estimate_ = Mod(pos_estimate_,0.0f,PI_TIMES_2);//当pos_estimate_特别大的时候，会卡死...
	vel_estimate_ = (vel_estimate_counts_ / (float)p_encoder_g->cpr)*2*PI;
}

/**
 * @brief  brief.
 * @note   give d-axis voltage and measure d-axis current,then calculate the d-axis resistance
 * @retval None
 */
static void MeasureResistance(void)
{
  float volt_d_test = voltage_test, volt_q_test = 0.0f;//1V
  float d_current_sum = 0.0f, d_current_ave = 0.0f;

  ApplyVoltDQToSVPWM(volt_d_test, volt_q_test, theta_test); // apply a voltage  将电机吸到电角度为0的位置
  HAL_Delay(1000);// wait for some time to settle
	
  for (int i = 0; i < 800; i++)
  {
		d_current_sum += p_motor_g->phase_a_current;//必须是a相电流，因为theta = 0时，d轴电压与a相电压相等
    HAL_Delay(1); // wait for some time
  }
  d_current_ave = d_current_sum / 800.0f;
  p_motor_g->phase_resistance = volt_d_test / d_current_ave;

  printf("volt:%f,phase resistance: %f\r\n",volt_d_test, p_motor_g->phase_resistance);
}

void MotorStop(void)
{
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0xffff);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0xffff);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0xffff);
}

/**
 * @brief  Measure D_axis inductance
 * @note   Principle: if we apply square waves to an inductance, the output wave
 *  		    would be a triangle wave.
 *  		    inductance = delta_v / delta_i
 * @retval None
 */
static void MeasureInductance(void)
{
	float volt_d_test = voltage_test, volt_q_test = 0.0f;//1V
	float K = 0.0f;
  uint8_t sampletime = 200;
	for (int k = 0; k < measure_induct_num; k++)//measure_induct_num = 15
			current_sum[k] = 0.0f;
	
  for (int i = 0; i < sampletime; i++)
  {
    ApplyVoltDQToSVPWM(volt_d_test, volt_q_test, theta_test); // apply a voltage 将电机吸到电角度为0的位置
    HAL_Delay(5);
    MotorStop();
    measure_time = measure_induct_num;
    while (measure_time)//每进行一次电流采样就减1
      ;
		for (int k = 0; k < measure_induct_num; k++)
			current_sum[k] += current_a[k];
  }
	//average for reducing the random error
	for (int k = 0; k < measure_induct_num; k++)
	{
		current_sum[k] /= (float)sampletime;
//		printf("%f \r\n", current_sum[k]);
		current_sum[k] = IterationLn(current_sum[k], 100);//ln(current_sum[k])
	}
	// remove the first value
	if (least_square_method(current_sum + 1, measure_induct_num - 1, &K))
	{
		p_motor_g->phase_inductance = p_motor_g->phase_resistance / (-K * PWM_FREQUENCY_DEFAULT);
		printf("K:%f,phase inductance: %.8f\r\n",K, p_motor_g->phase_inductance);
		p_motor_g->motor_calibrated = 1;
	}
	else
	{
		HAL_Delay(5);
		p_motor_g->phase_inductance = p_motor_g->phase_resistance / (-K * PWM_FREQUENCY_DEFAULT);
		printf("K:%f,phase inductance: %.8f\r\n",K, p_motor_g->phase_inductance);
		p_motor_g->motor_calibrated = 0;
	}
}
void CalcCurrentOffset(float *phase_a_offset, float *phase_b_offset, float *phase_c_offset)
{
	p_motor_g->volt2amp_rate =  ADC_supply/ADC_resolution;
  /* record offset summary */
  float a_offset_sum = 0.0f;
	float b_offset_sum = 0.0f;
	float c_offset_sum = 0.0f;
  /* adc1_Rank1 for phase A, adc1_Rank2 for phase B*/
  uint16_t currentA_raw, currentB_raw, currentC_raw;
  /* sample 500 times, then get average */
  for (uint16_t i = 0; i < 5000; i++)
  {
		//过采样
//		uint32_t ADC_Value_sum[2] = {0};
//		/* 在采样值数组中分别取出每个通道的采样值并求和 */
//		for (uint8_t i = 0;i < ADC2_CHANNELS_WINDOW;i ++)
//		{
//			ADC_Value_sum[0] +=  ADC_Cur_vbus_Value[i*ADC2_CHANNELS+0];
//			ADC_Value_sum[1] +=  ADC_Cur_vbus_Value[i*ADC2_CHANNELS+1];
//		}
//    currentA_raw = ADC_Value_sum[0]>>ADC2_CHANNELS_WINDOW_movebit;
//    currentB_raw = ADC_Value_sum[1]>>ADC2_CHANNELS_WINDOW_movebit;
		hadc1.Instance->CR  |= ADC_CR_JADSTART;
		HAL_Delay(0);
    currentA_raw = ADC1->JDR2;
    currentB_raw = ADC1->JDR1;
		currentC_raw = ADC1->JDR3;
		printf("%d   %d   %d\r\n",currentA_raw,currentB_raw,currentC_raw);
		if (i>=2)
		{
			a_offset_sum += (float) currentA_raw;
			b_offset_sum += (float) currentB_raw;
			c_offset_sum += (float) currentC_raw;
		}
  }
	/*前两个数据不对*/
  *phase_a_offset = a_offset_sum / 4998.0f;
	*phase_b_offset = b_offset_sum / 4998.0f;
	*phase_c_offset = c_offset_sum / 4998.0f;

//#if defined(JM65_DP28_DE_C)
//  *phase_a_offset = a_offset_sum / 500.0f;
//	*phase_b_offset = b_offset_sum / 500.0f;
//#elif defined(JM57_DP18_DE_C)//手动校准电流偏置
//	*phase_a_offset = a_offset_sum / 500.0f + 13.54f;
//	*phase_b_offset = b_offset_sum / 500.0f + 6.69f;
//#elif defined(FIVE_2808)//手动校准电流偏置
//	*phase_a_offset = a_offset_sum / 500.0f;
//	*phase_b_offset = b_offset_sum / 500.0f;
//#else
//    #error "未定义产品型号！"
//#endif

	printf("Phase Current Offset:a:%f b:%f c:%f\r\n",p_motor_g->phase_a_current_offset,p_motor_g->phase_b_current_offset,p_motor_g->phase_c_current_offset);
}
void CurrentSample()
{
	//过采样
//		uint32_t ADC_Value_sum[2] = {0};
//		/* 在采样值数组中分别取出每个通道的采样值并求和 */
//		for (uint8_t i = 0;i < ADC2_CHANNELS_WINDOW;i ++)
//		{
//			ADC_Value_sum[0] +=  ADC_Cur_vbus_Value[i*ADC2_CHANNELS+0];
//			ADC_Value_sum[1] +=  ADC_Cur_vbus_Value[i*ADC2_CHANNELS+1];
//		}
//		float CurrentA_Raw = (float)(ADC_xValue_sum[0]>>ADC2_CHANNELS_WINDOW_movebit) - p_motor_g->phase_a_current_offset;
//		float CurrentB_Raw = (float)(ADC_Value_sum[1]>>ADC2_CHANNELS_WINDOW_movebit) - p_motor_g->phase_b_current_offset;
		
		float CurrentA_Raw = (float)ADC1->JDR2 - p_motor_g->phase_a_current_offset;
		float CurrentB_Raw = (float)ADC1->JDR1 - p_motor_g->phase_b_current_offset;
		float CurrentC_Raw = (float)ADC1->JDR3 - p_motor_g->phase_c_current_offset;
		if (p_motor_g->phase_order == POSITIVE_PHASE_ORDER)
		{
//			p_motor_g->phase_b_current = (p_motor_g->volt2amp_rate * CurrentB_Raw)/10.0f/0.005f;
//			p_motor_g->phase_a_current = (p_motor_g->volt2amp_rate * CurrentA_Raw)/10.0f/0.005f;
			p_motor_g->phase_b_current = (p_motor_g->volt2amp_rate * CurrentB_Raw)/10.0f/sample_resistance;
			p_motor_g->phase_c_current = (p_motor_g->volt2amp_rate * CurrentC_Raw)/10.0f/sample_resistance;
		}
		else
		{
//			p_motor_g->phase_b_current = (p_motor_g->volt2amp_rate * CurrentA_Raw)/10.0f/0.005f;
//			p_motor_g->phase_a_current = (p_motor_g->volt2amp_rate * CurrentB_Raw)/10.0f/0.005f;
			p_motor_g->phase_b_current = (p_motor_g->volt2amp_rate * CurrentC_Raw)/10.0f/sample_resistance;
			p_motor_g->phase_c_current = (p_motor_g->volt2amp_rate * CurrentB_Raw)/10.0f/sample_resistance;
		}
//		p_motor_g->phase_c_current = -p_motor_g->phase_a_current - p_motor_g->phase_b_current;
//		p_motor_g->phase_c_current_actual = (p_motor_g->volt2amp_rate * CurrentC_Raw)/10.0f/0.005f;
		p_motor_g->phase_a_current = -p_motor_g->phase_c_current - p_motor_g->phase_b_current;
		p_motor_g->phase_a_current_actual = (p_motor_g->volt2amp_rate * CurrentA_Raw)/10.0f/sample_resistance;
		
/*一阶低通滤波*/
//		p_motor_g->phase_a_current_filt = 0.2*p_motor_g->phase_a_current_filt + 0.8*p_motor_g->phase_a_current;
//		p_motor_g->phase_b_current_filt = 0.2*p_motor_g->phase_b_current_filt + 0.8*p_motor_g->phase_b_current;
//		p_motor_g->phase_c_current_filt = -p_motor_g->phase_b_current_filt - p_motor_g->phase_a_current_filt;
//		p_motor_g->phase_c_current_filt = 0.8*p_motor_g->phase_c_current_filt + 0.2*p_motor_g->phase_c_current;
//		p_motor_g->phase_b_current_filt = 0.8*p_motor_g->phase_b_current_filt + 0.2*p_motor_g->phase_b_current;
//		p_motor_g->phase_a_current_filt = -p_motor_g->phase_b_current_filt - p_motor_g->phase_c_current_filt;
		
		if (measure_time)//整定电感时用到
		{
			current_a[measure_induct_num - measure_time] = p_motor_g->phase_a_current;
			measure_time--;
		}
}

#define SAMPLE_CNT 1000
void Calc_current_rms(void)
{
    float sumA = 0,sumB = 0,sumC = 0;
    // 1. 平方累加
    for (int i = 0; i<SAMPLE_CNT; i++)
	{
        sumA += p_motor_g->phase_a_current * p_motor_g->phase_a_current;
		sumB += p_motor_g->phase_b_current * p_motor_g->phase_b_current;
		sumC += p_motor_g->phase_c_current * p_motor_g->phase_c_current;
    }
    // 2. 均方根
    p_motor_g->phase_a_Current_RMS =  sqrt(sumA / SAMPLE_CNT);
	p_motor_g->phase_b_Current_RMS =  sqrt(sumB / SAMPLE_CNT);
	p_motor_g->phase_c_Current_RMS =  sqrt(sumC / SAMPLE_CNT);
}

void VoltageSample()
{
	p_motor_g->vbus = (voltage_coefficient * (float)ADC1->JDR4)*21.0f;
}

void TemperatureSample()
{
	//电机温度计算
//	float r1_ntc = RES_DIVIDE_MOTOR*((float)ADC2->JDR2*ADC_supply/ADC_resolution)/(ADC_supply-((float)ADC2->JDR2*ADC_supply/ADC_resolution));//电机绕组端NTC电阻阻值 单位：kΩ
//	float r1_ntc = RES_DIVIDE_MOTOR*((float)ADC1->DR*ADC_supply/ADC_resolution)/(ADC_supply-((float)ADC1->DR*ADC_supply/ADC_resolution));//电机绕组端NTC电阻阻值 单位：kΩ
	float adc_jdr2 = (float)ADC2->JDR2 / ADC_resolution;
	float r1_ntc;
	if (adc_jdr2 >= 0.97f) // 更保守的阈值，防止除零
		r1_ntc = 999.0f;
	else
		r1_ntc = RES_DIVIDE_MOTOR * (adc_jdr2 / (1.0f - adc_jdr2));
	if (r1_ntc < 0.01f) r1_ntc = 0.01f; // 防止log(0)
	TEMP_MOTOR = 1.0f / ( (1.0f / (ABSOLUTE_ZERO + 25.0f) ) + (logf(r1_ntc / NOMINAL_RES_MOTOR) / B_CONST_MOTOR ) ) - ABSOLUTE_ZERO;//单位：摄氏度
//	计算绝对差值
//  float delta = fabsf(TEMP_MOTOR - TEMP_MOTOR_filter1);
//  TEMP_MOTOR_filter1 = ((delta > 1) ? TEMP_MOTOR_filter1 : TEMP_MOTOR);//100us升不了1℃吧？
//	增强版
	static uint8_t errorCount = 0;
	float delta = fabsf(TEMP_MOTOR - TEMP_MOTOR_filter1);
	if (delta > 0.5f) {
        errorCount++;
        if (errorCount > 99) {  // 连续100次(10ms)超限则认为有效变化
            errorCount = 0;
            TEMP_MOTOR_filter1 = TEMP_MOTOR;
        }
    } 
    else {
        errorCount = 0;
        TEMP_MOTOR_filter1 = TEMP_MOTOR;
    }
	TEMP_MOTOR_filter2 = 0.6f*TEMP_MOTOR_filter1 + 0.4f*TEMP_MOTOR_filter2;//一阶低通
	
	//MOS温度计算
	//共约3.5us 使用热敏电阻的Steinhart-Hart方程进行温度计算
//	float r2_ntc = 3.3f*3.3f/((float)ADC2->DR*3.3f/4095.0f)-3.3;//kΩ
//	float r2_ntc = RES_DIVIDE_MOS*((float)ADC2->JDR1*ADC_supply/ADC_resolution)/(ADC_supply-((float)ADC2->JDR1*ADC_supply/ADC_resolution));//kΩ
//	float r2_ntc = RES_DIVIDE_MOS*((float)ADC2->DR*ADC_supply/ADC_resolution)/(ADC_supply-((float)ADC2->DR*ADC_supply/ADC_resolution));//kΩ
	
	float adc_jdr1 = (float)ADC2->JDR1 / ADC_resolution;
	float r2_ntc;
	if (adc_jdr1 >= 0.97f) // 更保守的阈值，防止除零
		r2_ntc = 999.0f;
	else
		r2_ntc = RES_DIVIDE_MOS * (adc_jdr1 / (1.0f - adc_jdr1));
	if (r2_ntc < 0.01f) r2_ntc = 0.01f;

	TEMP_MOS = 1.0f / ( (1.0f / (ABSOLUTE_ZERO + 25.0f) ) + (logf(r2_ntc / NOMINAL_RES_MOS) / B_CONST_MOS ) ) - ABSOLUTE_ZERO;//摄氏度
	TEMP_MOS_filter1 = 0.6f*TEMP_MOS + 0.4f*TEMP_MOS_filter1;//一阶低通
}

/* ------------------------------ Manager Declaration ------------------------------ */
const MotorManager_typedef Motor =
{
//	Init,
//	MeasureCurrent,
//	MeasureVoltage,
	MeasureResistance,
	MeasureInductance,
//	MeasureFluxAndInertia
};

void EnableADC(void)
{
	hadc1.Instance->CR |=  ADC_CR_ADEN;
	hadc2.Instance->CR |=  ADC_CR_ADEN;
//	hadc3.Instance->CR |=  ADC_CR_ADEN;
}

void StartJADC(void)
{
	// ADC1 现在由 TIM1 TRGO 硬件触发，无需软件启动
	// 只保留 ADC2（温度采样）
	HAL_ADCEx_InjectedStart(&hadc2);
}

//差值限幅滤波器
void DeltaFilter(uint8_t errcnt, volatile float phase_current, volatile float *phase_current_filter1, volatile float *phase_current_filter2)
{
	float delta = fabsf(phase_current - *phase_current_filter1);
	if (delta > 0.5f) {
				errcnt++;
				if (errcnt > 1) {  // 连续10次(1ms)超限则认为有效变化
						errcnt = 0;
						*phase_current_filter1 = phase_current;
				}
		}
		else {
				Current_errorCount = 0;
				*phase_current_filter1 = phase_current;
		}
	*phase_current_filter2 = 0.6f*(*phase_current_filter1) + 0.4f*(*phase_current_filter2);//一阶低通
}
