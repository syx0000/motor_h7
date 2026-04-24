#include "calibration.h"
#include <FOC.h>
#include "main.h"

float error_f[2688]={0};                        // error vector rotating forwards  2688=128*p_motor_g->pole_pairs
float error_b[2688]={0};											  // error vector rotating backwards 2688=128*p_motor_g->pole_pairs
float error[2688]={0};                          //2688=128*p_motor_g->pole_pairs
//float error_filt[2688]={0};											//2688=128*p_motor_g->pole_pairs
uint32_t   raw_f[2688]={0};											//2688=128*p_motor_g->pole_pairs
uint32_t   raw_b[2688]={0};											//2688=128*p_motor_g->pole_pairs
//int32_t offset_lut[128]={0};
//int32_t offset_lut[128]={-28,-26,-23,-21,-20,-17,-14,-12,-10,-6,-3,-2,2,5,6,9,12,14,15,17,20,20,21,24,25,24,25,27,26,25,
//26,26,25,24,24,23,21,21,20,17,16,16,14,11,11,10,7,6,5,4,2,1,1,-1,-2,-1,-2,-3,-2,-2,-2,-2,-1,0,0,1,3,3,3,5,6,6,7,9,9,10,11,12,
//12,12,14,13,13,14,14,13,13,13,13,11,11,11,9,7,7,5,2,1,0,-3,-6,-7,-9,-12,-14,-15,-18,-21,-22,-23,-26,-28,-29,-31,-33,-33,-34,
//-35,-36,-35,-35,-36,-35,-34,-34,-33,-30,-29};
void order_phases()
{
	///Checks phase order, to ensure that positive Q current produces
	///torque in the positive direction wrt the position sensor.
	p_encoder_g->rotations = 0;//否则电机长时运行后，再整定会出现问题
//	int32_t lut[128]={0};                      			// 位置线性化表,为了清零，以免影响重新整定
//	PositionSensor_WriteLUT(lut);//offset_lut清零
	p_motor_g->phase_order = POSITIVE_PHASE_ORDER;
	printf("\n\r Checking phase ordering\n\r");
	float theta_ref = 0;
	float theta_actual = 0;
	float v_d = V_CAL;                                                             //Put all volts on the D-Axis
	float v_q = 0.0f;
	uint16_t sample_counter = 0;
	///Set voltage angle to zero, wait for rotor position to settle
	ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
	HAL_Delay(1000);
    
	float theta_start;
    
	/// Rotate voltage angle
	while(theta_ref < 4*PI)//rotate for 2 electrical cycles
	{                                                       
		ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
		HAL_Delay(0);
		encoderSample();//采集最新鲜的编码器位置  在执行该函数时进了TIM1更新中断怎么办？？？函数重入？
		theta_actual = p_encoder_g->pos_abs;
		if(theta_ref==0)
		{
			theta_start = theta_actual;
		}
		if(sample_counter > 50)
		{
			sample_counter = 0 ;
			printf("%.4f   %.4f\n\r", theta_ref/p_motor_g->pole_pairs, theta_actual);
		}
		sample_counter++;
		theta_ref += 0.005f;
//		theta_ref += 0.1f;
    }
    
	float theta_end = p_encoder_g->pos_abs;
	int direction = ((theta_end - theta_start) > 0);//编码器增加
	printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
	printf("Direction:  %d\n\r", direction);
	if(direction)
	{
		printf("Encoder&Motor in same order\n\r");
	}
	else if(!direction)
	{
		printf("Encoder&Motor in different order\n\r");
	}
	p_motor_g->phase_order = direction;
}

/// Measures the electrical angle offset of the position sensor
/// and (in the future) corrects nonlinearity due to position sensor eccentricity（偏心）
void calibrate()
{
	order_phases();//相序检测
	HAL_Delay(500);
	
//	printf("\n\r Starting calibration procedure\n\r");
	const 	uint32_t n_lut = 128;
	const 	uint32_t window = 128;
	const 	uint32_t n = 128*21;              			// number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
	const 	uint32_t n2 = 1;                  			// increments between saved samples (for smoothing motion)
	int32_t lut[128]={0};                      			// 位置线性化表
	float 	delta = 2*PI*p_motor_g->pole_pairs/(n*n2);      // change in angle between samples

//	PositionSensor_WriteLUT(lut);//offset_lut清零
//	for(uint32_t i=0;i<2688;i++) error_filt[i] = 0;	//以免连续两次整定error_filt未清零

	float theta_ref = 0;
	float theta_actual = 0;
	float v_d = V_CAL;              						    // Put volts on the D-Axis
	float v_q = 0.0f;

	HAL_Delay(0);
	//Set voltage angle to zero, wait for rotor position to settle
	ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
	HAL_Delay(2000);

	printf("rotations Current Angle  Rotor Angle  error  Raw Encoder\n\r");
	for(int i = 0; i<n; i++)//n=128*NPP
	{// rotate forwards
		for(int j = 0; j<n2; j++)//n2=40  for smoothing motion
		{
			theta_ref += delta;//2*PI*NPP/(n*n2)
			ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
			HAL_Delay(1);//2ms
		}
		encoderSample();//采集最新鲜的编码器位置
		theta_actual = p_encoder_g->pos_abs;
		error_f[i] = theta_actual - theta_ref/(float)p_motor_g->pole_pairs;//实际角度-开环定位角度
		raw_f[i] = p_encoder_g->mech_pos;//编码器原始值
//		printf("%.4f,%.4f,%.4f\n\r",p_motor_g->phase_a_current,p_motor_g->phase_b_current,p_motor_g->phase_c_current);

		printf("%d   %.4f   %.4f   %.4f   %d\n\r", p_encoder_g->rotations, theta_ref/(float)p_motor_g->pole_pairs, theta_actual, error_f[i], raw_f[i]);
	} 
    
	for(int i = 0; i<n; i++)
	{	// rotate backwards
		for(int j = 0; j<n2; j++)
		{
			theta_ref -= delta;//从2*PI*NPP开始减
			ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
			HAL_Delay(1);//2ms
		}
		encoderSample();//采集最新鲜的编码器位置
		theta_actual = p_encoder_g->pos_abs;
		error_b[i] = theta_actual - theta_ref/(float)p_motor_g->pole_pairs;
		raw_b[i] = p_encoder_g->mech_pos;
//		printf("%.4f,%.4f,%.4f\n\r",p_motor_g->phase_a_current,p_motor_g->phase_b_current,p_motor_g->phase_c_current);

		printf("%d   %.4f   %.4f   %.4f   %d\n\r", p_encoder_g->rotations, theta_ref/p_motor_g->pole_pairs, theta_actual, error_b[i], raw_b[i]);//耗时近3ms
	}    
        
	float offset = 0;                                  
	for(int i = 0; i<n; i++)
	{
		offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
	}                         
	offset = Mod(offset*p_motor_g->pole_pairs,0,PI_TIMES_2);								// convert mechanical angle to electrical angle
	HAL_Delay(10);
		
    /// Perform filtering to linearize position sensor eccentricity（偏心）
    /// FIR n-sample average, where n = number of samples in one electrical cycle
    /// This filter has zero gain at electrical frequency and all integer multiples
    /// So cogging effects should be completely filtered out.

//    float mean = 0;
//    for (int i = 0; i<n; i++)//n = 128*21  Average the forward and back directions
//    {                                             
//        error[i] = (error_f[i] + error_b[n-i-1])/2.0f;
//    }//128*NPP个位置处的偏差（编码器反馈角度-定位角度）
//    
//    /* 对error[n]进行FIR滤波，得到error_filt[n] 得到的error_filt[i]是error[i]前后128个数的平均*/
//    for (uint32_t i = 0; i<n; i++)
//    {
//        for(uint32_t j = 0; j<window; j++)//window==128
//        {
//            int32_t ind =  j + i - window/2;        // Indexes from -window/2 to + window/2  -64 to +64（i=0）   -63 to +65（i=1）
//            if(ind < 0)
//            {
//                ind += n;//-64+128*NPP
//            }                                  		  // Moving average wraps around 环形滑窗滤波
//            else if(ind > n-1)
//            {
//                ind -= n;
//            }					
//            error_filt[i] += error[ind]/(float)window;//error_filt[0]=(error[-64+128*NPP]+error[-63+128*NPP]+error[-62+128*NPP]+...+error[0]+error[1]+...+error[63])/128
//        }
//        mean += error_filt[i]/n;//对error_filt求均值		
//    }

//    uint32_t raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty
//    printf("\n\r Encoder non-linearity compensation table\n\r");
//    printf(" Sample Number   Lookup Index   Lookup Value\n\r\n\r");
//    for (uint32_t i = 0; i<n_lut; i++)//n_lut=128
//    {                                          // build lookup table
//        uint32_t ind = (raw_offset>>7) + i;
//        if(ind > (n_lut-1))
//        { 
//            ind -= n_lut;
//        }
//				//round！！！
//        lut[ind] = (int32_t) round(((error_filt[i*p_motor_g->pole_pairs] - mean)*(float)(p_encoder_g->cpr)/(2.0f*PI)));//error_filt是包含电机零位的，所以补偿非线性误差时要减去mean
//				printf("%d   %d   %d\n\r", i, ind, lut[ind]);
//    }

//    PositionSensor_WriteLUT(lut);                                                      // write lookup table to position sensor object
	printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
	p_encoder_g->elec_offset = offset;		
	p_encoder_g->cali_finish = 1;
}

void PositionSensor_WriteLUT(int32_t new_lut[128])
{
//    memcpy(offset_lut, new_lut, sizeof(offset_lut));
}
