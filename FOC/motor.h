#ifndef __MOTOR_H
#define __MOTOR_H

/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */
#define NEGATIVE_PHASE_ORDER	0.0f
#define POSITIVE_PHASE_ORDER	1.0f
#define Motor_Rph	0.4965f//相电阻实测值(Ω)
#define Motor_Lph	0.00025062f//相电感实测值(H)
#define VOLTAGE_RES_DIVIDE 21.0f // 电压采样分压电阻比例

//#define ADC_sample_time 400.0f; // remain some time for ADC sample

#define MIT_PD						0x00
#define FOC_POSITION_LOOP           0x01
#define FOC_VELOCITY_LOOP           0x02
#define FOC_CURRENT_LOOP            0x03
#define FOC_POSITION_LOOP_PP        0x04

//一级故障（严重故障）
#define MotorErr1_Nomal					0x0000
#define MotorErr1_PhaseOverCurrent		0x0001
#define MotorErr1_OverVolt				0x0002
#define MotorErr1_MosOverT				0x0004
#define MotorErr1_MotorOverT			0x0008
#define MotorErr1_DclinkOverCurrent		0x0010
#define MotorErr1_OverLoad				0x0020
#define MotorErr1_Encoder				0x0040
#define MotorErr1_Other					0x0080
//二级故障（一般故障）
#define MotorErr2_Nomal					0x00
#define MotorErr2_CanRxError			0x01
#define MotorErr2_LowVotage				0x02
#define MotorErr2_OverSpeed				0x04
//三级警告（一般警告）
#define MotorWarning_Nomal				0x00
#define MotorWarning_MosOverT			0x01
#define MotorWarning_MotorOverT			0x02


extern float pos_estimate_; //当前估算的位置值，单位[turn]
extern float vel_estimate_; //当前估算转速，单位[turn/s]
extern float vel_estimate_counts_;  //当前估算转速，单位[count/s]
extern float pos_cpr_counts_; 
extern float TEMP_MOTOR;
extern float TEMP_MOS;
extern float TEMP_MOTOR_filter1;
extern float TEMP_MOTOR_filter2;
extern float TEMP_MOS_filter1;

void CalcCurrentOffset(float *phase_a_offset, float *phase_b_offset, float *phase_c_offset);
//typedef enum _Drv8302_G
//{
//  Amp10,
//	Amp40
//}Drv8302_G;
//void Drv8302_Init(Drv8302_G drv8302_G);
/* ------------------------------ Enum Typedef ------------------------------ */

typedef enum _Motor_Err
{
	Normal = 0,
	/* encoder error */
	As5047pError,
	EncoderShake,
	/* current error */
	SwOverCurrent,
	HwOverCurrent,
	/* volt error */
	OverVolt,
	UnderVolt,
	/*CommunicationError*/
	UsartRxError,
	CanRxError,
	/*TemperatureError*/
	MOS_OverTEMP,
	MOTOR_OverTEMP,
	/* colorful egg just to make enum for four bytes*/
	WORK_TOO_TIRED = 0x1fffffff
}Motor_Err;


/* ------------------------------ Struct Typedef ------------------------------ */
typedef struct _Motor_t
{
  /* parameters to be configurated by user */
	float IMax;
	uint8_t can_id;
	float current_loop_bandwidth;
	uint8_t pole_pairs;
	float velocity_loop_bandwidth;
	/* parameters to be read from flash */
	float phase_resistance;
	float phase_inductance;
	float sta_flux;//phi_f
	uint8_t flux_measured;
	
	float inertia;
	uint8_t inertia_measured;

	float direct_current;
	float cali_voltage;
	float flux_current;

	/* parameters to be configurated outside */
	float phase_a_current_offset;
	float phase_b_current_offset;
	float phase_c_current_offset;
	/* current measure volt to amp rate */
	float volt2amp_rate;
	float phase_order;		/* Phase order will be configurated when calibrating encoder.
							Phase order is automatically configurated, and should not
							be changed by user. */
	/*总线电压*/
	volatile float vbus;
	/* variables */
	volatile float phase_a_current;
	volatile float phase_a_current_filt1;
	volatile float phase_a_current_filt2;
	volatile float phase_a_current_actual;  // A 相计算值（用于调试）

	volatile float phase_b_current;
	volatile float phase_b_current_filt1;
	volatile float phase_b_current_filt2;

	volatile float phase_c_current;
	volatile float phase_c_current_filt1;
	volatile float phase_c_current_filt2;
	
	volatile float phase_a_current_filt;
	volatile float phase_b_current_filt;
	volatile float phase_c_current_filt;
	
	volatile float alpha_axis_current;
	volatile float beta_axis_current;
	
	volatile float D_axis_current;
	volatile float Q_axis_current;
	volatile float D_axis_current_filt;
	volatile float Q_axis_current_filt;
	
	volatile float max_current_ever;
	volatile float max_current_now;
	
	volatile float phase_a_Current_RMS;
	volatile float phase_b_Current_RMS;
	volatile float phase_c_Current_RMS;

	float i_d_ref;
	float i_q_ref;
	uint8_t controlMode;
	uint8_t cali_start;
	uint8_t motor_calibrated;	/* This parameter is to prevent running the motor without
								calibration for safety. */

	/* error code */
	uint8_t lastError;
	Motor_Err error;
	uint16_t Err1;
	uint8_t Err2;
	uint8_t Warning;

	uint8_t homingFlag;//回零
}Motor_t;

/* ------------------------------ Variable Declarations ------------------------------ */
extern Motor_t *p_motor_g;

/* ------------------------------ Manager Typedef ------------------------------ */
typedef struct MotorManager
{
/**
  * @brief  Initiate motor.
  * @note   Motor should be initiated before reading flash.
  * @param  volt2amp_rate the current sample conversion rate from volt to amp,
  *         this value should be the magenification of gate driver divided by 
  *         resistance of sampling resistor in ohm.
  * @retval None
  */
//  void (*Init)(void);

/**
  * @brief  Get phase current and VBUS from ADCs and save into motor struct.
  * @note   This function depends on ADC conversion, thus should be called
  *         after ADC conversions are enabled.
  * @retval None
  */
//  void (*MeasureCurrent)(void);
//  void (*MeasureVoltage)(void);
void (*MeasureResistance)(void);
void (*MeasureInductance)(void);
//	void (*MeasureFluxAndInertia)(void);
}MotorManager_typedef;
/* ------------------------------ Manager Extern ------------------------------ */
extern const MotorManager_typedef Motor;

// ADC3 规则通道采样结果（在 main.c 中定义）
extern volatile uint32_t adc3_vdc_value;
extern volatile uint32_t adc3_temp_mos_value;
extern volatile uint32_t adc3_temp_motor_value;

void Motor_Init(void);
void CurrentSample(void);
void Calc_current_rms(void);
void VoltageSample(void);
void TemperatureSample(void);
void RunPllVelocity(void);
void MotorStop(void);
void EnableADC(void);
void DeltaFilter(uint8_t errcnt, volatile float phase_current, volatile float *phase_current_filter1, volatile float *phase_current_filter2);
#endif

