/**
  ******************************************************************************
  * @file           : encoder.c
  * @author         : MQC
  * @brief          : brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */
void EncoderSample(void);
void Encoder_Init(void);
static void prvCalcVelocity(void);

extern uint8_t vel_loop_flag;
extern uint8_t pos_loop_flag;
extern float speed_w;
extern int32_t delta_time_CNT_T;
extern int32_t delta_encoder_cnt_T;
extern volatile int32_t delta_encoder_cnt_M;
extern volatile int32_t delta_encoderInner_cnt_M;
extern uint8_t vel_zero_cnt;
/* ------------------------------ Enum Typedef ------------------------------ */

/* ------------------------------ Struct Typedef ------------------------------ */
typedef struct _Encoder_t
{
	uint32_t cpr;
	int32_t cpr_div_two;
	float one_div_cpr;
	uint16_t cali_num;
	uint8_t cali_bit;
	
	//pos
	volatile float elec_pos;
	volatile float pos_abs;	//absolute position in rad
//	volatile float pos_abs_norotations;	
	volatile float mechPosition;	//输出端absolute position in rad
	volatile float mech_abs;	//absolute 机械 position in rad
	float several_times_before_pos_abs;
//	float several_times_before_pos_abs_norotations;
	int32_t mech_pos;//编码器机械位置
	int32_t mech_pos_several_times_before;
	int32_t last_mech_pos;
	int32_t delta_mech_pos;
	volatile int32_t rotations;
	
  //vel
	volatile float mech_vel;
	float mech_vel_filt;
	float elec_vel;
	float elec_offset;
	float mech_offset;//机械零位
	float compSin;//机械角度正弦值
	float phase_order;
	/*校准*/
	uint8_t cali_start;
	uint8_t cali_finish;
}Encoder_t;

typedef struct _Torque_t
{
	float deform;
	float relative_encoder_pos;
	float relative_encoder2_pos;
	float current2torque;
	float deform2torque;
	//ratio
	float encoder_order_ratio;
	float reduction_ratio;
	float torque_current_ratio;
	float torque_deform_ratio;
}Torque_t;
/* ------------------------------ Variable Declarations ------------------------------ */
extern Encoder_t *p_encoder_g;
extern Encoder_t *p_encoder2_g;
extern Torque_t *p_torque_g;
extern const uint8_t vel_calc_step_period;
volatile extern float encoder_theta;
extern float test_H1;
extern float test_H2;
extern float test_H3;
extern float test_H4;
/* ------------------------------ Manager Typedef ------------------------------ */
//typedef struct EncoderManager
//{
///**
//  * @brief  Init
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
//  void (*Init)(void);

///**
//  * @brief  Sample
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
//  uint8_t (*Sample)(void);
//	
///**
//  * @brief  Calibrate
//  * @note   note.
//  * @param  param brief
//  * @retval None
//  */
//	void (*Calibrate)(void);
//}EncoderManager_typedef;
/* ------------------------------ Manager Extern ------------------------------ */
//extern const EncoderManager_typedef Encoder;
#endif
