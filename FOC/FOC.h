#ifndef __FOC_H
#define __FOC_H
#include <stdint.h>
#include "SweepSine.h"
extern char cmd_val[8];
extern char cmd_id;
extern char char_count;

extern float I_BW_set;
extern float I_SWOver_set;
extern float Motor_Iq_set;
extern float Motor_W_set;
extern float Motor_P_set;
extern float Velocity_P_set;
extern float Velocity_I_set;
extern float Position_P_set;
extern float Position_I_set;
extern float Current_P_set;
extern float Current_I_set;
extern float FOC_velAccDec_set;
extern char is_PPMode;
#define DWT_CR              *(volatile uint32_t *)0xE0001000
#define DWT_CYCCNT          *(volatile uint32_t *)0xE0001004
#define DEM_CR              *(volatile uint32_t *)0xE000EDFC
extern uint32_t ISR_start, ISR_end;
extern float ISR_time_us;
/* ------------------------------ Includes ------------------------------ */
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "stm32h7xx_it.h"

#include "FOC.h"
#include "calibration.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "encoderRaw.h"

#include "uart_printf.h"
#include "user_config.h"
#include "interact.h"
#include "can_rv.h"
#include "uart4_VOFA+.h"
#include "Diag.h"

#include "trajectory.h"
#include "PPtraj.h"




#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define HOMING_MODE         6

#define ADC2_CHANNELS 2
#define ADC2_CHANNELS_WINDOW 16
#define ADC2_CHANNELS_WINDOW_movebit 4

#define ADC_resolution 65535.0f
#define ADC_supply 3.3f
extern uint8_t caliOn_flag;
extern volatile uint16_t FSMstate;
extern uint16_t ADC_Temp_Value[2];
extern uint16_t ADC_Cur_vbus_Value[ADC2_CHANNELS*ADC2_CHANNELS_WINDOW];
extern uint8_t svpwm_on;
extern uint16_t Ta, Tb, Tc;
typedef struct
{
    int adc1_raw, adc2_raw, adc3_raw;                       // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus;                                            // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec;                         // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float v_d_ff, v_q_ff;                                   // D/Q feed forward voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float k_d, k_q, ki_d, ki_q, alpha;                      // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc1_offset, adc2_offset;                           // ADC offsets
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    float p_des, v_des, kp, kd, t_ff;                       // Desired position, velocity, gians, torque
    float v_ref;                                            // output voltage magnitude, field-weakening integral
} ControllerStruct;
extern ControllerStruct controller;

/* ------------------------------ Defines ------------------------------ */
#define PI_TIMES_2    6.2831853071795f
#define PI_BY_2       1.5707963267948f
#define SQRT3         1.7320508075688f
#define SQRT3_BY_2    0.8660254037844f
#define SQRT3_BY_3    0.5773502691896f
#define _1_BY_3 			0.3333333333333f
#define _2_BY_3 			0.6666666666667f
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)
#define clamp(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ADC_sample_time 400.0f; // remain some time for ADC sample

static void SVPWM(float alpha, float beta);
void ApplyVoltDQToSVPWM(float volt_d,float volt_q,float theta);

void ClarkTransform(volatile float *current_a, volatile float *current_b, volatile float *current_c, volatile float *current_alpha, volatile float *current_beta);
void InverseClarkTransform(volatile float *current_alpha, volatile float *current_beta, volatile float *current_a, volatile float *current_b, volatile float *current_c);
void ParkTransform(volatile float *current_alpha, volatile float *current_beta, volatile float theta, volatile float *current_d, volatile float *current_q);
void InverseParkTransform(volatile float *current_d, volatile float *current_q, volatile float theta, volatile float *current_alpha, volatile float *current_beta);
void init_controller_params(ControllerStruct *controller);
void Homing(void);
void PositionLoop(void);
void VelocityLoop(void);//速度环 PI运算得到q轴电流给定
void CurrentLoop(void);
void PD_FOC_clear(void);
void torque_control(ControllerStruct *controller);
void enablePWM(void);
void disablePWM(void);
void enablePre_drive(void);
void disablePre_drive(void);

float fmodf_pos(float x, float y);
float wrap_pm(float x, float y);
void limit_norm(float *x, float *y, float limit);
uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint32_to_float(uint32_t x_int, float x_min, float x_max, int bits);
float prvMod2PI(volatile float theta);
int encoderMod(const int dividend, const int divisor);
float Clamp(volatile float num, volatile float low, volatile float high);
float Mod(volatile float num, volatile float mod_range_min, volatile float mod_range_max);
uint16_t UintMod(volatile uint16_t output,volatile uint16_t mod_range_min,volatile uint16_t mod_range_max);
float IterationLn(volatile float a, int order);
float Uint2Float(volatile uint8_t *p_raw);
bool least_square_method(float *data, uint8_t num, float *K);
bool least_square_method_flux(float *xdata, float *ydata, uint8_t num, float *K);
void delay_us(uint16_t nus);
uint8_t calcCRC(uint8_t * buffer, uint8_t length);
uint8_t DWT_Init(void);
#define MCU_SYSCLK 480000000.0f		// Frequence of MCU[Hz]
#define PWM_PERIOD_DEFAULT 24000.0f //180,000,000/PWM_FREQUENCY_DEFAULT
#define PWM_FREQUENCY_DEFAULT	10000.0f		// Frequence of pwm[Hz]
#define PeriodPWM 0.0001f

#define current_compensation_ratio 1.00f
#define vel_calc_period 4  // 采集位置和解算速度的周期比值
#define pos_calc_period 4  // 采集位置和进位置环的周期比值
#define KT_OUT 1.48//3.5f            //KT*GR  模组测试得到
#endif

