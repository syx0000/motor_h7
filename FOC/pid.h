/**
  ******************************************************************************
  * @file           : pid.h
  * @author         : Mark Xiang
  * @brief          : PID controller
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */

/* ------------------------------ Enum Typedef ------------------------------ */

/* ------------------------------ Struct Typedef ------------------------------ */
typedef struct _pid_t
{
	/* coefficients & parameters to be read from flash */
	float kp, ki, kd;
	volatile float P, I, D;
	float integtal_limit, output_limit;
	float deadband;
	volatile float feedforward;
	float feedforward_ratio;
	/* working variables */
	float target;
	volatile float feedback;
	volatile float err;
	float err_last;

	/* output */
	float output;

	float targetend;
}pid_t;

/* ------------------------------ Variable Declarations ------------------------------ */
extern pid_t *p_D_current_loop_g , *p_Q_current_loop_g , *p_velocity_loop_g ,*p_position_loop_g, *p_position_loop2_g;
/* ------------------------------ Manager Typedef ------------------------------ */
typedef struct PidManager
{
/**
  * @brief  Initiate pid struct.
  * @param  pid pointer to pid struct
  * @param  kp kp
  * @param  ki ki
  * @param  kd kd
  * @param  output_limit limit of output of pid controller
  * @param  integtal_limit limit of output increment of pid controller
  * @retval None
  */
	void (*Init)(pid_t *pid, float kp, float ki, float kd, float output_limit, float deadband, float feedforward_ratio);

/**
  * @brief  Change kp.
  * @param  pid pointer to pid struct
  * @param  kp new kp
  * @retval None
  */
	void (*ResetKp)(pid_t *pid, float kp);

/**
  * @brief  Change ki.
  * @param  pid pointer to pid struct
  * @param  ki new ki
  * @retval None
  */
	void (*ResetKi)(pid_t *pid, float ki);

/**
  * @brief  Change kd.
  * @param  pid pointer to pid struct
  * @param  kd new kd
  * @retval None
  */
	void (*ResetKd)(pid_t *pid, float kd);

/**
  * @brief  Set target of pid controller.
  * @param  pid pointer to pid struct
  * @param  target target of pid controller
  * @retval None
  */
	void (*SetTarget)(pid_t *pid, float target);

/**
  * @brief  Do pid calculation.
  * @param  pid pointer to pid struct
  * @param  feedback feedback to pid controller
  * @retval output of pid controller
  */
	float (*DoPidCalc)(pid_t *pid, float feedback);
}PidManager_typedef;
/* ------------------------------ Manager Extern ------------------------------ */
extern const PidManager_typedef Pid;


#endif

