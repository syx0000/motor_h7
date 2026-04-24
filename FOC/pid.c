/**
  ******************************************************************************
  * @file           : pid.c
  * @author         : Peilin Tao
  * @brief          : Discrete PID controller with output limit & output increment limit.
  ******************************************************************************
  * @attention
  *             1. This file has dependency on the definition of MAX(a,b) and MIN(a,b)
  * 
  ******************************************************************************
  */

/* ------------------------------ Includes ------------------------------ */
#include "pid.h"
#include <FOC.h>
/* ------------------------------ Defines ------------------------------ */

/* ------------------------------ Private Function Declarations ------------------------------ */

/* ------------------------------ Public Function Declarations ------------------------------ */
static void Init(pid_t *pid, float kp, float ki, float kd, float output_limit, float deadband, float feedforward_ratio);
static void ResetKp(pid_t *pid, float kp);
static void ResetKi(pid_t *pid, float ki);
static void ResetKd(pid_t *pid, float kd);
static void SetTarget(pid_t *pid, float target);
static float DoPidCalc(pid_t *pid, float feedback);
/* ------------------------------ Variable Declarations ------------------------------ */
pid_t D_current_loop,Q_current_loop,velocity_loop,position_loop,position_loop2;
pid_t *p_D_current_loop_g = &D_current_loop, *p_Q_current_loop_g = &Q_current_loop, *p_velocity_loop_g = &velocity_loop, *p_position_loop_g = &position_loop, *p_position_loop2_g= &position_loop2;
/* ------------------------------ Private ------------------------------ */

/* ------------------------------ Public ------------------------------ */
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
static void Init(pid_t *pid, float kp, float ki, float kd, float output_limit, float deadband, float feedforward_ratio)
{
	/* init variables as 0 */
	memset(pid, 0, sizeof(*pid));

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->deadband = deadband;
	pid->output_limit = output_limit;
	pid->feedforward_ratio = feedforward_ratio;
}

/**
  * @brief  Change kp.
  * @param  pid pointer to pid struct
  * @param  kp new kp
  * @retval None
  */
static void ResetKp(pid_t *pid, float kp)
{
  pid->kp = kp;
}

/**
  * @brief  Change ki.
  * @param  pid pointer to pid struct
  * @param  ki new ki
  * @retval None
  */
static void ResetKi(pid_t *pid, float ki)
{
  pid->ki = ki;
}

/**
  * @brief  Change kd.
  * @param  pid pointer to pid struct
  * @param  kd new kd
  * @retval None
  */
static void ResetKd(pid_t *pid, float kd)
{
  pid->kd = kd;
}

/**
  * @brief  Set target of pid controller.
  * @param  pid pointer to pid struct
  * @param  target target of pid controller
  * @retval None
  */
static void SetTarget(pid_t *pid, float target)
{
	pid->target = target;
}

/**
  * @brief  Do pid calculation.
  * @param  pid pointer to pid struct
  * @param  feedback feedback to pid controller
  * @retval output of pid controller
  */
//static float DoPidCalc(pid_t *pid, float feedback)
//{

//				    pid->err = pid->target - feedback;
//						pid->P = pid->kp * pid->err;//比例系数×误差
//						pid->output = pid->P + pid->feedforward * pid->feedforward_ratio;
//						if (fabsf(pid->output) > pid->output_limit) pid->I = 0.0f;//如果比例部分已经大于限幅，则将积分项设为0
//						else
//						{	
//							pid->I += pid->ki * pid->err;//积分系数×误差（一直累积）
//							/*也单独对积分部分做一个限幅，对最终的输出效果output没影响*/
//							pid->I = MIN(pid->I, pid->output_limit - pid->output);
//							pid->I = MAX(pid->I, -pid->output_limit - pid->output);
//						}
//						pid->output += pid->I;
//						pid->output = MIN(pid->output, pid->output_limit);//|pid->output|<pid->output_limit
//						pid->output = MAX(pid->output, -pid->output_limit);

//			
//  return pid->output;
//}

static float DoPidCalc(pid_t *pid, float feedback)
{
	pid->err = pid->target - feedback;
	if ((fabsf(pid->err)<pid->deadband)&&(p_motor_g->controlMode == FOC_POSITION_LOOP))//位置环添加死区
	{
		pid->output = 0.0f;
		return pid->output;
	}
//	if (fabsf(pid->err)>pid->deadband)//添加死区
	{
		pid->P = pid->kp * pid->err;//比例系数×误差
		pid->output = pid->P + pid->feedforward * pid->feedforward_ratio;

		// 积分饱和冻结策略：新积分值导致输出饱和时停止累积
		float i_new = pid->I + pid->ki * pid->err;
		if (fabsf(pid->output + i_new) < pid->output_limit)
		{
			pid->I = i_new;
		}
		// 积分限幅
		pid->I = MIN(pid->I, pid->output_limit);
		pid->I = MAX(pid->I, -pid->output_limit);

		pid->output += pid->I;
		pid->output = MIN(pid->output, pid->output_limit);//|pid->output|<pid->output_limit
		pid->output = MAX(pid->output, -pid->output_limit);
	}
//	else pid->output = 0;
	
  return pid->output;
}
/* ------------------------------ Manager Declaration ------------------------------ */
const PidManager_typedef Pid =
{
	Init,
	ResetKp,
	ResetKi,
	ResetKd,
	SetTarget,
	DoPidCalc
};

