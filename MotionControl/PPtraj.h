#ifndef __PPTRAJ_H
#define __PPTRAJ_H

#include <math.h>
#include <stdbool.h>
#include "main.h"


typedef struct {
	float current_position;   // 当前位置
	float current_velocity;  // 当前速度
	float acceleration;      // 实际加速度（带方向）
	float deceleration;      // 实际减速度（带方向）
	float target_position;   // 目标位置
	float max_velocity;      // 带方向的最大速度
	enum {ACCEL, CRUISE, DECEL, STOP} state; // 运动状态
	float phase_time_acc;    // 加速阶段结束时间
	float phase_time_cruise; // 匀速阶段结束时间
	float phase_time_dec;    // 减速阶段结束时间
	float total_time;        // 总运动时间
	float elapsed_time;      // 已用时间
} MotionPlanner;
extern void init_planner(MotionPlanner* planner, float start_pos, float target,  float vmax, float acc, float dec);
extern bool get_next_position(MotionPlanner* planner, float dt, float* position);
extern MotionPlanner *p_planner_s;



#define POSLOOP_FREQUENCY_DEFAULT	2500.0f		// Frequence of pwm[Hz]
#define NUM_STEPS 5
#define STEP_PANNEL 1.0f/POSLOOP_FREQUENCY_DEFAULT * NUM_STEPS
extern uint8_t trajcplt;

#endif


