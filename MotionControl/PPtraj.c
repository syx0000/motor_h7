/*PP模式 位置规划改进版本*/
#include <math.h>
#include <FOC.h>
#include <stdbool.h>
#include "main.h"
#include "PPtraj.h"
uint8_t trajcplt=0;


MotionPlanner planner_s;
MotionPlanner *p_planner_s = &planner_s;
void init_planner(MotionPlanner* planner, float start_pos, float target,  float vmax, float acc, float dec);
bool get_next_position(MotionPlanner* planner, float dt, float* position);

// 初始化运动规划器
void init_planner(MotionPlanner* planner, float start_pos, float target, float vmax, float acc, float dec)//确定位置规划曲线类型（三角形或梯形），确定加速、减速、匀速时间
{
    // 保留当前位置
									 
	float start_vel = planner->current_velocity;
    planner->current_position = start_pos;
    planner->current_velocity = start_vel;
    planner->target_position = target;
    planner->elapsed_time = 0.0f;//运行时间清零

    // 计算实际运动参数
    const float epsilon = 1e-6f;
    const float displacement = target - start_pos;
    const float abs_displacement = fabsf(displacement);
    
    // 立即到达判断
    if (abs_displacement < epsilon)
	{
		planner->state = STOP;
		planner->phase_time_acc = 0.0f;
		planner->phase_time_cruise = 0.0f; // 无匀速阶段
		planner->phase_time_dec = 0.0f;
		planner->total_time = 0.0f;
		return;
	}

	// 计算运动方向
	const float sign_dir = (displacement >= 0) ? 1.0f : -1.0f;
		
	float effective_acc = (acc * dec) / (acc + dec);
//	float max_feasible_v = sqrtf(2 * effective_acc * abs_displacement + start_vel * start_vel);
	float max_feasible_v = sqrtf(2 * effective_acc * abs_displacement);
		
		
//	vmax = fminf(vmax, abs_displacement); // 防止速度过大导致无法停止
	vmax = fminf(vmax, max_feasible_v); // 防止速度过大导致无法停止

	// 设置方向参数
	planner->max_velocity = vmax * sign_dir;
	planner->acceleration = acc * sign_dir;
	planner->deceleration = dec * (-sign_dir);

	// 计算阶段参数
	const float t_acc = fabsf(planner->max_velocity / planner->acceleration);//加速时间
	const float t_dec = fabsf(planner->max_velocity / planner->deceleration);//减速时间
	const float s_acc = 0.5f * planner->acceleration * t_acc * t_acc;//加速路程
	const float s_dec = 0.5f * planner->deceleration * t_dec * t_dec;//减速路程
//	float t_acc = fabsf((planner->max_velocity - start_vel) / planner->acceleration);
//	float t_dec = fabsf(planner->max_velocity / planner->deceleration);
//	float s_acc = start_vel * t_acc + 0.5f * planner->acceleration * t_acc * t_acc;
//	float s_dec = 0.5f * planner->deceleration * t_dec * t_dec;
//	if ((s_acc + s_dec) <= abs_displacement) {
//		float s_cruise = abs_displacement - (s_acc + s_dec);
//		float t_cruise = s_cruise / fabsf(planner->max_velocity);
//        
//		planner->phase_time_acc = t_acc;
//		planner->phase_time_cruise = t_acc + t_cruise;
//		planner->phase_time_dec = planner->phase_time_cruise + t_dec;
//	} else {
//		// 三角形曲线，考虑初始速度
//		float v_peak_dir = (displacement >= 0) ? 1.0f : -1.0f;
//		float v_peak_sq = (2 * effective_acc * abs_displacement) + start_vel * start_vel;
//		float v_peak = v_peak_dir * sqrtf(fabsf(v_peak_sq));
//        
//		planner->max_velocity = v_peak;
//		t_acc = fabsf((v_peak - start_vel) / planner->acceleration);
//		t_dec = fabsf(v_peak / planner->deceleration);
//        
//		planner->phase_time_acc = t_acc;
//		planner->phase_time_cruise = t_acc; // 无匀速阶段
//		planner->phase_time_dec = t_acc + t_dec;
//	}
		
	// 判断运动曲线类型
	if ((fabsf(s_acc) + fabsf(s_dec)) <= abs_displacement) { // 梯形曲线
		const float s_cruise = abs_displacement - (fabsf(s_acc) + fabsf(s_dec));//匀速路程
		const float t_cruise = s_cruise / vmax;//匀速时间

		planner->phase_time_acc = t_acc;//加速时间点
		planner->phase_time_cruise = t_acc + t_cruise;//匀速时间点
		planner->phase_time_dec = planner->phase_time_cruise + t_dec;//减速时间点
	} else { // 三角形曲线
		const float effective_acc = (acc * dec) / (acc + dec);
		const float v_peak = sign_dir * sqrtf(2 * effective_acc * abs_displacement);
		
		planner->max_velocity = v_peak;
		planner->phase_time_acc = fabsf(v_peak / planner->acceleration);
		planner->phase_time_cruise = planner->phase_time_acc; // 无匀速阶段
		planner->phase_time_dec = planner->phase_time_acc + fabsf(v_peak / planner->deceleration);
	}

	planner->total_time = planner->phase_time_dec;
	planner->state = ACCEL;
}




// 获取下一个规划位置（时间步长建议0.001-0.1秒 0.002s）
bool get_next_position(MotionPlanner* planner, float dt, float* position)
{
	if (planner->state == STOP)
	{
		*position = planner->target_position;
		return false;
	}
	float temp_current_pos = planner->current_position;
	float t_remaining = dt;
	if (t_remaining > 1e-6f && planner->state != STOP)
	{
		float phase_remaining = 0.0f;
		float delta_t = 0.0f;

		switch (planner->state)
		{
			case ACCEL://加速
			{
				phase_remaining = planner->phase_time_acc - planner->elapsed_time;
				delta_t = fminf(phase_remaining, t_remaining);
				
				float a = planner->acceleration;
				planner->current_position = temp_current_pos + planner->current_velocity * delta_t + 0.5f * a * delta_t * delta_t;
				planner->current_velocity += a * delta_t;
                
                // 速度限幅（考虑方向）
				if ((a > 0 && planner->current_velocity > planner->max_velocity) ||
					(a < 0 && planner->current_velocity < planner->max_velocity)) {
					planner->current_velocity = planner->max_velocity;
				}
				break;
			}
            
			case CRUISE: {//匀速
				phase_remaining = planner->phase_time_cruise - planner->elapsed_time;
				delta_t = fminf(phase_remaining, t_remaining);
				
				planner->current_position = temp_current_pos + planner->current_velocity * delta_t;
				break;
			}
            
			case DECEL: {//减速
				phase_remaining = planner->phase_time_dec - planner->elapsed_time;
				delta_t = fminf(phase_remaining, t_remaining);
				
				float d = planner->deceleration;
				planner->current_position = temp_current_pos + planner->current_velocity * delta_t + 0.5f * d * delta_t * delta_t;
				planner->current_velocity += d * delta_t;
				
				// 速度归零保护
				if ((d > 0 && planner->current_velocity > 0) || 
					(d < 0 && planner->current_velocity < 0)) {
					planner->current_velocity = 0;
				}
				break;
			}
			default:
				break;
		}

		// 更新时间参数
		planner->elapsed_time += delta_t;//已经运行的时间
		t_remaining -= delta_t;

		// 状态转移检查
		if (planner->state == ACCEL && planner->elapsed_time >= planner->phase_time_acc)
		{
			planner->state = (planner->phase_time_cruise > planner->phase_time_acc) ? CRUISE : DECEL;
		}
		if (planner->state == CRUISE && planner->elapsed_time >= planner->phase_time_cruise)
		{
			planner->state = DECEL;
		}
		if (planner->state == DECEL && planner->elapsed_time >= planner->phase_time_dec)
		{
			planner->state = STOP;
			planner->current_position = planner->target_position;
		}
	}

//	// 最终位置校正
//	if (signbit(planner->target_position) != signbit(planner->current_position)) {
//		planner->current_position = planner->target_position;
//	} else if (fabsf(planner->current_position) > fabsf(planner->target_position)) {
//		planner->current_position = planner->target_position;
//	}

	*position = planner->current_position;
	return planner->state != STOP;
}

/* 使用示例：
// 正向运动
MotionPlanner pos_planner;
init_planner(&pos_planner, 100.0f, 10.0f, 2.0f, 2.0f);

// 反向运动
MotionPlanner neg_planner;
init_planner(&neg_planner, -50.0f, 8.0f, 3.0f, 3.0f);

float position;
while (get_next_position(&neg_planner, 0.1f, &position)) {
    printf("Current position: %.2f\n", position);
}
*/


