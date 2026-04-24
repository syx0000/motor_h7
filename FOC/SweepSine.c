/**
 * @file    sweep_sine_generator.c
 * @brief   FOC电流环带宽测试 - 正弦扫频信号发生器
 * @details 生成一个频率从0线性增加到f_max的正弦波，用于测试电流环的频率响应。
 * 
 * 使用说明：
 * 1. 初始化结构体并设置参数
 * 2. 在每个控制周期调用SweepSine_Update()获取当前指令值
 * 3. 记录id_ref和id_fbk用于分析带宽
 */
#include "SweepSine.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // 定义NULL
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



/**
 * @brief 初始化扫频信号发生器
 * @param gen 发生器结构体指针
 */
void SweepSine_Init(SweepSineGenerator* gen, float amplitude, float f_max, float sweep_time, float dt)
{
    if (gen == NULL || f_max <= 0 || sweep_time <= 0 || dt <= 0)
	{
        return;
    }
    
    gen->amplitude = amplitude;
    gen->f_max = f_max;
    gen->sweep_time = sweep_time;
    gen->dt = dt;
    
    gen->current_time = 0.0f;
    gen->current_freq = 0.0f;
    gen->current_phase = 0.0f;
    gen->is_active = false;
    gen->is_complete = false;
    gen->output_cache = 0.0f;
    gen->time_cache = 0.0f;
    gen->freq_cache = 0.0f;
    gen->new_data_ready = false;
    gen->sample_counter = 0;
}

/**
 * @brief 启动扫频
 * @param gen 发生器结构体指针
 */
void SweepSine_Start(SweepSineGenerator* gen)
{
    if (gen == NULL) return;
    
    gen->current_time = 0.0f;
    gen->current_freq = 0.0f;
    gen->current_phase = 0.0f;
    gen->is_active = true;
    gen->is_complete = false;
    gen->output_cache = 0.0f;
    gen->new_data_ready = false;
    gen->sample_counter = 0;
}

/**
 * @brief 中断服务程序中调用 - 必须快速执行！
 * @param gen 发生器结构体指针
 * @return 当前id_ref指令值
 */
float SweepSine_Update_ISR(SweepSineGenerator* gen)
{
    if (gen == NULL || !gen->is_active)
	{
        return 0.0f;
    }
    
    // 检查扫频是否已完成
    if (gen->current_time >= gen->sweep_time)
	{
        gen->is_active = false;
        gen->is_complete = true;
        gen->output_cache = 0.0f;
        return 0.0f;
    }
    
    // 计算当前频率（线性扫频）
    gen->current_freq = (gen->f_max / gen->sweep_time) * gen->current_time;
    
    // 计算当前相位增量并积分
    float phase_increment = 2.0f * M_PI * gen->current_freq * gen->dt;
    gen->current_phase += phase_increment;
    
    // 保持相位在0~2π范围内
    if (gen->current_phase > 2.0f * M_PI)
	{
        gen->current_phase -= 2.0f * M_PI;
    }
    
    // 计算正弦输出
    float output = gen->amplitude * sinf(gen->current_phase);
    gen->output_cache = output;
    
    // 更新时间
    gen->current_time += gen->dt;
    
    // 每N个周期标记一次数据就绪（用于主循环打印）
    if (gen->sample_counter++ >= 100)// 10kHz中断，100次=10ms
	{
        gen->time_cache = gen->current_time;
        gen->freq_cache = gen->current_freq;
        gen->new_data_ready = true;
        gen->sample_counter = 0;
    }
    
    return output;
}

/**
 * @brief 获取当前状态（主循环中安全调用）
 * @param gen 发生器结构体指针
 * @param output 输出电流指令
 * @param freq 当前频率
 * @param time 当前时间
 */
void SweepSine_GetCurrentStatus(SweepSineGenerator* gen, float* output, float* freq, float* time)
{
    if (gen == NULL) return;
    
    if (output) *output = gen->output_cache;
    if (freq) *freq = gen->freq_cache;
    if (time) *time = gen->time_cache;
}