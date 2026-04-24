#include <stdint.h>
#include <stdbool.h>
#include <math.h>
/**
 * @brief 扫频信号发生器状态结构体
 */
typedef struct {
    /* 用户设置参数 */
    float amplitude;      ///< 正弦波幅值 (A)
    float f_max;          ///< 最大扫频频率 (Hz)
    float sweep_time;     ///< 总扫频时间 (秒)
    float dt;             ///< 控制周期/采样时间 (秒)
    
    /* 内部状态变量 */
    float current_time;   ///< 当前已运行时间 (秒)
    float current_freq;   ///< 当前瞬时频率 (Hz)
    float current_phase;  ///< 当前相位 (弧度)
    bool is_active;       ///< 扫频是否进行中
    bool is_complete;     ///< 扫频是否完成
    
    /* 中断安全的状态缓存（主循环读取）*/
    float output_cache;   ///< 当前输出值缓存
    float time_cache;     ///< 时间缓存
    float freq_cache;     ///< 频率缓存
    
    /* 主循环数据记录 */
    volatile bool new_data_ready;  ///< 新数据就绪标志
    uint32_t sample_counter;      ///< 采样计数器（用于降采样打印）
} SweepSineGenerator;

void SweepSine_Init(SweepSineGenerator* gen, float amplitude, float f_max, float sweep_time, float dt);
void SweepSine_Start(SweepSineGenerator* gen);
float SweepSine_Update_ISR(SweepSineGenerator* gen);
void SweepSine_GetCurrentStatus(SweepSineGenerator* gen, float* output, float* freq, float* time);