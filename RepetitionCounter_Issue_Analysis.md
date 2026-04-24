# RepetitionCounter 配置分析

## 结论

TIM1 配置 `RepetitionCounter = 1` 在中心对齐模式下是**正确的**，实际中断频率为 10kHz，与代码假设一致。

---

## 当前配置

### TIM1 配置 (tim.c:48-54)

```c
htim1.Init.Prescaler = 0;
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
htim1.Init.Period = 11999;
htim1.Init.RepetitionCounter = 1;  // ✓ 正确配置
```

### 代码假设 (FOC.h:155-156)

```c
#define PWM_FREQUENCY_DEFAULT  10000.0f  // 10 kHz ✓
#define PeriodPWM              0.0001f   // 100 us ✓
```

---

## 理论分析

### PWM 频率计算

- 定时器时钟 = APB2 × 2 = 120 MHz × 2 = **240 MHz**
- 中心对齐模式：一个完整周期 = 上计数(0→11999) + 下计数(11999→0) = 24000 个时钟
- **PWM 频率 = 240 MHz / 24000 = 10 kHz** ✓

### RepetitionCounter 在中心对齐模式下的作用

在中心对齐模式下，RepetitionCounter 在每次**上溢（overflow）和下溢（underflow）**时都会递减：

- 每个 PWM 周期有 **2 次计数事件**（上溢 + 下溢）
- **RCR = 1**: 每 (RCR+1) = **2 次事件**产生一次更新中断
- **中断频率 = 2 × 10kHz / 2 = 10 kHz** ✓

这意味着每个 PWM 周期产生一次中断，与代码假设完全一致。

### 与边沿对齐模式的区别

**边沿对齐模式**（向上计数）：
- 每个 PWM 周期只有 **1 次计数事件**（上溢）
- RCR = 1 → 每 2 次上溢产生一次中断 → ISR频率 = PWM频率 / 2

**中心对齐模式**（上下计数）：
- 每个 PWM 周期有 **2 次计数事件**（上溢 + 下溢）
- RCR = 1 → 每 2 次事件产生一次中断 → ISR频率 = PWM频率

---

## 验证结果

当前配置：
- **实际中断频率 = 10 kHz** ✓
- **实际中断周期 = 100 us** ✓
- 与代码假设 `PWM_FREQUENCY_DEFAULT = 10000` 完全一致 ✓

### 时间标志正确性

```c
// stm32h7xx_it.c:292-299
u8_100usFlag = 1;  // ✓ 正确，每 100us 设置一次

u32Timecnt++;
if(u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)  // 10000/1000 = 10
{
    u32Timecnt = 0;
    u8_1msFlag = 1;  // ✓ 正确，每 10 × 100us = 1ms 设置一次
}
```

### 控制环频率正确性

```c
// FOC.h:159-160
#define vel_calc_period 4  // 速度环分频
#define pos_calc_period 4  // 位置环分频
```

**实际频率：**
- 电流环：**10 kHz** ✓
- 速度环：10 kHz / 4 = **2.5 kHz** ✓
- 位置环：10 kHz / 4 = **2.5 kHz** ✓

### PI 参数计算正确性

```c
// stm32h7xx_it.c:692-693
controller.ki_d = I_BW * 2.0f * PI * p_motor_g->phase_resistance * (1/PWM_FREQUENCY_DEFAULT);
controller.ki_q = I_BW * 2.0f * PI * p_motor_g->phase_resistance * (1/PWM_FREQUENCY_DEFAULT);
```

使用 `1/PWM_FREQUENCY_DEFAULT = 1/10000 = 0.0001`，与实际采样周期 100us 一致 ✓

### 速度计算正确性

```c
// encoder.c:165
p_encoder_g->mech_vel = delta_encoder_cnt_M * PI_TIMES_2 * PWM_FREQUENCY_DEFAULT 
                        / p_encoder_g->cpr / (float)vel_calc_period;
```

使用 `PWM_FREQUENCY_DEFAULT = 10000`，与实际中断频率 10kHz 一致 ✓

---

## 总结

**当前配置完全正确：**
- 硬件配置：10 kHz 中断（RCR=1 在中心对齐模式下）
- 软件假设：10 kHz 中断（PWM_FREQUENCY_DEFAULT=10000）
- **硬件与软件完全一致** ✓

所有时间相关的计算都是正确的：
- 时间标志（100us, 1ms）✓
- 控制环频率（10k/2.5k/2.5k）✓
- PI 参数计算 ✓
- 速度/加速度计算 ✓

**无需修改任何配置。**
