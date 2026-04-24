# FOC Algorithm & Three-Loop Control Review

**Project:** FIVE (STM32H743VITx FOC Motor Driver)
**Date:** 2026-04-21
**Scope:** FOC坐标变换、电流环PI、速度环、位置环级联、电角度计算、校准流程

---

## 1. 系统拓扑

```
[CAN/UART] -> 位置环 -> 速度环 -> 电流环 -> SVPWM -> 逆变器 -> PMSM -> 减速器(GR=50) -> 负载
                |          |         |                              |                      |
           encoder2     encoder1   ADC                         encoder1               encoder2
           (输出端)     (电机端)   (三相电流)                   (电机端)               (输出端)

双编码器:
  encoder1 (angleOutter) -> 电机端, FOC电角度 + 速度反馈
  encoder2 (angleInner)  -> 输出端(减速器后), 位置反馈
  CPR = 16777216 (24-bit), pole_pairs = 8
```

---

## 2. 电角度计算与偏移

### 2.1 运行时电角度公式

```c
// encoder.c:116
elec_pos = (mech_pos / cpr) * pole_pairs * 2pi - elec_offset
```

**值域分析:**
- `mech_pos`: [0, 16777215]
- `(mech_pos/cpr) * 8 * 2π`: [0, 16π] ≈ 50.27 rad
- 减去 `elec_offset` 后: [-offset, 16π - offset]

### 问题 F-01 [致命]: 电角度未归一化到 [0, 2π]

`elec_pos` 可能远超 2π，后续 `CurrentLoop()` 中直接使用:

```c
// FOC.c:295-296
sin_theta = arm_sin_f32(p_encoder_g->elec_pos);  // 输入可能 > 2π!
cos_theta = arm_cos_f32(p_encoder_g->elec_pos);
```

`arm_sin_f32` 内部虽会归一化，但对大角度值精度下降。而 `ApplyVoltDQToSVPWM` 中调用了 `prvMod2PI(theta)` 做归一化，但 `CurrentLoop` 内联Park变换跳过了这一步。

**修复:**
```c
p_encoder_g->elec_pos = fmodf_pos(
    (float)mech_pos * one_div_cpr * pole_pairs * PI_TIMES_2 - elec_offset,
    PI_TIMES_2);
```

### 2.2 校准偏移 (elec_offset)

**校准流程 (calibration.c):**
1. 施加D轴电压 (theta_ref=0)，转子对齐
2. 正向旋转 theta_ref: 0 → 2π×pole_pairs (一圈机械)
   记录 error_f[i] = θ_actual - θ_ref/pole_pairs
3. 反向旋转，记录 error_b[i]
4. offset = average(error_f + error_b)

**offset含义:** 编码器零点对应的电角度，用于将机械位置映射到正确电角度。

### 问题 F-02 [严重]: 校准期间ISR与校准代码竞争

校准在主循环中用 `HAL_Delay` 阻塞，但TIM1 ISR仍在运行:

```c
// calibration.c:106-109
ApplyVoltDQToSVPWM(v_d, v_q, theta_ref);
HAL_Delay(1);
encoderSample();  // 手动调用
theta_actual = p_encoder_g->pos_abs;

// 同时 TIM1 ISR 也在调用:
encoderSample();  // ISR中调用
```

两处同时修改 `mech_pos`, `rotations`, `pos_abs`，可能导致 `rotations` 双重更新。

**修复:** 校准期间禁用TIM1中断或设置标志跳过ISR中的 `encoderSample()`。

---

## 3. Clarke/Park 变换正确性

### 3.1 Clarke 变换

```c
// FOC.c:189-190
Iα = (2/3)*Ia - (1/3)*(Ib + Ic)
Iβ = (√3/3)*(Ib - Ic)
```

**标准等幅值Clarke:**
```
Iα = (2/3)[Ia - (1/2)Ib - (1/2)Ic] = (2/3)Ia - (1/3)(Ib+Ic) ✓
Iβ = (2/3)[(√3/2)Ib - (√3/2)Ic] = (√3/3)(Ib-Ic) ✓
```

**结论: 正确。**

### 3.2 Park 变换

```c
// FOC.c:298-299 (CurrentLoop内联)
Id =  cos(θ)*Iα + sin(θ)*Iβ   ✓
Iq = -sin(θ)*Iα + cos(θ)*Iβ   ✓

// FOC.c:343 (反Park内联)
Vα = cos(θ)*Vd - sin(θ)*Vq   ✓
Vβ = sin(θ)*Vd + cos(θ)*Vq   ✓
```

**结论: 正确。**

---

## 4. 电流环 PI 控制器

### 4.1 结构

```c
// FOC.c:308-339
i_d_error = i_d_ref - D_axis_current;
i_q_error = i_q_ref - Q_axis_current;

d_int += ki_d * i_d_error;
q_int += ki_d * i_q_error;     // BUG: 应为 ki_q

// 积分限幅 (标量独立)
d_int = clamp(d_int, -1.15*Vbus, 1.15*Vbus);
q_int = clamp(q_int, -1.15*Vbus, 1.15*Vbus);

// PI输出
v_d = k_d * i_d_error + d_int;
v_q = k_d * i_q_error + q_int;   // BUG: 应为 k_q

// 电压矢量限幅
limit_norm(&v_d, &v_q, 1.15*Vbus);
```

### 问题 F-03 [严重]: Q轴PI使用D轴增益

```c
controller.q_int += controller.ki_d * i_q_error;  // 应为 ki_q
controller.v_q = controller.k_d * i_q_error + q_int;  // 应为 k_q
```

当前 `k_d == k_q` 且 `ki_d == ki_q`，实际不影响。但代码逻辑错误，未来独立调节dq增益时会出错。

### 问题 F-04 [中等]: 积分限幅方式不标准

积分项用标量限幅，输出用向量限幅。导致:
- d_int 和 q_int 各自可达 1.15*Vbus
- 积分向量最大 = √2 × 1.15*Vbus ≈ 1.63*Vbus
- 加比例项后可能远超限制，被 `limit_norm` 截断 → 向量方向偏移

**建议:** 积分项也用向量限幅:
```c
limit_norm(&d_int, &q_int, OVERMODULATION * Vbus);
```

### 问题 F-05 [中等]: 前馈补偿被注释

```c
// FOC.c:328-329 (计算了但未使用)
v_d_ff = -ω*Np*L*iq_ref;          // d轴解耦
v_q_ff = ω*Np*L*id_ref + 0.005619*ω*Np;  // q轴 + 反电动势

// FOC.c:336-337 (注释掉)
// v_d += v_d_ff;
// v_q += v_q_ff;
```

**影响:**
- 无前馈时，dq耦合完全靠PI消除
- 高速时耦合项大 (∝ω)，PI跟不上 → 电流畸变
- 反电动势不补偿 → 高速需更大q轴电压裕度

**注:** `0.005619` 是反电动势常数λf，硬编码 → 换电机需改

---

## 5. 速度环分析

### 5.1 结构

```c
// FOC.c:259-288
void VelocityLoop(void) {
    // 1. 速度斜坡 (仅FOC_VELOCITY_LOOP)
    if(controlMode == FOC_VELOCITY_LOOP) {
        max_step = |FOC_velAccDec| * vel_calc_period / PWM_FREQ;
        target += clamp(targetend - target, -max_step, max_step);
    }

    // 2. PI计算
    Pid.DoPidCalc(p_velocity_loop_g, p_encoder_g->mech_vel);

    // 3. 输出
    i_q_ref = velocity_loop.output;
    i_d_ref = 0;
}
```

### 问题 F-06 [严重]: 位置/速度反馈编码器不同

```
位置环: feedback = p_encoder2_g->pos_abs  (输出端)
速度环: feedback = p_encoder_g->mech_vel   (电机端)
```

```c
// FOC.c:255
Pid.DoPidCalc(p_position_loop_g, p_encoder2_g->pos_abs);  // 输出端

// FOC.c:282
Pid.DoPidCalc(p_velocity_loop_g, p_encoder_g->mech_vel);  // 电机端
```

**问题:** 位置环输出是输出端期望速度，速度环反馈是电机端速度，差减速比GR=50:

```
位置环输出: ω_output (输出端 rad/s)
速度环反馈: ω_motor (电机端 rad/s)
关系: ω_motor = ω_output × GR
```

速度环 `target` 与 `feedback` 单位不一致！

**验证:** 位置环 `output_limit=150` (main.c:162)，如果单位是输出端rad/s:
- target = 1.0 (输出端)
- feedback = 50 (电机端，GR=50)
- error = 1.0 - 50 = -49 → 反向扭矩

**但代码实际运行正常**，说明位置环Kp隐含了单位转换或通过 `output_limit` 限制了范围。

**建议:** 显式乘减速比:
```c
Pid.SetTarget(p_velocity_loop_g, p_position_loop_g->output * GR);
```

### ~~问题 F-07~~ [已确认正确]: 速度计算公式中PWM_FREQ正确

```c
// encoder.c:165
mech_vel = delta_cnt * 2π * PWM_FREQUENCY_DEFAULT / cpr / vel_calc_period;
```

ISR实际为10kHz（中心对齐模式RCR=1），与PWM_FREQUENCY_DEFAULT=10000一致，速度计算正确。

### 问题 F-08 [中等]: 滑窗滤波引入延迟

```c
// encoder.c:167-175  v_window_N = 2
mech_vel = (当前值 + 上一值) / 2
```

N=2延迟 = 0.5采样周期 = 0.5×4×100us = 200us，影响小。

但encoder2 `v_window_N_Inner = 10`，延迟 = 4.5×4×100us = **1.8ms**，对速度环响应有明显影响。

---

## 6. 位置环分析

### 6.1 结构

```c
// FOC.c:251-257
void PositionLoop(void) {
    Pid.DoPidCalc(p_position_loop_g, p_encoder2_g->pos_abs);
    Pid.SetTarget(p_velocity_loop_g, output);
}

// main.c:162
Pid.Init(p_position_loop_g, 300, 0.0000001, 0, 150, 0.0004, 1);
// Kp=300, Ki≈0, Kd=0, limit=150, deadband=0.0004rad
```

### 问题 F-09 [中等]: 死区仅对FOC_POSITION_LOOP生效

```c
// pid.c:132
if(fabsf(err) < deadband && controlMode == FOC_POSITION_LOOP) {
    output = 0;
    return;
}
```

**问题:**
1. PID通用模块硬编码检查 `controlMode`
2. PP模式 (`FOC_POSITION_LOOP_PP`) 不触发死区 → 可能振荡
3. 违反单一职责

### 问题 F-10 [严重]: 积分清零策略导致抖动

```c
// pid.c:141
if(fabsf(output) > output_limit)
    I = 0;  // 输出超限 → 积分清零
```

当位置误差大时:
1. P项大 → output超限 → I清零
2. 接近目标 → P减小 → output<limit → I开始累加
3. I从零开始需时间建立 → 稳态误差消除慢
4. 负载扰动可能再次清零I

**建议:** 冻结积分 (不清零):
```c
if(fabsf(output) > output_limit) {
    // 不改变I
} else {
    I += ki * err;
}
```

---

## 7. 三环级联时序

### 7.1 执行频率

```
TIM1 ISR (每周期):
  encoderSample()  → 更新位置/速度标志
  currentSample()  → 更新电流
  状态机:
    if(pos_loop_flag) → PositionLoop()
    if(vel_loop_flag) → VelocityLoop()
    CurrentLoop()  (每次)
```

| 控制环 | 频率 (ISR=10kHz) |
|--------|-----------------|
| 电流环 | 每ISR = 10kHz |
| 速度环 | 每4 ISR = 2.5kHz |
| 位置环 | 每4 ISR = 2.5kHz |

### 问题 F-11 [中等]: 位置/速度环同频率

```c
// encoder.c:137-151
vel_calc_period = 4;  // 速度环
pos_calc_period = 4;  // 位置环 (同频!)
```

经典级联控制中，外环应比内环慢3-10倍:

```
推荐: 电流10k → 速度2.5k → 位置500Hz
当前: 电流10k → 速度2.5k → 位置2.5k (同频!)
```

**建议:**
```c
#define pos_calc_period 16  // 位置环625Hz
```

### 问题 F-12 [信息]: 位置/速度环同ISR顺序执行

```c
// stm32h7xx_it.c:433-447
if(pos_loop_flag) PositionLoop();  // 设置速度target
if(vel_loop_flag) VelocityLoop();  // 立即使用新target
CurrentLoop();
```

当两标志同时为1时，速度环立即使用位置环刚设的target。这是"同步更新"，控制理论上可接受，不是bug。

---

## 8. 电流采样与相序

### 8.1 相序一致性

```c
// motor.c:319-335
if(phase_order == POSITIVE) {
    phase_b = volt2amp * CurrentB_Raw / 10 / R;
    phase_c = volt2amp * CurrentC_Raw / 10 / R;
} else {  // NEGATIVE
    phase_b = volt2amp * CurrentC_Raw / 10 / R;  // B←C
    phase_c = volt2amp * CurrentB_Raw / 10 / R;  // C←B
}
phase_a = -phase_b - phase_c;

// FOC.c:156-167 SVPWM
if(phase_order == POSITIVE) {
    CH1=Ta, CH2=Tb, CH3=Tc;
} else {
    CH1=Ta, CH2=Tc, CH3=Tb;  // 交换B/C
}
```

**分析:** 反向相序时，SVPWM和电流采样都交换B/C，保持一致性。

**结论: 正确。**

### 问题 F-13 [中等]: A相电流非直接采样

```c
phase_a = -phase_b - phase_c;
```

A相由-(B+C)计算，标准两电阻方案。但硬件有三路ADC (JDR1/2/3)，第三路仅用于 `phase_a_current_actual` 但不参与FOC。

**影响:** A相误差 = B误差 + C误差，噪声叠加。

---

## 9. 初始化与上电

### 9.1 上电流程

```
Motor_Init()     → phase_order=NEGATIVE, R=0.088, ...
Encoder_Init()   → elec_offset=1.719367(硬编码)
Read_MotorData() → 从Flash加载 (如果cali_finish==1)
init_controller_params()
CalcCurrentOffset()
启动TIM1
```

### 问题 F-14 [严重]: 未校准可运行

```c
// encoder.c:53
elec_offset = 1.719367;  // 硬编码默认值
```

**如果Flash无效 (首次上电):**
1. `cali_finish = 0` → `elec_offset` 保持硬编码值
2. 这是某次特定电机的校准结果，**不适用其他电机**
3. 错误offset → 错误换相 → 不可预测力矩

**当前行为:** 代码不阻止未校准电机运行！

**建议:**
```c
case MOTOR_MODE:
    if(state_change) {
        if(p_encoder_g->cali_finish != 1) {
            printf("ERROR: Not calibrated!\r\n");
            FSMstate = REST_MODE;
            break;
        }
        // ...
    }
```

### 问题 F-15 [中等]: 上电rotations=0

```c
p_encoder_g->rotations = 0;
```

每次上电从0开始，多圈信息丢失。绝对位置精度依赖编码器是否绝对式 (24-bit可能是MBS) → 单圈绝对，多圈靠 `rotations` → 上电后多圈丢失。

---

## 10. 问题汇总

| 编号 | 严重度 | 问题 | 位置 |
|------|--------|------|------|
| **F-01** | **致命** | elec_pos未mod 2π, arm_sin/cos输入超范围 | encoder.c:116 |
| **F-02** | **严重** | 校准期间ISR与校准代码竞争修改编码器 | calibration.c |
| **F-03** | **严重** | Q轴PI用D轴增益 ki_d/k_d | FOC.c:315,334 |
| **F-04** | **中等** | 积分标量限幅，输出向量限幅，不一致 | FOC.c:317-325 |
| **F-05** | **中等** | 前馈补偿注释掉，高速dq耦合大 | FOC.c:336-337 |
| **F-06** | **严重** | 位置环(输出端)速度环(电机端)单位不一致 | FOC.c:255,282 |
| ~~F-07~~ | ~~中等~~ | ~~速度计算用PWM_FREQ，可能与实际不符~~ | **已确认正确**: ISR=10kHz与代码一致 |
| **F-09** | **中等** | PID死区硬编码只对POSITION_LOOP生效 | pid.c:132 |
| **F-10** | **严重** | 输出超限积分清零导致定位恢复慢 | pid.c:141 |
| **F-11** | **中等** | 位置/速度环同频率，无频率分离 | encoder.c:137-151 |
| **F-13** | **中等** | A相电流由-(B+C)，未用第三路ADC | motor.c:335 |
| **F-14** | **严重** | 未校准可运行，用硬编码offset | encoder.c:53 |
| **F-15** | **中等** | 上电rotations=0，多圈丢失 | encoder.c:49 |

---

## 11. 修复优先级

### P0 - 立即修复

| 编号 | 修复 | 工作量 |
|------|------|--------|
| F-01 | elec_pos做fmodf_pos归一化 | 1行 |
| F-03 | q_int用ki_q, v_q用k_q | 2行 |
| F-14 | MOTOR_MODE入口检查cali_finish | 5行 |

### P1 - 尽快修复

| 编号 | 修复 | 工作量 |
|------|------|--------|
| F-06 | 理清位置/速度单位，确认是否需×GR | 需分析 |
| F-10 | 积分饱和改冻结策略 | 3行 |
| F-02 | 校准期间禁用ISR的encoderSample | 5行 |

### P2 - 计划修复

| 编号 | 修复 | 工作量 |
|------|------|--------|
| F-05 | 启用dq前馈 | 2行+调参 |
| F-11 | pos_calc_period改16 | 1行 |

---

*Report generated by FOC algorithm review - Claude*
