# ADC 真正并行采样方案分析

**项目**：STM32H7 FOC 电机控制系统  
**日期**：2026-04-24  
**版本**：v1.0  

---

## 1. 当前配置分析

### 1.1 硬件连接

| 信号 | 引脚 | ADC通道 | 当前分配 |
|------|------|---------|---------|
| CUR_A | PA7 | ADC1_INP7 | ADC1 注入通道 Rank 2 |
| CUR_B | PC4 | ADC1_INP4 | ADC1 注入通道 Rank 1 |
| CUR_C | PC5 | ADC1_INP8 | ADC1 注入通道 Rank 3 |
| VDC | PA6 | ADC1_INP3 | ADC1 注入通道 Rank 4 |
| TEMP_MOS | PB1 | ADC2_INP5 | ADC2 注入通道 Rank 1 |
| TEMP_MOTOR | PB0 | ADC2_INP9 | ADC2 注入通道 Rank 2 |

### 1.2 当前采样时序

```
ADC1 注入序列（顺序转换）：
  Rank 1: CUR_B (1.5 + 8.5 = 10 周期)
  Rank 2: CUR_A (10 周期)
  Rank 3: CUR_C (10 周期)
  Rank 4: VDC   (10 周期)
  总计: 40 周期 / 30MHz = 1.33µs

ADC2 注入序列（独立，软件触发）：
  Rank 1: TEMP_MOS (16.5 + 8.5 = 25 周期)
  Rank 2: TEMP_MOTOR (25 周期)
  总计: 50 周期 / 30MHz = 1.67µs
```

### 1.3 问题分析

**核心问题**：三相电流是顺序采样，存在 ~0.33µs 的通道间时间差

**影响评估**：
```
PWM 周期：100µs (10kHz)
电流采样窗口：1.33µs
占比：1.3%

假设电流变化率：di/dt = 100A/ms（极端情况）
在 0.33µs 内的电流变化：100A/ms × 0.33µs = 0.033A = 33mA

对于额定电流 10A 的电机：
  相对误差 = 33mA / 10A = 0.33%
```

**结论**：对于大多数应用，当前精度已足够（误差 < 0.5%）

---

## 2. 方案对比

### 方案 A：ADC1/ADC2 Dual Mode Simultaneous

#### 原理

利用 STM32H7 的 dual mode，ADC1 和 ADC2 同步采样不同通道。

#### 配置方案

```
ADC1 (Master) - 注入通道：
  Rank 1: INP7 (CUR_A)
  Rank 2: INP4 (CUR_B)
  
ADC2 (Slave) - 注入通道：
  Rank 1: INP8 (CUR_C) ← 需要硬件改动
  Rank 2: INP3 (VDC)   ← 需要硬件改动

Dual Mode 配置：
  Mode: ADC_DUALMODE_INJECSIMULT
  Trigger: TIM1_TRGO (共享)
```

#### 硬件改动需求

**必须改动**：
- PC5 (CUR_C) 需要同时连接到 ADC1_INP8 和 ADC2_INP8
- PA6 (VDC) 需要同时连接到 ADC1_INP3 和 ADC2_INP3

**实现方式**：
- 在 PCB 上将信号线同时引到两个 ADC 的输入引脚
- 或使用模拟开关/多路复用器

#### 时序优化

```
优化前（顺序）：
  CUR_B → CUR_A → CUR_C → VDC
  |--0.33µs--|--0.33µs--|--0.33µs--|
  总计: 1.33µs

优化后（dual mode）：
  ADC1: CUR_A → CUR_B
  ADC2: CUR_C → VDC  (同时进行)
  |-----0.67µs-----|
  总计: 0.67µs（减半）
```

#### 优劣分析

**优势**：
- ✅ 采样时间减半（1.33µs → 0.67µs）
- ✅ CUR_A 和 CUR_B 完全同步
- ✅ CUR_C 和 VDC 完全同步
- ✅ 软件实现相对简单

**劣势**：
- ❌ 需要硬件重新布线
- ⚠️ CUR_A/B 与 CUR_C 之间仍有 ~0.33µs 时间差
- ⚠️ 增加 PCB 布线复杂度

**推荐度**：⭐⭐（需要硬件改动，收益有限）

---

### 方案 B：ADC1/ADC2/ADC3 Triple Mode（理想方案）

#### 原理

三个 ADC 同时采样三相电流，实现真正的零时间差并行。

#### 配置方案

```
ADC1: INP7 (CUR_A)
ADC2: INP4 (CUR_B) ← 需要硬件改动
ADC3: INP8 (CUR_C) ← 需要硬件改动

Triple Mode 配置：
  Mode: ADC_TRIPLEMODE_INJECSIMULT
  Trigger: TIM1_TRGO (共享)
```

#### 硬件改动需求

**必须改动**：
- PC4 (CUR_B) 需要连接到 ADC2 的某个输入（如 ADC2_INP4）
- PC5 (CUR_C) 需要连接到 ADC3 的某个输入（如 ADC3_INP8）

**引脚重新分配**：

| 信号 | 原引脚 | 新引脚（示例） | ADC通道 |
|------|--------|---------------|---------|
| CUR_A | PA7 | PA7 | ADC1_INP7 |
| CUR_B | PC4 | PF11 | ADC2_INP2 |
| CUR_C | PC5 | PH2 | ADC3_INP13 |

#### 时序优化

```
优化前（顺序）：
  CUR_B → CUR_A → CUR_C
  |--0.33µs--|--0.33µs--|
  总计: 1.0µs

优化后（triple mode）：
  ADC1: CUR_A
  ADC2: CUR_B  (完全同时)
  ADC3: CUR_C
  |--0.33µs--|
  总计: 0.33µs（减少 67%）
```

#### 优劣分析

**优势**：
- ✅ 三相电流完全同步采样（零时间差）
- ✅ 采样时间最短（1.0µs → 0.33µs）
- ✅ 最高精度的 Clarke 变换
- ✅ 电流环带宽可进一步提升

**劣势**：
- ❌ 需要大量硬件改动（PCB 重新设计）
- ❌ 占用 ADC3（当前未使用）
- ⚠️ 软件复杂度增加（triple mode 配置）
- ⚠️ 调试难度增加

**推荐度**：⭐⭐⭐⭐⭐（硬件改版时的最佳方案）

---

### 方案 C：提高 ADC 时钟（软件优化，推荐）

#### 原理

加快单个 ADC 的转换速度，减少顺序采样的通道间时间差。

#### 配置方案

```c
// 当前配置（adc.c:63）：
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;  // 120MHz / 4 = 30MHz

// 优化配置：
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // 120MHz / 2 = 60MHz
```

#### 效果对比

| 参数 | 当前 (30MHz) | 优化后 (60MHz) | 改善 |
|------|-------------|---------------|------|
| 单通道时间 | 0.33µs | 0.167µs | -50% |
| 4 通道总时间 | 1.33µs | 0.67µs | -50% |
| 通道间时间差 | 0.33µs | 0.167µs | -50% |
| 相对误差 (10A) | 0.33% | 0.17% | -50% |

#### 实现步骤

1. **修改 ADC1 时钟**（adc.c:63）：
```c
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
```

2. **修改 ADC2 时钟**（adc.c:166）：
```c
hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
```

3. **验证采样时间**：
   - 确保信号建立时间足够（1.5 周期 @ 60MHz = 25ns）
   - 测试 ADC 噪声是否增加

#### 优劣分析

**优势**：
- ✅ 无需硬件改动
- ✅ 实现极其简单（改 2 行代码）
- ✅ 时间差减小 50%
- ✅ 风险低，易于回退

**劣势**：
- ⚠️ 仍然是顺序采样，不是真正并行
- ⚠️ 更高的 ADC 时钟可能增加噪声（需实测）
- ⚠️ 采样时间不能太短（信号建立时间限制）

**风险评估**：
- STM32H7 ADC 最高支持 80MHz（60MHz 在安全范围内）
- 1.5 周期采样时间 @ 60MHz = 25ns（通常足够）
- 如果噪声增加，可尝试 40MHz (DIV3) 作为折中

**推荐度**：⭐⭐⭐⭐（立即可行，收益明显）

---

### 方案 D：保持现状（实用主义）

#### 性能评估

**当前指标**：
- 采样时间：1.33µs
- 通道间时间差：0.33µs
- 相对误差：0.33% (10A 电机)

**适用场景**：
- 额定电流 < 50A
- PWM 频率 ≤ 20kHz
- 电流环带宽 < 1kHz
- 对 d/q 解耦精度要求不极端

#### 理论分析

**Clarke 变换误差**：

假设三相电流在 0.33µs 内的变化：
```
Ia(t) = I·sin(ωt)
Ib(t) = I·sin(ωt - 2π/3)
Ic(t) = I·sin(ωt + 2π/3)

采样时刻：
  t1 = 0      (CUR_B)
  t2 = 0.33µs (CUR_A)
  t3 = 0.67µs (CUR_C)

对于 1000 RPM, 7 极对电机：
  电角频率 ω = 1000/60 × 7 × 2π = 733 rad/s
  相位差 Δφ = ω × 0.33µs = 0.00024 rad = 0.014°

结论：相位误差 < 0.02°，可忽略
```

**推荐度**：⭐⭐⭐⭐（当前已足够）

---

## 3. 推荐实施路线

### 3.1 短期方案（立即可行）

**步骤 1**：评估当前性能
- 测试电流环带宽（阶跃响应）
- 观察 d/q 电流纹波（示波器）
- 测量高速时的稳定性

**步骤 2**：如果性能满足要求
- 保持现状（方案 D）
- 专注于其他优化（控制算法、参数整定）

**步骤 3**：如果需要进一步优化
- 实施方案 C（提高 ADC 时钟到 60MHz）
- 测试噪声和精度
- 如果噪声增加，回退到 40MHz

### 3.2 长期方案（硬件改版）

**下一代硬件设计建议**：
- 采用方案 B（triple mode）
- 三相电流分配到 ADC1/ADC2/ADC3
- 预留 ADC3 的引脚和电路
- 实现真正的零时间差并行采样

---

## 4. 方案 C 实现细节

### 4.1 代码修改

**文件**：`Core/Src/adc.c`

```c
// Line 63: ADC1 时钟配置
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // 从 DIV4 改为 DIV2

// Line 166: ADC2 时钟配置
hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // 从 DIV4 改为 DIV2
```

### 4.2 验证方案

**编译验证**：
- 确保 0 error, 0 warning
- 检查 ADC 初始化代码

**功能验证**：
1. 测试 ADC 采样值是否正常
2. 观察电流波形是否平滑
3. 测量 ADC 噪声（FFT 分析）

**性能对比**：

| 测试项 | 优化前 | 优化后 | 目标 |
|--------|--------|--------|------|
| ADC 采样时间 | 1.33µs | 0.67µs | < 1µs |
| 电流纹波 (Iq) | 测量 | 测量 | 减小 |
| 电流环带宽 | 测量 | 测量 | 提升 |
| ADC 噪声 (RMS) | 测量 | 测量 | < 2× |

### 4.3 风险缓解

**如果噪声增加**：
- 尝试 DIV3 (40MHz) 作为折中
- 增加采样时间到 2.5 周期
- 添加硬件滤波电路

**如果信号建立时间不足**：
- 增加采样时间（1.5 → 2.5 周期）
- 降低 ADC 时钟（60MHz → 40MHz）

---

## 5. 总结与建议

### 5.1 方案对比表

| 方案 | 硬件改动 | 软件复杂度 | 采样时间 | 时间差 | 推荐度 |
|------|---------|-----------|---------|--------|--------|
| A. Dual Mode | 需要 | 中 | 0.67µs | 0.17µs | ⭐⭐ |
| B. Triple Mode | 大量 | 高 | 0.33µs | 0µs | ⭐⭐⭐⭐⭐ (改版) |
| C. 提高时钟 | 无 | 低 | 0.67µs | 0.17µs | ⭐⭐⭐⭐ |
| D. 保持现状 | 无 | 无 | 1.33µs | 0.33µs | ⭐⭐⭐⭐ (当前) |

### 5.2 决策建议

**立即实施**：
1. 先测试当前性能（方案 D）
2. 如果 d/q 纹波 > 5% 或电流环带宽 < 500Hz，实施方案 C

**中期规划**：
- 如果方案 C 效果不明显，考虑算法优化（PID 参数、前馈补偿）
- 评估是否真的需要硬件级并行采样

**长期规划**：
- 下一代硬件设计时采用方案 B（triple mode）
- 预留 ADC3 电路和引脚
- 实现真正的零时间差并行采样

### 5.3 关键结论

1. **当前精度已足够**：0.33µs 的时间差对应 0.33% 的误差，对大多数应用可接受

2. **方案 C 性价比最高**：无需硬件改动，2 行代码，时间差减半

3. **方案 B 是终极方案**：硬件改版时的最佳选择，实现真正并行

4. **不要过度优化**：先测试当前性能，确认瓶颈后再优化

---

## 附录：STM32H7 ADC Dual/Triple Mode 配置示例

### Dual Mode 配置（参考）

```c
// ADC1 (Master) 配置
ADC_MultiModeTypeDef multimode = {0};
multimode.Mode = ADC_DUALMODE_INJECSIMULT;  // 注入通道同步模式
multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

// ADC1 注入通道配置
ADC_InjectionConfTypeDef sConfigInjected = {0};
sConfigInjected.InjectedChannel = ADC_CHANNEL_7;  // CUR_A
sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
sConfigInjected.InjectedNbrOfConversion = 2;
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

// ADC2 (Slave) 注入通道配置
sConfigInjected.InjectedChannel = ADC_CHANNEL_8;  // CUR_C
HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);
```

**注意**：Dual mode 需要硬件支持，确保信号连接到对应的 ADC 输入。

---

**文档版本历史**：

| 版本 | 日期 | 作者 | 变更说明 |
|------|------|------|---------|
| v1.0 | 2026-04-24 | syx0000 & Claude | 初始版本，ADC 并行采样方案分析 |
