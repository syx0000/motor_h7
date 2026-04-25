# 温度采样处理方案（Dual Mode 配置下）

**项目**：STM32H7 FOC 电机控制系统  
**日期**：2026-04-24  
**版本**：v1.0  

---

## 问题背景

实施 ADC1/ADC2 dual mode 后，ADC2 的注入通道被用于电流和电压采样：
- ADC2 注入 Rank 1: CUR_C (INP8)
- ADC2 注入 Rank 2: VDC (INP3)

原本的温度采样需要重新安排：
- TEMP_MOS (PB1, ADCx_INP5)
- TEMP_MOTOR (PB0, ADCx_INP9)

---

## 方案对比

### 方案 A：使用 ADC3 注入通道（推荐 ⭐⭐⭐⭐⭐）

#### 原理

ADC3 独立工作，不影响 ADC1/ADC2 dual mode。

#### 配置

```c
// ADC3 注入通道配置
ADC_HandleTypeDef hadc3;

void MX_ADC3_Init(void)
{
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;  // 30MHz
    hadc3.Init.Resolution = ADC_RESOLUTION_16B;
    hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagementMode = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.OversamplingMode = DISABLE;
    HAL_ADC_Init(&hadc3);
    
    // 注入通道配置
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedNbrOfConversion = 2;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    sConfigInjected.InjecOversamplingMode = DISABLE;
    
    // Rank 1: TEMP_MOS (INP5)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_16CYCLES_5;
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);
    
    // Rank 2: TEMP_MOTOR (INP9)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);
}
```

#### 触发方式

在 TIM1_UP 的 1ms 分支里软件触发：

```c
void TIM1_UP_IRQHandler(void)
{
    // ...
    u32Timecnt++;
    if (u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)  // 1ms
    {
        u32Timecnt = 0;
        u8_1msFlag = 1;
        
        // 触发 ADC3 温度采样（1kHz）
        HAL_ADCEx_InjectedStart(&hadc3);
    }
    // ...
}
```

#### 数据读取

```c
void TemperatureSample(void)
{
    // 读取 ADC3 注入通道结果
    float temp_mos_raw = (float)ADC3->JDR1;
    float temp_motor_raw = (float)ADC3->JDR2;
    
    // 温度计算（根据实际传感器特性）
    p_motor_g->temp_mos = temp_mos_raw * TEMP_COEFFICIENT;
    p_motor_g->temp_motor = temp_motor_raw * TEMP_COEFFICIENT;
}
```

#### 优劣分析

**优势**：
- ✅ 完全独立，不影响 dual mode
- ✅ 配置简单，逻辑清晰
- ✅ 采样频率灵活（1kHz 足够）
- ✅ 无需复杂的时分复用

**劣势**：
- ⚠️ 占用 ADC3（但当前未使用）
- ⚠️ 需要在 CubeMX 中配置 ADC3

**推荐度**：⭐⭐⭐⭐⭐

---

### 方案 B：ADC2 规则通道 + 注入通道并行

#### 原理

ADC2 同时配置注入通道和规则通道：
- 注入通道：CUR_C, VDC（10kHz，dual mode）
- 规则通道：TEMP_MOS, TEMP_MOTOR（1kHz，独立）

#### 配置

```c
void MX_ADC2_Init(void)
{
    // ... 注入通道配置（dual mode）...
    
    // 规则通道配置
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Rank 1: TEMP_MOS (INP5)
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
    
    // Rank 2: TEMP_MOTOR (INP9)
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}
```

#### 触发方式

在主循环或 1ms 定时器里触发规则通道：

```c
// 在主循环里
if (u8_1msFlag == 1)
{
    u8_1msFlag = 0;
    
    // 触发 ADC2 规则通道（温度采样）
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    temp_mos_raw = HAL_ADC_GetValue(&hadc2);
    
    HAL_ADC_PollForConversion(&hadc2, 10);
    temp_motor_raw = HAL_ADC_GetValue(&hadc2);
    
    HAL_ADC_Stop(&hadc2);
}
```

#### 优劣分析

**优势**：
- ✅ 不占用 ADC3
- ✅ 注入通道和规则通道可以并行工作

**劣势**：
- ⚠️ 配置复杂（同一个 ADC 两种通道）
- ⚠️ 需要仲裁机制（注入通道优先级更高）
- ⚠️ 轮询方式阻塞主循环

**推荐度**：⭐⭐⭐

---

### 方案 C：降低采样频率，主循环轮询

#### 原理

温度变化慢，1Hz 采样足够。在主循环里用 ADC1 或 ADC3 的规则通道采样。

#### 配置

```c
// 在主循环里
static uint32_t temp_sample_cnt = 0;
if (u8_1msFlag == 1)
{
    u8_1msFlag = 0;
    temp_sample_cnt++;
    
    if (temp_sample_cnt >= 1000)  // 1Hz
    {
        temp_sample_cnt = 0;
        
        // 使用 ADC1 规则通道采样温度
        // （此时 ADC1 注入通道空闲）
        HAL_ADC_Start(&hadc1);
        
        // 采样 TEMP_MOS
        HAL_ADC_PollForConversion(&hadc1, 10);
        temp_mos_raw = HAL_ADC_GetValue(&hadc1);
        
        // 切换通道，采样 TEMP_MOTOR
        // ...
        
        HAL_ADC_Stop(&hadc1);
    }
}
```

#### 优劣分析

**优势**：
- ✅ 不占用额外 ADC
- ✅ 采样频率低，对系统影响小

**劣势**：
- ⚠️ 需要动态切换 ADC 通道
- ⚠️ 轮询方式阻塞主循环
- ⚠️ 1Hz 采样可能太慢（过温保护响应慢）

**推荐度**：⭐⭐

---

### 方案 D：ADC2 时分复用（不推荐）

#### 原理

在不同时间段，ADC2 切换配置：
- 大部分时间：dual mode（电流/电压采样）
- 偶尔切换：温度采样

#### 实现

```c
// 在 1ms 分支里
if (u32Timecnt % 10 == 0)  // 每 10ms
{
    // 暂停 dual mode
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    
    // 重新配置 ADC2 为温度采样
    // ...
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedPollForConversion(&hadc2, 10);
    // 读取温度
    
    // 恢复 dual mode 配置
    // ...
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}
```

#### 优劣分析

**优势**：
- ✅ 不占用额外 ADC

**劣势**：
- ❌ 配置切换复杂，容易出错
- ❌ 切换期间 FOC 采样中断
- ❌ 调试困难

**推荐度**：⭐（不推荐）

---

## 推荐实施方案

### 首选：方案 A（ADC3 注入通道）

**理由**：
1. 完全独立，不影响 dual mode
2. 配置简单，逻辑清晰
3. 采样频率灵活
4. ADC3 当前未使用

**实施步骤**：

#### 步骤 1：在 CubeMX 中配置 ADC3

1. 打开 `FIVE.ioc`
2. 在 Pinout & Configuration 中启用 ADC3
3. 配置 ADC3 注入通道：
   - Channel 5 (PB1, TEMP_MOS)
   - Channel 9 (PB0, TEMP_MOTOR)
4. 生成代码

#### 步骤 2：修改代码

**文件**：`Core/Src/adc.c`

添加 ADC3 初始化（CubeMX 自动生成）

**文件**：`Core/Src/stm32h7xx_it.c`

在 TIM1_UP 的 1ms 分支里触发 ADC3：

```c
if (u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)
{
    u32Timecnt = 0;
    u8_1msFlag = 1;
    
    // 触发 ADC3 温度采样
    HAL_ADCEx_InjectedStart(&hadc3);
}
```

**文件**：`FOC/motor.c`

修改 `TemperatureSample()` 函数：

```c
void TemperatureSample(void)
{
    // 读取 ADC3 结果
    float temp_mos_raw = (float)ADC3->JDR1;
    float temp_motor_raw = (float)ADC3->JDR2;
    
    // 温度计算
    p_motor_g->temp_mos = temp_mos_raw * TEMP_COEFFICIENT;
    p_motor_g->temp_motor = temp_motor_raw * TEMP_COEFFICIENT;
}
```

#### 步骤 3：在主循环里调用

```c
// 在主循环里
if (u8_1msFlag == 1)
{
    u8_1msFlag = 0;
    TemperatureSample();
    
    // 过温保护
    if (p_motor_g->temp_mos > TEMP_MAX || p_motor_g->temp_motor > TEMP_MAX)
    {
        // 触发保护
        DisablePWM();
        FSMstate = REST_MODE;
    }
}
```

---

### 备选：方案 B（ADC2 规则通道）

如果 ADC3 不可用或引脚冲突，使用方案 B。

**关键点**：
- ADC2 注入通道：dual mode（优先级高）
- ADC2 规则通道：温度采样（优先级低）
- 在主循环里轮询规则通道

---

## 性能对比

| 方案 | ADC 占用 | 采样频率 | 实现复杂度 | 对 FOC 影响 | 推荐度 |
|------|---------|---------|-----------|------------|--------|
| A. ADC3 注入 | ADC3 | 1kHz | 低 | 无 | ⭐⭐⭐⭐⭐ |
| B. ADC2 规则 | ADC2 | 1kHz | 中 | 极小 | ⭐⭐⭐ |
| C. 主循环轮询 | ADC1/3 | 1Hz | 中 | 极小 | ⭐⭐ |
| D. 时分复用 | ADC2 | 100Hz | 高 | 较大 | ⭐ |

---

## 温度采样频率分析

### 温度变化特性

```
电机温度时间常数：τ ≈ 10-60 秒
MOS 管温度时间常数：τ ≈ 1-5 秒

根据采样定理，采样频率应 > 2 × 带宽：
  电机温度：f_sample > 2 / 60s = 0.033Hz → 1Hz 足够
  MOS 管温度：f_sample > 2 / 1s = 2Hz → 10Hz 足够
```

### 推荐采样频率

| 应用 | 采样频率 | 说明 |
|------|---------|------|
| 正常监控 | 1Hz | 足够检测缓慢温升 |
| 过温保护 | 10Hz | 快速响应突发过热 |
| 高性能应用 | 100Hz | 实时温度反馈控制 |

**建议**：使用 10Hz（每 100ms 采样一次），平衡响应速度和系统开销。

---

## 完整实施方案（方案 A）

### 文件修改清单

| 文件 | 修改内容 |
|------|---------|
| `FIVE.ioc` | 启用 ADC3，配置注入通道 |
| `Core/Src/adc.c` | ADC3 初始化（自动生成） |
| `Core/Src/stm32h7xx_it.c` | TIM1_UP 1ms 分支触发 ADC3 |
| `Core/Src/main.c` | 主循环调用 TemperatureSample() |
| `FOC/motor.c` | 修改 TemperatureSample() 读取 ADC3 |

### 代码示例

#### TIM1_UP_IRQHandler（stm32h7xx_it.c）

```c
void TIM1_UP_IRQHandler(void)
{
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);
    ISR_start = DWT_CYCCNT;
    
    u8_100usFlag = 1;
    
    u32Timecnt++;
    if (u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)
    {
        u32Timecnt = 0;
        u8_1msFlag = 1;
        
        // 触发 ADC3 温度采样（1kHz）
        HAL_ADCEx_InjectedStart(&hadc3);
    }
    
    ErrorDiag();
    EncoderSample();
    
    TIM1->SR = 0x0;
}
```

#### 主循环（main.c）

```c
while (1)
{
    if (u8_1msFlag == 1)
    {
        u8_1msFlag = 0;
        
        // 温度采样和保护
        TemperatureSample();
        
        if (p_motor_g->temp_mos > TEMP_MAX_MOS || 
            p_motor_g->temp_motor > TEMP_MAX_MOTOR)
        {
            DisablePWM();
            FSMstate = REST_MODE;
            state_change = 1;
            printf("[ERROR] Over temperature!\r\n");
        }
    }
    
    // 其他主循环任务
    // ...
}
```

#### TemperatureSample（motor.c）

```c
void TemperatureSample(void)
{
    // 读取 ADC3 注入通道结果
    uint32_t temp_mos_raw = ADC3->JDR1;
    uint32_t temp_motor_raw = ADC3->JDR2;
    
    // 温度计算（根据实际传感器特性调整）
    // 假设使用 NTC 热敏电阻，需要查表或公式转换
    p_motor_g->temp_mos = NTC_to_Temperature(temp_mos_raw);
    p_motor_g->temp_motor = NTC_to_Temperature(temp_motor_raw);
}

// NTC 温度转换（示例）
float NTC_to_Temperature(uint32_t adc_value)
{
    // 根据实际 NTC 参数调整
    float voltage = (float)adc_value * 3.3f / 65536.0f;  // 16-bit ADC
    float resistance = 10000.0f * voltage / (3.3f - voltage);  // 10k 上拉
    
    // Steinhart-Hart 方程（需要根据实际 NTC 参数校准）
    float temp_k = 1.0f / (A + B * logf(resistance) + C * powf(logf(resistance), 3));
    float temp_c = temp_k - 273.15f;
    
    return temp_c;
}
```

---

## 验证方案

### 功能验证

1. **ADC3 数据验证**：
   - 读取 ADC3->JDR1/JDR2
   - 确认数值在合理范围内

2. **温度计算验证**：
   - 对比实际温度（红外测温枪）
   - 校准温度转换公式

3. **过温保护验证**：
   - 模拟过温（加热 MOS 管）
   - 确认保护动作正常

### 性能验证

- 确认 ADC3 采样不影响 dual mode
- 测量温度采样的 CPU 开销
- 验证 1kHz 采样频率足够

---

## 总结

### 推荐方案

**方案 A（ADC3 注入通道）**：
- ✅ 完全独立，不影响 dual mode
- ✅ 配置简单，逻辑清晰
- ✅ 1kHz 采样频率足够
- ✅ 实施风险低

### 实施优先级

1. **立即实施**：配置 ADC3，实现温度采样
2. **功能验证**：确认温度数据正确
3. **集成测试**：与 dual mode 一起测试

### 后续优化

- 根据实际需求调整采样频率（1Hz - 100Hz）
- 添加温度滤波（移动平均）
- 实现温度曲线记录（用于故障分析）

---

**文档版本历史**：

| 版本 | 日期 | 作者 | 变更说明 |
|------|------|------|---------|
| v1.0 | 2026-04-24 | syx0000 & Claude | 初始版本，温度采样处理方案 |
