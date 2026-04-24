# Interrupt & Loop Architecture Review

**Project:** FIVE (STM32H743VITx FOC Motor Driver)  
**Date:** 2026-04-17  
**Scope:** 中断优先级、定时器配置、主循环调度、数据流竞争、阻塞分析

---

## 1. 中断优先级配置现状

### NVIC 分组

```c
// stm32h7xx_hal.c:147
HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
// → 4 bits 抢占优先级, 0 bits 子优先级
// → 数值越小优先级越高 (0=最高, 15=最低)
```

### 实际优先级表

| 中断源 | 抢占优先级 | 功能 | 配置位置 |
|--------|-----------|------|---------|
| **DMA1_Stream0 (USART2 RX)** | **0** | RS485编码器接收DMA | dma.c:47 |
| **DMA1_Stream1 (USART2 TX)** | **0** | RS485编码器发送DMA | dma.c:50 |
| **USART2** | **0** | RS485编码器中断(IDLE+TC) | usart.c:343 |
| **TIM1_UP** | **1** | FOC电流环 + 状态机 | tim.c:220 |
| **FDCAN1_IT0** | **1** | CAN消息接收 | fdcan.c:125 |
| **USART1** | **1** | 调试串口命令 | usart.c:269 |
| DMA1_Stream3 (USART6 TX) | 2 | VOFA+调试数据DMA | dma.c:53 |
| USART6 | 2 | VOFA+调试串口 | usart.c:433 |
| SysTick | 15 | HAL时基 | HAL默认 |

---

## 2. 严重问题：中断优先级倒挂

### 问题 I-01 [致命]: USART2/DMA 优先级高于 TIM1 (FOC电流环)

```
当前配置:
  USART2 (编码器RS485)  → 优先级 0 (最高)
  DMA1_Stream0 (RX DMA) → 优先级 0 (最高)
  DMA1_Stream1 (TX DMA) → 优先级 0 (最高)
  TIM1_UP (FOC电流环)   → 优先级 1

后果:
  RS485通信可以随时抢占FOC电流环！
```

**影响分析:**

USART2中断处理函数 (`stm32h7xx_it.c:937-961`) 会在接收到编码器数据或发送完成时触发。如果USART2中断在FOC电流环执行到一半时抢占：

1. **SVPWM计算被打断:** Ta/Tb/Tc可能被写入一半
2. **电流采样时序偏移:** ADC注入转换的触发时机被延迟
3. **最坏情况:** USART2_IRQHandler中的IDLE检测 + DMA停止 + 数据拷贝 + CRC校验可能占用10-20us

```
时序示意 (打断场景):

TIM1 ISR: [ADC采样][Clark][Park]--被USART2抢占--[PI计算][SVPWM]
                                 ↑                        ↑
                          电流值已采样              SVPWM基于旧角度
                          但角度已更新              输出电压可能错误
```

**修复:**

```c
// TIM1 (FOC) 必须是最高优先级
HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);    // 最高

// USART2 (编码器) 可以略低
HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
```

---

### 问题 I-02 [严重]: FDCAN 与 TIM1 同优先级

```
FDCAN1_IT0 → 优先级 1
TIM1_UP    → 优先级 1
```

同一抢占优先级的中断**不能互相抢占**。当 FDCAN ISR 先进入时（取决于NVIC向量号顺序），TIM1必须等待FDCAN处理完成。

FDCAN回调链：
```
FDCAN1_IT0_IRQHandler()
  → HAL_FDCAN_IRQHandler()
    → HAL_FDCAN_RxFifo0Callback()
      → CAN_MsgProcess()  ← 490行代码！
        → 可能调用 Write_MotorData()  ← 擦除Flash 1-2秒！
        → 发送CAN回复消息
        → unpack各种指令
```

`CAN_MsgProcess` 最短执行路径约 5-10us，最长路径（包含Flash操作）可达 **1-2秒**。

**影响:** TIM1 ISR最多被延迟 **1-2秒**，FOC电流环完全停摆，电机失控。

---

### 问题 I-03 [严重]: USART1 (调试串口) 与 TIM1 同优先级

```
USART1 → 优先级 1
TIM1   → 优先级 1
```

USART1 接收中断处理函数 (`stm32h7xx_it.c:568-931`) 包含：
- 状态机命令解析（300+ 行 switch-case）
- 多次 `printf` 调用（每次可能阻塞数十us）
- 多次 `delay_us(10)` 调用
- 校准模式设置
- `atof()`/`atoi()` 浮点解析
- 在REST_MODE下收到'm'时: **6次printf + 6次delay_us** → 约60-100us

**影响:** 每次串口输入字符，TIM1可能被延迟60-100us。

---

## 3. 定时器配置分析

### 3.1 TIM1 - PWM & FOC定时器

```c
// tim.c
htim1.Init.Prescaler = 0;                    // 无分频
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;  // 中心对齐模式1
htim1.Init.Period = 11999;                    // ARR = 11999
htim1.Init.RepetitionCounter = 1;             // RCR = 1
```

**时钟树:**
```
HSE 12MHz → PLL (M=5, N=192, P=2) → SYSCLK = 230.4MHz
                                             ↓ (但代码定义MCU_SYSCLK = 480MHz)
SYSCLK → AHB ÷2 = 115.2MHz → APB2 ÷2 = 57.6MHz → TIM1 ×2 = 115.2MHz
                                    或
如果SYSCLK = 480MHz → AHB ÷2 = 240MHz → APB2 ÷2 = 120MHz → TIM1 ×2 = 240MHz
```

**PWM频率计算 (假设TIM1 clk = 240MHz):**
```
PWM_freq = TIM1_clk / (2 × (ARR+1))
         = 240MHz / (2 × 12000)
         = 10kHz ✓ (与 PWM_FREQUENCY_DEFAULT = 10000 一致)
```

### ~~问题 I-04~~ [已确认正确]: RepetitionCounter=1 在中心对齐模式下ISR=10kHz

中心对齐模式1 (CMS=01) 下：
- 计数器在**上溢（overflow）和下溢（underflow）**时都会递减 RepetitionCounter
- 每个 PWM 周期有 **2 次计数事件**
- `RepetitionCounter = 1` 表示每 (RCR+1)=**2 次事件**才触发一次中断
- **实际ISR频率 = 2 × 10kHz / 2 = 10kHz** ✓

代码中所有时间相关的计算基于 10kHz，与实际一致：
```c
#define PWM_FREQUENCY_DEFAULT  10000.0f  // FOC.h:155 ✓
#define PeriodPWM              0.0001f   // 100us ✓

// 速度计算 (encoder.c:165)
mech_vel = delta_cnt * 2π * PWM_FREQUENCY_DEFAULT / cpr / vel_calc_period;
//                          ✓ ISR实际10kHz，速度计算正确

// 1ms定时标志 (stm32h7xx_it.c:295)
if(u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)  // = 10
//                 ✓ ISR是10kHz，10次 × 100us = 1ms 正确
```

**结论:** RCR=1 配置正确，无需修改。

---

### 3.2 TIM6 - 微秒延时定时器

```c
htim6.Init.Prescaler = 119;     // 分频 120
htim6.Init.Period = 65535;      // 最大计数
htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
```

**TIM6 频率:**
```
APB1 timer clk = 240MHz (假设)
TIM6_cnt_clk = 240MHz / (119+1) = 2MHz → 0.5us/count
```

但 `delay_us` 函数直接将 nus 与 CNT 比较：
```c
void delay_us(uint16_t nus) {
    htim6.Instance->CNT = 0;
    while (htim6.Instance->CNT < nus);  // nus个计数 = nus × 0.5us
    htim6.Instance->CNT &= ~TIM_CR1_CEN;  // BUG: 写CNT而非CR1
}
```

### 问题 I-05 [中等]: delay_us 实际延时为请求值的一半

如果TIM6计数频率为2MHz（0.5us/count），则 `delay_us(10)` 实际只延时 **5us**。

> 如果TIM6 clk实际是120MHz: 120M / 120 = 1MHz → 1us/count → delay_us 正确。
> 需要根据实际时钟树确认。

---

### 3.3 TIM2 - 未使用的定时器

```c
htim2.Init.Prescaler = 0;
htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
htim2.Init.Period = 11999;
```

TIM2已初始化但未启用中断、未启动计数器。应删除或注释说明用途。

---

## 4. ADC采样时序分析

### 4.1 ADC触发方式

```c
// adc.c:95
sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
// → 软件触发！非硬件触发
```

### 问题 I-06 [严重]: ADC使用软件触发而非TIM1硬件触发

**当前流程:**
```
TIM1 ISR → startJADC() → HAL_ADCEx_InjectedStart() → 软件触发ADC注入转换
         → encoderSample()  // 编码器采样
         → currentSample()  // 读取ADC结果
```

**问题:** 软件触发ADC的时机取决于：
1. TIM1 ISR的进入时间（受高优先级中断延迟影响）
2. `startJADC()` 之前的代码执行时间（RS485发送、错误诊断）

```c
// stm32h7xx_it.c:288-311
void TIM1_UP_IRQHandler(void) {
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);
    ISR_start = DWT_CYCCNT;
    u8_100usFlag = 1;
    u32Timecnt++;
    if(u32Timecnt >= ...) { ... }
    RS485DIR_TX;                          // GPIO操作
    startJADC();                          // ← ADC在这里才开始！
    HAL_UART_Transmit_DMA(&huart2, ...);  // RS485 DMA发送
    errorDiag();                          // 错误诊断
    encoderSample();                      // 编码器采样
    voltageSample();                      // 电压采样
    currentSample();                      // 电流采样 ← 读取ADC结果
}
```

**问题:**
1. ADC触发时刻不确定，电流采样点在PWM周期中的位置不固定
2. ADC 1.5周期采样 + 转换时间 ≈ 几us，但 `startJADC()` 到 `currentSample()` 之间执行了 `HAL_UART_Transmit_DMA` + `errorDiag()` + `encoderSample()` + `voltageSample()`，时间足够
3. 如果被高优先级中断抢占（USART2 Priority 0），ADC触发时间进一步偏移

**正确做法:** 使用TIM1 TRGO硬件触发ADC注入转换，在PWM中心点自动采样：
```c
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```

TIM1已配置了 `MasterOutputTrigger = TIM_TRGO_UPDATE`，TRGO信号已就绪，只需在ADC侧启用硬件触发。

---

## 5. 主循环调度分析

### 5.1 主循环结构

```c
// main.c:192 - while(1)
while(1) {
    // 1. 扭矩测试 (阻塞5秒)
    if(FOC_CURRENT_LOOP && TorqueTestFlag) {
        for(n=1; n<11; n++) {
            Motor_Iq = 24.32f * n / 10;
            HAL_Delay(5000);              // ← 阻塞5秒 × 10次 = 50秒！
        }
    }
    
    // 2. 100us定时任务
    if(u8_100usFlag) {
        Pack_ActiveReport_Current();
        CAN_SendMessage();                // ← 在主循环中发送CAN
        u8_100usFlag = 0;
    }
    
    // 3. 1ms定时任务
    if(u8_1msFlag) {
        Calc_current_rms();               // ← 1000次循环
        temperatureSample();
        LoadData() + UART DMA;            // VOFA+数据
        Pack_ActiveReport() + CAN;        // 动态上报
        u8_1msFlag = 0;
    }
    
    // 4. 校准流程 (阻塞数秒)
    if(caliOn_flag) {
        enablePWM();
        calibrate();                      // ← 阻塞5+秒
        MeasureResistance();              // ← 阻塞1+秒
        MeasureInductance();              // ← 阻塞1+秒
        disablePWM();
        Write_MotorData();                // ← Flash擦写1-2秒
    }
    
    // 5. 错误报告
    errorReport();                        // ← 含HAL_Delay(200)
}
```

### 问题 I-07 [严重]: 定时任务与阻塞操作共存

**阻塞场景:**

| 操作 | 阻塞时间 | 影响 |
|------|---------|------|
| 扭矩测试 `HAL_Delay(5000)` ×10 | 50秒 | 100us/1ms定时任务全部停止 |
| 校准 `calibrate()` | ~5.4秒 | CAN回复、VOFA上报停止 |
| 电阻测量 `MeasureResistance()` | ~1.8秒 | 同上 |
| Flash写入 `Write_MotorData()` | 1-2秒 | 同上 |
| 错误报告 `HAL_Delay(200)` | 200ms | 100us/1ms任务延迟200ms |

**后果:**
- 校准过程中无CAN心跳回复 → 上位机判断通信丢失
- 扭矩测试中VOFA数据中断 → 无法观察测试过程
- `100us定时任务`中的CAN上报在主循环阻塞时完全停止

### 问题 I-08 [中等]: CAN_SendMessage 在主循环中调用

```c
// main.c:212
CAN_SendMessage(CanID_Upload, FOC_CAN_TxData, 8);
```

同时 `CAN_MsgProcess` 在 FDCAN 中断中也调用 `CAN_SendMessage`。两者可能同时操作FDCAN TX FIFO：

```
主循环:  CAN_SendMessage(0x700, ...) → HAL_FDCAN_AddMessageToTxFifoQ()
                                            ↑ 被FDCAN ISR抢占
FDCAN ISR: CAN_MsgProcess() → CAN_SendMessage(0x100+ID, ...)
                             → HAL_FDCAN_AddMessageToTxFifoQ()
```

HAL内部有 `__HAL_LOCK` 保护，但在中断嵌套场景下不可靠（同优先级无法抢占除外）。

---

## 6. 数据竞争分析

### 6.1 关键竞争场景

#### 场景1: Motor_P/Motor_W/Motor_Iq 在多个中断间共享

```
写入方:
  USART1 ISR (Priority 1):  Motor_Iq = atof(cmd_val);      // stm32h7xx_it.c:817
  FDCAN ISR  (Priority 1):  Motor_Iq = unpack_torque_cmd(); // can_rv.c:422

读取方:
  TIM1 ISR   (Priority 1):  p_motor_g->i_q_ref = Motor_Iq;  // stm32h7xx_it.c:420
```

USART1、FDCAN、TIM1 同为 Priority 1，不会互相抢占。但：
- 如果USART2 (Priority 0) 在TIM1读取Motor_Iq时触发 → 不影响（USART2不写Motor_Iq）
- **CAN与UART同时修改同一变量:** 如果CAN修改Motor_Iq的同时UART也在修改 → 由于同优先级不抢占，实际不会并发。但**逻辑上**最后写入者生效，可能不符合预期。

#### 场景2: FSMstate 在多个中断间共享

```c
// 无volatile声明 (但实际在FOC.c中声明为volatile)
volatile uint16_t FSMstate = REST_MODE;
```

写入方:
- USART1 ISR: `FSMstate = MOTOR_MODE;`
- FDCAN ISR: `FSMstate = MOTOR_MODE;`
- TIM1 ISR: `FSMstate = REST_MODE;` (回零完成)
- 主循环: `FSMstate = MOTOR_MODE;` (stateChange)

同优先级中断间不会竞争，但主循环与ISR间存在竞争：
```c
// 主循环 main.c:256
if(stateChange == 1) {
    FSMstate = MOTOR_MODE;   // 写FSMstate
    state_change = 1;        // 写state_change
    stateChange = 0;
}
// 如果TIM1 ISR在FSMstate写入后、state_change写入前触发
// TIM1 ISR看到新的FSMstate但旧的state_change → 不会进入初始化分支
```

#### 场景3: 编码器数据在USART2回调与TIM1间共享

```c
// USART2 DMA回调 (Priority 0) 写入:
angleInner = USART2_RX_DATA[2] << 16 | ...;  // 3字节组装
angleOutter = USART2_RX_DATA[5] << 16 | ...;

// TIM1 ISR (Priority 1) 读取:
p_encoder_g->mech_pos = angleOutter;  // encoder.c:98
p_encoder2_g->mech_pos = angleInner;  // encoder.c:119
```

由于USART2优先级高于TIM1，USART2回调可以在TIM1读取 `angleOutter` 和 `angleInner` 之间抢占，导致两个编码器数据不同步（一个新值一个旧值）。

### 问题 I-09 [严重]: angleInner/angleOutter 非原子更新

32位 `uint32_t` 在Cortex-M7上是原子读写，单个变量不会撕裂。但两个变量的更新不是原子的：

```c
// USART2回调可能在这两行之间抢占TIM1:
p_encoder_g->mech_pos = angleOutter;   // TIM1读取旧值
// ← USART2抢占，更新angleInner和angleOutter
p_encoder2_g->mech_pos = angleInner;   // TIM1读取新值
```

**影响:** 内外编码器数据来自不同采样时刻，力矩估算（如果使用双编码器差值）会出现瞬时跳变。

**修复:**
```c
// TIM1 ISR中关中断读取
__disable_irq();
uint32_t local_inner = angleInner;
uint32_t local_outter = angleOutter;
__enable_irq();
p_encoder_g->mech_pos = local_outter;
p_encoder2_g->mech_pos = local_inner;
```

---

## 7. TIM1 ISR 执行时间分析

### 7.1 执行路径拆解

```
TIM1_UP_IRQHandler (目标: <100us @ 10kHz, 实际可用200us @ 5kHz)
├── GPIO + DWT计时                    ~0.1us
├── 标志位设置                        ~0.1us
├── RS485DIR_TX (GPIO)                ~0.1us
├── startJADC()                       ~3us (HAL调用)
├── HAL_UART_Transmit_DMA()           ~5us
├── errorDiag()                       ~5us (含3路3点滑窗滤波 + 多次比较)
├── encoderSample()                   ~3us (含速度计算)
├── voltageSample()                   ~1us
├── currentSample()                   ~3us (含ADC读取 + 电流计算)
├── [状态机 switch]
│   ├── REST_MODE:                    ~0us (仅enter_menu检查)
│   ├── MOTOR_MODE:
│   │   ├── state_change初始化:       ~5us (含PD_FOC_clear + enablePWM)
│   │   ├── CAN超时检查:             ~1us
│   │   ├── CurrentLoop():            ~15us (Clark+Park+PI+SVPWM)
│   │   ├── VelocityLoop():           ~3us (PI计算, 每4次)
│   │   └── PositionLoop():           ~3us (PI计算, 每4次)
│   ├── CALIBRATION_MODE:             ~0us (仅设flag)
│   ├── HOMING_MODE:                  与MOTOR_MODE类似
│   └── PP_MODE:                      与MOTOR_MODE类似 + 轨迹插补
├── HAL_TIM_IRQHandler()              ~2us
├── TIM1->SR = 0                      ~0.1us
└── DWT计时 + GPIO                    ~0.1us

总计 (MOTOR_MODE, 典型):  ~35-45us
总计 (MOTOR_MODE, 最坏):  ~55us (含速度环+位置环)
```

### ISR 执行时间评估

| 模式 | 典型耗时 | 占用率(10kHz) |
|------|---------|--------------|
| REST_MODE | ~20us | 20% |
| MOTOR_MODE (电流环) | ~35us | 35% |
| MOTOR_MODE (全环) | ~45us | 45% |
| CALIBRATION_MODE | ~20us | 20% |

**评估:** ISR运行在10kHz（RCR=1中心对齐模式），CPU占用率偏高但在480MHz主频下仍在合理范围。

**但以下路径会导致超时:**
- PP轨迹模式下 `printf("[OK] Planner Successful\r\n")` → 在ISR中调用printf → 可能阻塞50-100us
- HOMING_MODE自动完成时调用 `disablePWM()` + 修改 `FSMstate` → 无风险

---

## 8. 推荐的中断优先级方案

```c
// 建议配置 (Priority Group 4: 4 bits preempt, 0 bits sub)

// 最高优先级: FOC电流环 (绝对不可被打断)
HAL_NVIC_SetPriority(TIM1_UP_IRQn,       0, 0);

// 次高: 编码器数据接收 (DMA完成后快速处理)
HAL_NVIC_SetPriority(DMA1_Stream0_IRQn,  1, 0);  // USART2 RX DMA
HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,  2, 0);  // USART2 TX DMA
HAL_NVIC_SetPriority(USART2_IRQn,        1, 0);  // USART2 IDLE

// 中等: CAN通信
HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,    3, 0);

// 低: 调试接口
HAL_NVIC_SetPriority(USART1_IRQn,        5, 0);  // 调试串口
HAL_NVIC_SetPriority(USART6_IRQn,        6, 0);  // VOFA+
HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,  6, 0);  // VOFA+ DMA

// 最低: 系统时基
// SysTick: 15 (默认)
```

---

## 9. 推荐的架构改进

### 9.1 控制环调度重构

```
当前架构 (所有逻辑在TIM1 ISR中):
┌─────────────────────────────────────────┐
│ TIM1_UP_IRQHandler (10kHz)              │
│ ┌─ ADC采样                             │
│ ├─ 编码器采样                           │
│ ├─ 状态机                               │
│ ├─ 位置环 (每4次)                       │
│ ├─ 速度环 (每4次)                       │
│ ├─ 电流环                               │
│ ├─ SVPWM                               │
│ ├─ 错误诊断                             │
│ ├─ RS485发送                            │
│ └─ printf (多处)                        │
└─────────────────────────────────────────┘

建议架构 (分层调度):
┌─────────────────────────────────┐
│ TIM1_UP_IRQHandler (Priority 0) │  ← 仅电流环核心
│ ┌─ ADC读取                     │     执行时间: <15us
│ ├─ 编码器读取                   │
│ ├─ Clark + Park                │
│ ├─ 电流环PI                    │
│ ├─ SVPWM                      │
│ └─ 设置标志位                  │
└─────────────────────────────────┘
              ↓ 标志位
┌─────────────────────────────────┐
│ 主循环 while(1)                 │  ← 非实时任务
│ ┌─ 速度环 (基于标志位)          │
│ ├─ 位置环 (基于标志位)          │
│ ├─ 状态机转换                  │
│ ├─ 错误诊断                    │
│ ├─ CAN/UART通信               │
│ ├─ 温度采样                    │
│ └─ Flash操作                   │
└─────────────────────────────────┘
```

**优点:**
- TIM1 ISR执行时间压缩到15us以内
- 主循环中的阻塞操作不影响电流环
- 易于添加新功能

**缺点:**
- 速度环/位置环延迟增加1个电流环周期
- 需要更多标志位和数据缓冲

### 9.2 ADC硬件触发

```c
// 修改ADC配置: 使用TIM1 TRGO触发
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

// TIM1 ISR中移除 startJADC() 调用
// ADC注入转换由TIM1更新事件自动触发
// 在TIM1 ISR中只需读取JDR寄存器
```

---

## 10. 问题汇总

| 编号 | 严重度 | 问题 | 修复工作量 |
|------|--------|------|-----------|
| **I-01** | **致命** | USART2优先级(0)高于TIM1(1)，RS485可抢占FOC | 改1行 |
| **I-02** | **严重** | FDCAN与TIM1同优先级，CAN处理可延迟FOC | 改1行 |
| **I-03** | **严重** | USART1与TIM1同优先级，调试串口可延迟FOC | 改1行 |
| ~~I-04~~ | ~~严重~~ | ~~RCR=1导致ISR频率5kHz，代码按10kHz计算~~ | **已确认正确**: 中心对齐模式RCR=1实际ISR=10kHz |
| **I-05** | **中等** | delay_us可能延时精度有误 | 1行 |
| **I-06** | **严重** | ADC软件触发，采样时刻不确定 | 改2行ADC配置 |
| **I-07** | **严重** | 主循环阻塞50秒(扭矩测试)，定时任务停止 | 重构为非阻塞 |
| **I-08** | **中等** | CAN_SendMessage在主循环和ISR中同时调用 | 统一到一处 |
| **I-09** | **严重** | angleInner/angleOutter非原子更新 | 加关中断保护 |
| **I-10** | **低** | ISR中printf/delay_us调用 | 移到主循环 |

### 紧急修复（改配置即可，不动逻辑）:

```c
// 1. 修复优先级 (3行)
HAL_NVIC_SetPriority(TIM1_UP_IRQn,       0, 0);  // FOC最高
HAL_NVIC_SetPriority(USART2_IRQn,        1, 0);  // 编码器次之
HAL_NVIC_SetPriority(DMA1_Stream0_IRQn,  1, 0);
HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,  2, 0);
HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,    3, 0);  // CAN降低
HAL_NVIC_SetPriority(USART1_IRQn,        5, 0);  // 调试最低

// 2. 修复ADC触发 (adc.c中修改2行)
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```

---

*Report generated by interrupt architecture review - Claude*
