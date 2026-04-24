# STM32H743 FOC Motor Driver - Full Project Code Review

**Project:** FIVE (STM32H743VITx FOC Motor Driver)  
**Date:** 2026-04-17  
**Scope:** Core/, UserSrc/, FOC/ 全部用户源码 (49 files)  

---

## 1. 问题汇总

| 编号 | 严重度 | 模块 | 问题 | 文件:行号 |
|------|--------|------|------|-----------|
| **B-01** | **致命** | FOC | D轴电流滤波公式写错，使用Q轴滤波值代替D轴 | FOC.c:302 |
| **B-02** | **致命** | Flash | 中断上下文执行Flash擦除/编程，阻塞电流环1~2秒 | can_rv.c:511,812; stm32h7xx_it.c:640 |
| **B-03** | **致命** | Flash | Flash操作无重入保护 | flash.c:42-125 |
| **B-04** | **严重** | CAN | MIT模式 v_des/t_ff 解包用了 min 代替 max，范围退化为0 | can_rv.c:455-456 |
| **B-05** | **严重** | CAN | pack_reply 中 MOS 温度字段错误复制为电机温度 | can_rv.c:318-319 |
| **B-06** | **严重** | Motor | Motor_Init 中 Err1 被覆盖赋值为 MotorWarning_Nomal | motor.c:83-85 |
| **B-07** | **严重** | Motor | Calc_current_rms 采样1000次同一个瞬时值，结果等于瞬时值 | motor.c:354-368 |
| **B-08** | **严重** | Diag | 温度诊断逻辑顺序错误，Warning阈值 < Over阈值但先判断 | Diag.c:114-148, 159-194 |
| **B-09** | **严重** | Flash | 无掉电保护/数据校验 | flash.c:92-157 |
| **B-10** | **严重** | Flash | FDCAN_ID 写入Flash但从未读回 | flash.c:67 vs 127-157 |
| **B-11** | **中等** | CAN | 中断上下文中错误清除逻辑：Err1先赋值再被覆盖 | can_rv.c:517-519 |
| **B-12** | **中等** | FOC | delay_us 停止定时器写错寄存器 | FOC.c:631 |
| **B-13** | **中等** | FOC | Uint2Float 通过栈上局部数组做类型双关，存在严格别名风险 | FOC.c:515-523 |
| **B-14** | **中等** | ISR | TIM1中断中大量 printf/delay_us 调用 | stm32h7xx_it.c:598-620 |
| **B-15** | **中等** | Motor | currentSample 未计算 phase_a_current（仅用 -b-c 替代） | motor.c:335 |
| **B-16** | **中等** | Main | 主循环中 HAL_Delay(5000) 阻塞5秒（扭矩测试） | main.c:203 |
| **B-17** | **中等** | Main | SweepSine 在 Read_MotorData 之前就初始化并启动 | main.c:188-191 vs 157 |
| **B-18** | **低** | CAN | CAN_SendMessage 对未知 len 值无 default 处理 | can_rv.c:40-48 |
| **B-19** | **低** | ISR | cmd_val[8] 缓冲区无溢出检查，char_count 可超过7 | stm32h7xx_it.c:804 |
| **B-20** | **低** | Flash | 校准后无条件写Flash | main.c:384 |
| **B-21** | **低** | Linker | scatter文件代码区覆盖整个2MB，未为数据区预留 | FIVE.sct:5-6 |
| **B-22** | **建议** | 全局 | 大量被注释掉的代码未清理 | 多文件 |
| **B-23** | **建议** | 全局 | 全局变量过多，缺少 volatile 声明（ISR与主循环共享） | 多文件 |

---

## 2. 致命问题详细分析

### B-01 [致命] D轴电流滤波公式错误

**位置:** `FOC/FOC.c:302`

```c
// 第301行 (正确)
p_motor_g->Q_axis_current_filt = (0.4f * p_motor_g->Q_axis_current_filt) + (0.6f * p_motor_g->Q_axis_current);

// 第302行 (BUG: 使用了Q轴滤波值而非D轴)
p_motor_g->D_axis_current_filt = (0.4f * p_motor_g->Q_axis_current_filt) + (0.6f * p_motor_g->D_axis_current);
//                                              ^^^^^^^^^^^^^^^^^^^^^^^^^
//                                              应为 D_axis_current_filt
```

**影响:** D轴电流滤波值混入了Q轴成分。当前代码中 `D_axis_current_filt` 未被电流环直接使用（注释掉了），但如果启用带滤波的电流环反馈（第311行），将导致dq轴耦合，电机运行不稳定。

**修复:**
```c
p_motor_g->D_axis_current_filt = (0.4f * p_motor_g->D_axis_current_filt) + (0.6f * p_motor_g->D_axis_current);
```

---

### B-02/B-03 [致命] Flash中断安全问题

已在 `Flash_Code_Review.md` 中详细描述。`Write_MotorData` 在FDCAN中断回调链中被调用，128KB扇区擦除耗时1~2秒，期间FOC电流环完全停摆。

---

## 3. 严重问题详细分析

### B-04 [严重] MIT模式 CAN 解包范围错误

**位置:** `UserSrc/Src/can_rv.c:455-456`

```c
controller.v_des = uint32_to_float(v_raw, w_min, w_min, 16) * GR;
//                                        ^^^^^  ^^^^^
//                                        min 和 max 都用了 w_min!

controller.t_ff = uint32_to_float(t_raw, iq_min, iq_min, 16);
//                                       ^^^^^^  ^^^^^^
//                                       min 和 max 都用了 iq_min!
```

**影响:** `uint32_to_float` 中 `span = x_max - x_min = 0`，导致除零或结果恒为 `iq_min`/`w_min`。MIT PD模式下速度前馈和力矩前馈完全失效。

**修复:**
```c
controller.v_des = uint32_to_float(v_raw, w_min, w_max, 16) * GR;
controller.t_ff = uint32_to_float(t_raw, iq_min, iq_max, 16);
```

---

### B-05 [严重] CAN回复包 MOS温度字段错误

**位置:** `UserSrc/Src/can_rv.c:318-319`

```c
tData[7] = temp_Motor;   // 电机温度
tData[8] = temp_Motor>>8;
tData[9] = temp_Motor;   // BUG: 应为 temp_Mos
tData[10] = temp_Motor>>8; // BUG: 应为 temp_Mos
```

同样的错误出现在 `Pack_ActiveReport` (`can_rv.c:384-385`)。

**影响:** 上位机读到的 MOS 温度实际是电机温度，无法检测 MOS 过温。`temp_Mos` 变量被计算但从未使用（编译器应有 unused 警告）。

**修复:**
```c
tData[9] = temp_Mos;
tData[10] = temp_Mos>>8;
```

---

### B-06 [严重] Motor_Init 错误状态初始化

**位置:** `FOC/motor.c:83-85`

```c
p_motor_g->Err1 = MotorErr1_Nomal;   // 第83行: 赋值 Err1
p_motor_g->Err2 = MotorErr1_Nomal;   // 第84行: 赋值 Err2
p_motor_g->Err1 = MotorWarning_Nomal; // 第85行: 覆盖 Err1!
//         ^^^^
//         应为 p_motor_g->Warning
```

**影响:** `Warning` 字段未被初始化，`Err1` 被错误覆盖为 `MotorWarning_Nomal`。

同样的错误出现在 `can_rv.c:517-519`（错误清除命令0xFD）。

---

### B-07 [严重] RMS电流计算错误

**位置:** `FOC/motor.c:354-368`

```c
void Calc_current_rms(void) {
    float sumA = 0, sumB = 0, sumC = 0;
    for(int i=0; i<SAMPLE_CNT; i++) {  // SAMPLE_CNT = 1000
        sumA += p_motor_g->phase_a_current * p_motor_g->phase_a_current;
        // ^^^ 1000次循环中读取同一个值(非volatile)，编译器可能优化为 sumA = 1000 * val^2
    }
    p_motor_g->phase_a_Current_RMS = sqrt(sumA / SAMPLE_CNT);
    // = sqrt(1000 * val^2 / 1000) = |val| ← 等于瞬时值绝对值，不是RMS
}
```

**问题:**
1. 循环内读取同一个全局变量1000次不等待新的ADC采样，所有值相同
2. 使用 `sqrt()` (双精度) 而非 `sqrtf()` (单精度)，H7 FPU 不支持硬件双精度运算
3. 该函数在主循环1ms定时中调用，1000次循环不含任何延时，约消耗 ~10us 做无意义计算

**修复建议:** 在ADC采样中断中逐周期累加平方值，在1ms定时中取均值计算RMS。

---

### B-08 [严重] 温度诊断阈值判断顺序错误

**位置:** `UserSrc/Src/Diag.c:114-148`

```c
// TEMP_MOSWARNING = 90.0f, TEMP_MOSOver = 100.0f
if (TEMPERATURE_MOSFET > TEMP_MOSWARNING)       // >90: 进入warning分支
{
    // ... set warning flag
}
else if (TEMPERATURE_MOSFET > TEMP_MOSOver)      // >100: 永远不会执行!
{                                                 // 因为 >100 必然 >90，已被上面捕获
    disablePWM();  // 关断PWM - 永远执行不到!
}
```

**影响:** MOS/电机温度超过100度时，只触发Warning而**不会触发Over保护（不会关断PWM）**。这是安全隐患。

**修复:** 交换判断顺序，先判断 Over 再判断 Warning：
```c
if (TEMPERATURE_MOSFET > TEMP_MOSOver) {       // 先判断严重的
    // Over protection: disablePWM
}
else if (TEMPERATURE_MOSFET > TEMP_MOSWARNING) { // 再判断轻微的
    // Warning only
}
```

---

## 4. 中等问题详细分析

### B-12 delay_us 停止定时器写错寄存器

**位置:** `FOC/FOC.c:631`

```c
htim6.Instance->CNT &= ~TIM_CR1_CEN; // BUG: 写的是CNT寄存器，不是CR1
//              ^^^                    // 应该是 htim6.Instance->CR1 &= ~TIM_CR1_CEN;
```

**影响:** 定时器不会被停止，CNT值被意外修改。

---

### B-14 中断中调用 printf/delay_us

**位置:** `Core/Src/stm32h7xx_it.c:598-620` (USART1 IRQ中的 REST_MODE 菜单处理)

在USART1接收中断中，收到'm'字符后调用了多次 `printf` 和 `delay_us`。`printf` 通常走UART DMA/轮询发送，在中断上下文中可能耗时几十到几百us，阻塞其他低优先级中断。

**建议:** 中断中只设置标志/缓存数据，printf 放到主循环处理。

---

### B-15 三相电流采样 phase_a 由计算得出

**位置:** `FOC/motor.c:335`

```c
p_motor_g->phase_a_current = -p_motor_g->phase_c_current - p_motor_g->phase_b_current;
```

a相电流由 `-b-c` 计算得出而非直接采样。同时 `phase_a_current_actual` (第336行) 单独计算了ADC采样值但未参与FOC运算。这意味着a相电流完全依赖b、c相ADC的精度，误差会叠加。

考虑到三相电流之和理论为零（基尔霍夫），这种做法在两电阻采样方案中是正确的。但本工程实际有三个ADC通道 (JDR1/JDR2/JDR3)，建议利用第三路做冗余校验。

---

### B-19 UART命令缓冲区溢出

**位置:** `Core/Src/stm32h7xx_it.c:804`

```c
char cmd_val[8] = {0};  // 只有8字节
// ...
cmd_val[char_count-1] = c;  // char_count无上限检查
char_count++;
```

如果用户输入超过8个字符的参数值，`char_count` 超过8后将越界写入栈空间。

**修复:**
```c
if (char_count < 8) {
    cmd_val[char_count-1] = c;
}
char_count++;
```

---

## 5. 架构与设计问题

### 5.1 ISR过于臃肿

`TIM1_UP_IRQHandler` (~280行) 包含了完整的状态机、FOC电流环、速度环、位置环、轨迹规划。虽然FOC控制通常在PWM中断中执行以保证实时性，但以下内容不应在ISR中：

- `printf` 调用 (行467, 375-376)
- `enter_menu_state()` / `enter_setup_state()` 调用 (行342, 490)
- 轨迹规划计算 (行452-470)

建议将非时间关键的操作移至主循环，ISR只做信号采样和电流环计算。

### 5.2 全局变量无 volatile 声明

以下变量在ISR和主循环间共享，但未声明 volatile：

| 变量 | 写入位置 | 读取位置 |
|------|---------|---------|
| `Motor_Iq` | USART1 ISR / CAN ISR | TIM1 ISR |
| `Motor_W` | USART1 ISR / CAN ISR | TIM1 ISR |
| `Motor_P` | USART1 ISR / CAN ISR | TIM1 ISR |
| `caliOn_flag` | TIM1 ISR | Main loop |
| `svpwm_on` | TIM1 ISR | CurrentLoop (TIM1 ISR) |
| `bDynamMode` | CAN ISR | Main loop |
| `bSaveDataFlag` | CAN ISR | CAN ISR (局部OK) |

编译器在高优化级别下可能缓存这些变量，导致值更新不可见。

### 5.3 过多注释掉的代码

`can_rv.c` (974行) 中约 40% 是注释掉的旧代码；`user_config.c` (195行) 中约 70% 是注释掉的旧PI参数。这严重降低了可读性和可维护性。建议：
- 删除不再使用的代码（git有历史记录）
- 保留有价值的注释，删除纯"试过但不用"的参数组

---

## 6. 安全与可靠性

### 6.1 看门狗缺失

整个项目没有使用独立看门狗 (IWDG) 或窗口看门狗 (WWDG)。如果程序跑飞或死循环（如 `MeasureInductance` 中的 `while(measure_time);` 死等），电机将持续输出最后的PWM占空比，可能造成损坏。

**建议:** 启用 IWDG，在主循环和TIM1 ISR中分别喂狗。

### 6.2 Error_Handler 死循环无安全动作

**位置:** `Core/Src/main.c:522-531`

```c
void Error_Handler(void) {
    __disable_irq();
    while (1) { }  // 关中断后死循环，PWM输出状态未知
}
```

HAL初始化失败时进入死循环，但PWM可能已部分初始化。应先关断PWM输出、拉低使能引脚后再死循环。

### 6.3 硬件过流检测缺失

代码中只有软件过流保护 (`errorDiag` 中通过ADC采样判断)，没有看到硬件过流（比较器/BKIN引脚）的配置和处理。对于高功率电机驱动，纯软件保护存在响应延迟风险。

---

## 7. Flash操作问题

已在 `Flash_Code_Review.md` 中详细分析，此处概述关键问题：

| 编号 | 问题 | 严重度 |
|------|------|--------|
| B-02 | ISR中直接调用Write_MotorData | 致命 |
| B-03 | 无重入/中断保护 | 致命 |
| B-09 | 无掉电保护、无magic/CRC校验 | 严重 |
| B-10 | FDCAN_ID写入但未读回 | 严重 |
| B-20 | 校准后无条件写Flash | 低 |
| B-21 | Scatter文件未限制代码区 | 低 |

---

## 8. 地址与资源

| 资源 | 配置 | 状态 |
|------|------|------|
| Flash代码区 | 0x08000000, 2MB (全范围) | 实际115KB, 无冲突 |
| Flash数据区 | 0x081E0000, Bank2 Sector7 | 128字节实际使用, 对齐正确 |
| DTCM RAM | 0x20000000, 128KB | 正常 |
| AXI SRAM | 0x24000000, 512KB | flash_buffer在此区域 |
| 时钟 | HSE 12MHz -> PLL 480MHz | VOS0, Flash Latency 4 |
| PWM频率 | 10kHz (TIM1) | 中心对齐模式 |
| ADC | 注入模式, TIM1触发 | 3通道电流 + 母线电压 + 温度 |

---

## 9. 修复优先级

### P0 - 立即修复（影响安全/功能正确性）

| 编号 | 问题 | 预估工作量 |
|------|------|-----------|
| B-01 | D轴滤波公式错误 | 1min |
| B-04 | MIT解包 min/max 参数错误 | 1min |
| B-05 | MOS温度字段复制错误 | 1min |
| B-06 | Motor_Init Warning 赋值错误 | 1min |
| B-08 | 温度诊断判断顺序错误（安全隐患） | 5min |
| B-02 | ISR中Flash操作改标志位 | 30min |
| B-03 | Flash操作加中断保护 | 15min |

### P1 - 尽快修复

| 编号 | 问题 | 预估工作量 |
|------|------|-----------|
| B-07 | RMS计算重构 | 1h |
| B-09 | Flash数据校验 | 1h |
| B-10 | FDCAN_ID读回 | 5min |
| B-11 | CAN错误清除逻辑 | 5min |
| B-12 | delay_us修复 | 1min |
| B-19 | cmd_val溢出保护 | 5min |

### P2 - 计划修复

| 编号 | 问题 | 预估工作量 |
|------|------|-----------|
| B-14 | ISR中去除printf | 2h |
| B-16 | 扭矩测试非阻塞化 | 1h |
| B-18 | CAN_SendMessage len校验 | 5min |
| B-20 | 校准条件性Flash写入 | 15min |
| B-21 | Scatter文件修正 | 5min |
| 6.1 | 添加IWDG | 1h |
| 6.2 | Error_Handler安全关断 | 15min |

### P3 - 长期优化

| 项目 | 描述 |
|------|------|
| B-22 | 清理注释掉的代码 |
| B-23 | 共享变量添加volatile |
| 5.1 | ISR拆分，非实时部分移至主循环 |
| 5.3 | user_config.c清理旧参数 |

---

## 10. 涉及文件清单

| 文件 | 主要问题 |
|------|---------|
| `FOC/FOC.c` | B-01(D轴滤波), B-12(delay_us), B-13(类型双关) |
| `FOC/motor.c` | B-06(初始化覆盖), B-07(RMS), B-15(a相电流) |
| `FOC/calibration.c` | 校准流程（与Flash问题关联） |
| `FOC/encoder.c` | 硬编码默认值（Flash回退值） |
| `UserSrc/Src/can_rv.c` | B-04(MIT解包), B-05(温度), B-11(错误清除), B-18(len) |
| `UserSrc/Src/flash.c` | B-02, B-03, B-09, B-10 (详见Flash报告) |
| `UserSrc/Src/Diag.c` | B-08(温度判断顺序) |
| `Core/Src/main.c` | B-16(阻塞), B-17(初始化顺序), B-20 |
| `Core/Src/stm32h7xx_it.c` | B-02(Flash ISR), B-14(printf ISR), B-19(溢出) |
| `MDK-ARM/FIVE/FIVE.sct` | B-21(scatter) |

---

*Report generated by code review - Claude*
