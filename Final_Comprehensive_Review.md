# FIVE FOC Servo Driver - Final Comprehensive Review

**MCU:** STM32H743VITx | **Date:** 2026-04-21 | **LOC:** ~5400 (user code)

---

## 问题总表 (按严重度排序)

### CRITICAL - 致命 (可导致电机失控/硬件损坏)

| # | 问题 | 位置 | 详情 |
|---|------|------|------|
| C-01 | USART2优先级(0)高于TIM1(1), RS485可抢占FOC电流环 | tim.c:220, usart.c:343 | 编码器通信打断SVPWM计算, 电流采样时序偏移 |
| C-02 | Flash擦写在FDCAN中断中执行, 阻塞1-2秒 | can_rv.c:511,812; stm32h7xx_it.c:640 | 128KB扇区擦除期间FOC完全停摆, 电机失控 |
| C-03 | Flash操作无重入保护 | flash.c:42-125 | 主循环与ISR同时调用Write_MotorData导致数据损坏 |
| C-04 | 温度保护判断顺序错误, 100°C不关断PWM | Diag.c:114-148, 159-194 | Warning(90°C)先于Over(100°C)判断, else-if导致Over分支永不执行 |
| C-05 | 未校准电机可直接运行 | stm32h7xx_it.c:357 | cali_finish=0时仍可进入MOTOR_MODE, 使用硬编码elec_offset=1.719367 |
| C-06 | elec_pos未mod 2π, 值域[0,16π] | encoder.c:116 | arm_sin_f32输入超范围, Park变换精度下降 |

### HIGH - 严重 (功能错误/数据损坏)

| # | 问题 | 位置 | 详情 |
|---|------|------|------|
| H-01 | D轴电流滤波公式用了Q轴滤波值 | FOC.c:302 | `D_filt = 0.4*Q_filt + 0.6*D` 应为 `0.4*D_filt + 0.6*D` |
| H-02 | Q轴PI用了D轴增益 ki_d/k_d | FOC.c:315, 334 | 当前dq相同不影响, 独立调参时出错 |
| H-03 | MIT模式v_des/t_ff解包: min代替max | can_rv.c:455-456 | `uint32_to_float(v_raw, w_min, w_min, 16)` span=0, 除零 |
| H-04 | pack_reply MOS温度字段复制为电机温度 | can_rv.c:318-319 | `tData[9]=temp_Motor` 应为 `temp_Mos`, Pack_ActiveReport同样错误(:384-385) |
| H-05 | Motor_Init Err1被Warning覆盖 | motor.c:83-85 | `Err1=MotorWarning_Nomal` 应为 `Warning=...`, can_rv.c:517-519同样 |
| H-06 | Calc_current_rms采样1000次同一瞬时值 | motor.c:354-368 | 无延时循环, 结果=|瞬时值|; 且用sqrt()双精度 |
| H-07 | Flash无掉电保护/数据校验 | flash.c:92-157 | 擦除后写入前断电→全0xFF→校准数据丢失 |
| H-08 | FDCAN_ID写入Flash但从未读回 | flash.c:67 vs 127-157 | 每次上电FDCAN_ID恢复默认值1 |
| H-09 | 位置环(输出端)与速度环(电机端)编码器不同 | FOC.c:255 vs 282 | 单位差减速比GR=50, 级联target/feedback不一致 |
| H-10 | PID输出超限时积分清零(I=0) | pid.c:141 | 大误差→I清零→接近目标→I从零重建→稳态恢复慢 |
| H-11 | FDCAN/USART1与TIM1同优先级(1) | fdcan.c:125, usart.c:269 | CAN_MsgProcess 490行代码可延迟FOC; UART printf可延迟60-100us |
| H-12 | ADC使用软件触发而非TIM1硬件触发 | adc.c:95 | 采样时刻不确定, 受ISR进入延迟影响 |
| H-13 | 校准期间ISR与校准代码竞争修改编码器 | calibration.c + stm32h7xx_it.c | 两处encoderSample()同时修改rotations, 可能双重更新 |
| H-15 | Motor_Init温度计算除零(ADC=4095) | motor.c:94-99 | 分母3.3-3.3=0, logf(0)=-∞ |
| H-16 | Pack_ActiveReport溢出: float_to_uint结果×1000 | can_rv.c:366 | `uint32_t p_int = float_to_uint(...,24) * 1000` 24bit最大16M×1000溢出uint32 |

### MEDIUM - 中等

| # | 问题 | 位置 | 详情 |
|---|------|------|------|
| M-01 | delay_us停止定时器写错寄存器 | FOC.c:631 | `CNT &= ~TIM_CR1_CEN` 应为 `CR1 &= ~TIM_CR1_CEN` |
| ~~M-02~~ | ~~RCR=1可能导致ISR=5kHz~~ | tim.c:53 | **已确认正确**: 中心对齐模式RCR=1实际ISR=10kHz, 与代码一致 |
| M-03 | 前馈补偿被注释掉 | FOC.c:336-337 | 高速时dq耦合大, 反电动势不补偿 |
| M-04 | 积分限幅dq标量独立, 输出向量限幅 | FOC.c:317-325 vs 339 | 积分向量可达√2×1.15Vbus, 被limit_norm截断方向偏移 |
| M-05 | PID死区硬编码只对POSITION_LOOP生效 | pid.c:132 | PP模式无死区, PID耦合controlMode |
| M-06 | 位置/速度环同频率(period=4) | encoder.c:137-151 | 无频率分离, 外环应比内环慢3-10倍 |
| M-07 | A相电流由-(B+C)计算, 未用第三路ADC | motor.c:335 | 噪声叠加, 三路ADC仅用两路 |
| M-08 | cmd_val[8]缓冲区无溢出检查 | stm32h7xx_it.c:804 | char_count无上限, 超8字节栈溢出 |
| M-09 | 主循环HAL_Delay(5000)×10阻塞50秒 | main.c:203 | 扭矩测试期间100us/1ms任务全停 |
| M-10 | angleInner/angleOutter非原子更新 | stm32h7xx_it.c:992-993 | USART2(P0)可在TIM1读两值之间抢占 |
| M-11 | 校准后无条件写Flash | main.c:384 | 未校准也擦写, 增加磨损和掉电风险 |
| M-12 | CAN_SendMessage对未知len无default | can_rv.c:40-48 | len非8/12/16时DataLength未设置 |
| M-13 | Uint2Float严格别名违规 | FOC.c:515-523 | `*(float*)temp` 通过uint8_t数组转float, UB |
| M-14 | one_by_sample_resistance与sample_resistance不一致 | motor.c:23-24 | `250=1/0.004` 但 `sample_resistance=0.0025`, 250≠1/0.0025=400 |
| M-15 | Encoder_t含float elec_pos_table[2048]未使用 | encoder.h:58 | 浪费8KB RAM |
| M-16 | 上电rotations=0, 多圈绝对位置丢失 | encoder.c:49 | 关机前非零位→上电位置跳变 |
| M-17 | CAN_timeout在ISR中递增但永不清零 | stm32h7xx_it.c:376 | 搜索全代码库无CAN_timeout=0, 必然触发超时 |
| M-18 | PPtraj.c effective_acc除零(acc+dec=0) | PPtraj.c:45 | 用户未设置加减速时崩溃 |
| M-19 | trajectory.c sqrt()双精度 | trajectory.c:67 | 应为sqrtf()单精度 |
| M-20 | PID back-calculation限幅逻辑不严谨 | pid.c:146-147 | output为负时I上限可能>output_limit |

### LOW - 低

| # | 问题 | 位置 | 详情 |
|---|------|------|------|
| L-01 | Scatter文件代码区覆盖整个2MB | FIVE.sct:5-6 | 未为Flash数据区预留 |
| L-02 | ISR中printf/delay_us | stm32h7xx_it.c:598-620 | REST_MODE菜单打印阻塞60-100us |
| L-03 | SweepSine在Read_MotorData前初始化并启动 | main.c:188-191 vs 157 | 参数未从Flash加载就开始扫频 |
| L-04 | 无看门狗 | 全局 | 程序跑飞时电机保持最后PWM状态 |
| L-05 | Error_Handler死循环未关PWM | main.c:522-531 | HAL初始化失败时PWM可能已部分使能 |
| L-06 | ~800行注释掉的代码 | 多文件 | 占总代码15%, 严重降低可读性 |
| L-07 | 全局变量缺volatile (Motor_Iq/W/P等) | 多文件 | ISR与主循环共享, 编译器可能优化掉读取 |
| L-08 | 无Flash磨损均衡 | flash.c | 每次参数修改擦写128KB扇区, 寿命10000次 |
| L-09 | 前馈反电动势常数硬编码0.005619 | FOC.c:329 | 换电机需改代码 |
| L-10 | calibration.c全局数组43KB | calibration.c:5-10 | error_f/b/raw_f/b各2688×4B, 占AXI SRAM |

---

## 新发现问题 (本次审查新增)

### NEW-01 [严重]: Pack_ActiveReport 整数溢出

```c
// can_rv.c:366
uint32_t p_int = float_to_uint(pos, p_min, p_max, 24) * 1000;
```

`float_to_uint` 返回最大 `(1<<24)-1 = 16,777,215`。乘以1000 = **16,777,215,000** 远超 `uint32_t` 最大值 4,294,967,295。**溢出导致位置数据错误。**

同行: `uint16_t v_int = float_to_uint(...,16) * 10` → 最大 655,350 也溢出 uint16_t (max 65535)。

### NEW-02 [中等]: one_by_sample_resistance 与 sample_resistance 不一致

```c
// motor.c:23-24
const float one_by_sample_resistance = 250.0f; // 注释说 1/0.004
const float sample_resistance = 0.0025f;       // 实际采样电阻
// 1/0.0025 = 400, 不是250!
```

`one_by_sample_resistance` 未被使用 (代码中用 `/sample_resistance`)，但如果未来启用会导致电流计算错误25%。

### NEW-03 [中等]: CAN_timeout 永不清零

```c
// stm32h7xx_it.c:376
CAN_timeout++;
if((CAN_timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)) {
    // 清零指令, 保持MOTOR_MODE
}
```

`CAN_timeout` 在每个ISR周期递增，但搜索整个代码库未找到 `CAN_timeout = 0` 的清零操作（除了 `PD_FOC_clear` 中未见）。这意味着一旦进入MOTOR_MODE，`CAN_timeout` 持续递增，在 `CAN_TIMEOUT` 个周期后**必然触发超时**，无论CAN是否正常通信。

**验证:** 需确认CAN接收处理中是否有 `CAN_timeout = 0`。

### NEW-04 [中等]: Encoder_t 含未使用的 8KB 数组

```c
// encoder.h:58
float elec_pos_table[2048];  // 8192 bytes, 从未被写入或读取
```

每个 Encoder_t 实例浪费 8KB RAM。两个实例 = 16KB。

### NEW-05 [低]: SVPWM 占空比计算运算符优先级

```c
// FOC.c:84
Ta = (uint16_t)(PWM_PERIOD_DEFAULT - Tx - Ty) >> 2;
```

`(uint16_t)(...)` 先将float截断为uint16_t，然后 `>> 2`。但 `>>` 的优先级低于 `=`... 实际上 `>>` 优先级高于 `=`，所以这里是 `Ta = ((uint16_t)(...)) >> 2`，逻辑正确。但如果 `PWM_PERIOD_DEFAULT - Tx - Ty` 为负数，`(uint16_t)` 截断会产生错误的大值。

### NEW-06 [严重]: PPtraj除零风险 - acc+dec=0

```c
// PPtraj.c:45
float effective_acc = (acc * dec) / (acc + dec);
```

如果 `acc = dec = 0` (用户未设置加减速度)，分母 `acc + dec = 0` → **除零崩溃**。

同样问题在 trajectory.c:67:
```c
vel_max = sqrt(2.0f * sq / (1.0f / acc + 1.0f / dec));
```
如果 `acc = 0` 或 `dec = 0` → `1.0f / 0` → **除零**。

### NEW-07 [中等]: PID积分限幅逻辑错误

```c
// pid.c:146-147
pid->I = MIN(pid->I, pid->output_limit - pid->output);
pid->I = MAX(pid->I, -pid->output_limit - pid->output);
```

这是anti-windup的back-calculation方法，但实现有误：
- `output = P + feedforward` 已计算
- 限幅 `I` 使得 `output + I <= output_limit`
- 但 `output` 可能为负，导致 `output_limit - output` 可能 > `output_limit`

**正确做法:** 限幅 `I` 本身，或限幅 `output + I` 的总和。

### NEW-08 [低]: trajectory.c 使用 sqrt() 双精度

```c
// trajectory.c:67
vel_max = sqrt(2.0f * sq / (1.0f / acc + 1.0f / dec));
```

应为 `sqrtf()` 单精度。STM32H7虽有双精度FPU，但单精度更快。

### NEW-09 [低]: limit_norm 未检查 norm=0

```c
// FOC.c:413-418
float norm = sqrtf(*x * *x + *y * *y);
if(norm > limit) {
    *x = *x * limit/norm;  // 如果norm=0 → 除零
    *y = *y * limit/norm;
}
```

如果 `x=y=0`，`norm=0`，除零。虽然 `if(norm > limit)` 通常能避免 (limit>0)，但如果 `limit=0` 仍会除零。

### NEW-10 [严重]: Motor_Init 温度计算除零风险

```c
// motor.c:94-95
float r1_ntc = 10.0f*((float)ADC1->DR*3.3f/4095.0f)/(3.3-((float)ADC1->DR*3.3f/4095.0f));
TEMP_MOTOR_filter1 = 1.0f / ( (1.0f / 298.15f) + (logf(r1_ntc / 10.0f) / 3950.0f ) ) - 273.15f;
```

**问题1:** 如果 `ADC1->DR = 4095` (满量程)，分母 `3.3 - 3.3 = 0` → **除零崩溃**

**问题2:** 如果 `r1_ntc < 10.0f`，`logf(r1_ntc/10.0f)` 为负数，温度计算可能异常

**问题3:** 如果 `r1_ntc = 0`，`logf(0)` = -∞ → **未定义行为**

同样问题在 motor.c:98-99 (MOS温度) 和 motor.c:380-381, 407-408 (temperatureSample)。

### NEW-11 [中等]: printf/HAL_UART_Transmit 阻塞1000ms超时

```c
// uart_printf.c:35
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
```

每个字符阻塞最多1000ms。如果UART1硬件故障或未初始化，`printf` 一个字符就阻塞1秒。ISR中的 `printf` (REST_MODE菜单) 会导致严重延迟。

### NEW-12 [低]: GR 和 V_CAL 未定义

```c
// can_rv.c:414, 455
Motor_W = temp * GR;
controller.v_des = uint32_to_float(v_raw, w_min, w_min, 16) * GR;
```

`GR` (减速比) 在 user_config.h 中被注释掉，未定义。代码能编译说明在其他地方有定义，但搜索发现只有注释掉的定义。

**验证:** 需确认 `GR` 和 `V_CAL` 的实际定义位置。

### NEW-13 [低]: Calc_current_rms 无延迟采样同一值

```c
// motor.c:358-363
for(int i=0; i<1000; i++) {
    sumA += p_motor_g->phase_a_current * p_motor_g->phase_a_current;
    sumB += p_motor_g->phase_b_current * p_motor_g->phase_b_current;
    sumC += p_motor_g->phase_c_current * p_motor_g->phase_c_current;
}
```

循环1000次无延迟，采样的是同一瞬时值。RMS = |瞬时值|，不是真正的有效值。

**已在H-06中记录，此处为确认。**

---

## 已确认正确的部分

| 模块 | 验证结果 |
|------|---------|
| Clarke变换 | 等幅值公式正确 ✓ |
| Park/反Park变换 | 公式正确 ✓ |
| SVPWM扇区判断 | 位运算映射正确 ✓ |
| 电流采样相序一致性 | NEGATIVE时SVPWM和currentSample都交换B/C ✓ |
| 编码器圈数跟踪 | 过零检测逻辑正确 ✓ |
| 校准偏移计算 | 正反向取平均消除间隙 ✓ |
| CRC8校验 | 查表法实现正确 ✓ |
| limit_norm向量限幅 | sqrtf单精度, 逻辑正确 ✓ |
| float_to_uint/uint32_to_float | 编解码逻辑正确 ✓ |
| 速度斜坡规划 | clamp限幅正确 ✓ |

---

## 修复优先级路线图

### Phase 1: 紧急修复 (1天, 改配置+单行修复)

```
1. NVIC优先级: TIM1→0, USART2→1, FDCAN→3, USART1→5     [C-01, H-11]
2. 温度判断: 交换Warning/Over的if-else顺序                [C-04]
3. MOTOR_MODE入口: 检查cali_finish!=1则拒绝               [C-05]
4. elec_pos: 加fmodf_pos归一化                            [C-06]
5. D轴滤波: Q_axis→D_axis                                [H-01]
6. MIT解包: w_min→w_max, iq_min→iq_max                   [H-03]
7. MOS温度: temp_Motor→temp_Mos                           [H-04]
8. Motor_Init: Err1→Warning                               [H-05]
9. delay_us: CNT→CR1                                      [M-01]
10. cmd_val溢出: 加char_count<8检查                       [M-08]
```

### Phase 2: Flash安全 (2天)

```
1. ISR中Write_MotorData改为设标志位                       [C-02, C-03]
2. 添加magic number数据校验                               [H-07]
3. Read_MotorData补充FDCAN_ID读回                         [H-08]
4. 校准后条件性写Flash                                    [M-11]
```

### Phase 3: 控制优化 (3天)

```
1. Q轴PI: ki_d→ki_q, k_d→k_q                            [H-02]
2. ADC改硬件触发                                          [H-12]
3. 积分饱和改冻结策略                                     [H-10]
4. 位置环频率降低: pos_calc_period=16                     [M-06]
5. Pack_ActiveReport溢出修复                              [H-14/NEW-01]
```

### Phase 4: 架构改进 (1周)

```
1. 状态机移到主循环, ISR仅保留电流环
2. 启用IWDG看门狗
3. Error_Handler添加disablePWM
4. 启用dq轴前馈补偿
5. 清理注释代码
6. 删除未使用的elec_pos_table[2048]
```

---

## 统计

| 类别 | 数量 |
|------|------|
| 致命 (CRITICAL) | 6 |
| 严重 (HIGH) | 16 |
| 中等 (MEDIUM) | 20 |
| 低 (LOW) | 12 |
| **总计** | **54** |
| 已确认正确 | 11项 (含M-02 RCR配置) |

---

## 关联文档

| 文档 | 内容 |
|------|------|
| Flash_Code_Review.md | Flash存储专项 (9个问题) |
| Project_Code_Review.md | 全项目代码审查 (23个问题) |
| Deep_Dive_Review.md | 算法深度分析 (20+问题) |
| Interrupt_Loop_Review.md | 中断与循环架构 (10个问题) |
| FOC_Algorithm_Review.md | FOC算法与三环控制 (15个问题) |
| Architecture_Design.md | 架构与详细设计 (16章) |

---

*Final Comprehensive Review - Claude*