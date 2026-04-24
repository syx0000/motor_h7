# FIVE - FOC Servo Driver Architecture & Design Document

**MCU:** STM32H743VITx (Cortex-M7, 480MHz, 2MB Flash, 1MB RAM)
**Application:** 8极对PMSM FOC伺服驱动, 50:1减速器, 双编码器
**Build:** Keil MDK-ARM v6 (ARMCLANG), CMSIS-DSP

---

## 1. 硬件架构

### 1.1 功率级

```
DC Bus (48V) ─┬─ 三相全桥逆变器 (6 MOSFET) ─── PMSM (8极对)
              │       ↑ TIM1 CH1/2/3 + CH1N/2N/3N (PE8-PE13)
              │       ↑ Dead-time = 12 counts ≈ 50ns
              │       ↑ BKIN (PE15) 硬件过流保护
              │
              ├─ 母线电压采样 (PA6 → ADC1_INP3, 21:1分压)
              └─ EN_GATE 预驱使能
```

### 1.2 电流采样

```
三相电流 → 采样电阻(2.5mΩ) → 运放(×10) → ADC1注入通道
  Phase A: PA7  → ADC1_INP7 → JDR2
  Phase B: PC4  → ADC1_INP4 → JDR1
  Phase C: PC5  → ADC1_INP8 → JDR3
  Vbus:    PA6  → ADC1_INP3 → JDR4

ADC1: 16-bit, 软件触发注入, 1.5 cycle采样, 16x过采样(常规通道)
ADC2: 16-bit, 温度采样
  Motor NTC: PB1 → ADC2_INP5 → JDR2
  MOS NTC:   PB0 → ADC2_INP9 → JDR1
```

### 1.3 编码器接口

```
双编码器通过RS485 (USART2, 2.5Mbps) 读取:
  USART2 TX/RX: PA2/PA3, DMA收发
  RS485方向控制: PD7 (RS485DIR)

数据格式 (7 bytes + 1 CRC):
  [0:2] angleInner  (24-bit, 输出端编码器)
  [3:5] angleOutter (24-bit, 电机端编码器)
  [6]   reserved
  [7]   CRC8

CPR = 16,777,216 (24-bit), 分辨率 = 0.0000214°
```

### 1.4 通信接口

```
FDCAN1 (PA11/PA12):
  Nominal: 5Mbps prescaler (1Mbps实际)
  Data:    20Mbps (BRS开启)
  FIFO: RX×10, TX×10, 32字节元素

USART1 (PB14/PB15): 调试串口, 中断接收
USART6 (PC6/PC7):   VOFA+调试, DMA发送
```

---

## 2. 软件架构

### 2.1 目录结构

```
90_product_260408/
├── Core/                          # CubeMX生成 + 用户代码
│   ├── Inc/                       # 外设头文件
│   │   ├── main.h                 # 全局宏/extern声明
│   │   ├── adc.h, tim.h, fdcan.h  # 外设配置
│   │   └── stm32h7xx_it.h        # 中断声明
│   └── Src/
│       ├── main.c                 # 初始化 + 主循环
│       ├── stm32h7xx_it.c         # 中断处理 (FOC状态机核心)
│       ├── adc.c, tim.c, fdcan.c  # 外设初始化
│       ├── usart.c, dma.c         # 通信外设
│       └── stm32h7xx_hal_msp.c   # MSP底层初始化
│
├── FOC/                           # FOC算法核心
│   ├── FOC.c/h                    # 坐标变换 + SVPWM + 控制环 + 数学工具
│   ├── motor.c/h                  # 电机模型 + 电流/电压/温度采样
│   ├── encoder.c/h                # 编码器处理 + 速度计算
│   ├── encoderRaw.c/h             # 编码器原始读取(SPI, 已弃用)
│   ├── calibration.c/h            # 相序检测 + 电角度偏移校准
│   ├── pid.c/h                    # 通用PID控制器
│   └── SweepSine.c/h             # 扫频信号发生器(辨识用)
│
├── UserSrc/                       # 应用层
│   ├── Inc/
│   │   ├── can_rv.h               # CAN协议定义
│   │   ├── flash.h                # Flash存储定义
│   │   ├── Diag.h                 # 故障诊断
│   │   ├── interact.h             # 串口交互菜单
│   │   ├── user_config.h          # PI参数/限幅配置
│   │   ├── uart_printf.h          # printf重定向
│   │   └── uart4_VOFA+.h         # VOFA+调试协议
│   └── Src/
│       ├── can_rv.c               # CAN消息解析/打包/协议
│       ├── flash.c                # Flash读写(NVM参数存储)
│       ├── Diag.c                 # 过流/过压/过温诊断
│       ├── interact.c             # 串口菜单界面
│       ├── user_config.c          # PI参数默认值
│       ├── uart_printf.c          # printf → USART1
│       └── uart4_VOFA+.c         # VOFA+数据打包
│
├── Drivers/                       # ST HAL库 (不修改)
└── MDK-ARM/                       # Keil工程文件
    └── FIVE/FIVE.sct             # Linker scatter文件
```

### 2.2 模块依赖关系

```
                    ┌──────────┐
                    │  main.c  │ 初始化 + 主循环
                    └────┬─────┘
                         │
          ┌──────────────┼──────────────┐
          ↓              ↓              ↓
   ┌─────────────┐ ┌──────────┐ ┌────────────┐
   │stm32h7xx_it │ │ flash.c  │ │ interact.c │
   │ (ISR核心)   │ │ (NVM)    │ │ (串口菜单) │
   └──────┬──────┘ └──────────┘ └────────────┘
          │
    ┌─────┼─────┬──────────┬───────────┐
    ↓     ↓     ↓          ↓           ↓
┌──────┐┌────┐┌──────┐┌────────┐┌──────────┐
│FOC.c ││pid ││motor ││encoder ││calibrate │
│坐标  ││控制││采样  ││位置    ││校准      │
│变换  ││器  ││电流  ││速度    ││          │
│SVPWM ││    ││电压  ││        ││          │
└──────┘└────┘└──────┘└────────┘└──────────┘
    ↓                     ↑
┌──────────┐        ┌──────────┐
│ can_rv.c │        │ Diag.c   │
│ CAN协议  │        │ 故障诊断 │
└──────────┘        └──────────┘
```

---

## 3. 时钟与定时器

### 3.1 时钟树

```
HSE 25MHz → PLL1 (M=5, N=192, P=2)
  → SYSCLK = 480MHz ✓
  → AHB ÷2 = 240MHz
  → APB1 ÷2 = 120MHz → TIM6 clk ×2 = 240MHz
  → APB2 ÷2 = 120MHz → TIM1 clk ×2 = 240MHz

PLL2 (M=5, N=100, P=5, Q=5):
  → ADC clk = PLL2P = 100MHz
  → FDCAN clk = PLL2Q = 100MHz
```

### 3.2 定时器分配

| 定时器 | 模式 | 参数 | 用途 |
|--------|------|------|------|
| TIM1 | 中心对齐1, PWM2 | PSC=0, ARR=11999, RCR=1 | 三相PWM + FOC中断(10kHz) |
| TIM2 | 中心对齐1 | PSC=0, ARR=11999 | 已初始化未使用 |
| TIM6 | 向上计数 | PSC=119, ARR=65535 | 微秒延时 |

**TIM1 PWM频率:**
```
f_pwm = TIM1_clk / (2 × 12000) = 240MHz / 24000 = 10kHz
中心对齐模式下RCR=1: 每个PWM周期有上溢+下溢两次计数事件, RCR+1=2次事件产生一次更新中断
ISR频率 = 10kHz (每个PWM周期中断一次, 与PWM_FREQUENCY_DEFAULT=10000一致)
```

---

## 4. 中断架构

### 4.1 优先级配置 (NVIC_PRIORITYGROUP_4)

| 优先级 | 中断 | 频率 | 功能 |
|--------|------|------|------|
| **0** | USART2, DMA1_S0/S1 | 事件 | RS485编码器通信 |
| **1** | TIM1_UP | 10kHz | FOC电流环+状态机 |
| **1** | FDCAN1_IT0 | 事件 | CAN消息接收处理 |
| **1** | USART1 | 事件 | 调试串口命令 |
| **2** | USART6, DMA1_S3 | 事件 | VOFA+调试 |
| **15** | SysTick | 1kHz | HAL时基 |

**已知问题:** USART2优先级(0)高于TIM1(1), 编码器通信可抢占FOC电流环。

### 4.2 TIM1 ISR 执行流程

```
TIM1_UP_IRQHandler() [10kHz]
│
├── startJADC()              // 触发ADC注入转换
├── HAL_UART_Transmit_DMA()  // RS485编码器查询
├── errorDiag()              // 过流/过压/过温检测
├── encoderSample()          // 编码器位置+速度更新
├── voltageSample()          // 母线电压
├── currentSample()          // 三相电流
│
└── switch(FSMstate)         // 状态机
    ├── REST_MODE:           无控制输出
    ├── CALIBRATION_MODE:    设置caliOn_flag
    ├── MOTOR_MODE:
    │   ├── [每4次] PositionLoop()
    │   ├── [每4次] VelocityLoop()
    │   └── [每次]  CurrentLoop() → SVPWM
    ├── HOMING_MODE:         位置环→速度环→电流环(目标=零位)
    └── FOC_POSITION_LOOP_PP: 轨迹插补→位置环→速度环→电流环
```

---

## 5. FOC控制详细设计

### 5.1 坐标变换

```
三相电流 ──Clarke──→ αβ ──Park──→ dq
  Ia,Ib,Ic          Iα,Iβ         Id,Iq

Clarke (等幅值):
  Iα = (2/3)Ia - (1/3)(Ib + Ic)
  Iβ = (√3/3)(Ib - Ic)

Park:
  Id =  cos(θe)·Iα + sin(θe)·Iβ
  Iq = -sin(θe)·Iα + cos(θe)·Iβ

反Park:
  Vα = cos(θe)·Vd - sin(θe)·Vq
  Vβ = sin(θe)·Vd + cos(θe)·Vq

θe = (mech_pos / CPR) × pole_pairs × 2π - elec_offset
```

### 5.2 电流环 (10kHz)

```
         i_d_ref ──(+)──→ [Kp_d + Ki_d/s] ──→ v_d ──┐
                    (-)                                │
                     ↑                                 ├→ limit_norm(1.15×Vbus)
                   I_d                                 │      │
                                                       │      ↓
         i_q_ref ──(+)──→ [Kp_q + Ki_q/s] ──→ v_q ──┘  反Park → SVPWM
                    (-)
                     ↑
                   I_q

参数:
  Kp_d = Kp_q = Current_P = 0.10768 (手调)
  Ki_d = Ki_q = Current_I = 0.0005  (手调)
  积分限幅: ±1.15×Vbus (标量独立)
  输出限幅: 1.15×Vbus (向量)

前馈 (已注释):
  v_d_ff = -ωe × L × iq_ref
  v_q_ff =  ωe × L × id_ref + ωe × λf
  λf = 0.005619 V·s/rad
```

### 5.3 速度环 (2.5kHz)

```
Motor_W ──[斜坡]──→ target ──(+)──→ [Kp_v + Ki_v/s] ──→ i_q_ref
                               (-)
                                ↑
                          encoder1.mech_vel (电机端)

参数:
  Kp_v = Velocity_P = 1.0
  Ki_v = Velocity_I = 0.0001
  output_limit = 45.0 (A)
  斜坡加速度 = FOC_velAccDec = 200 rad/s²

速度计算 (M法):
  mech_vel = Δcount × 2π × f_ISR / CPR / vel_calc_period
  滑窗滤波: N=2 (电机端), N=10 (输出端)
  一阶低通: k=0.1
```

### 5.4 位置环 (2.5kHz)

```
Motor_P ──→ target ──(+)──→ [Kp_p + Ki_p/s] ──→ ω_ref (速度环target)
                      (-)
                       ↑
                 encoder2.pos_abs (输出端)

参数:
  Kp_p = Position_P = 300.0
  Ki_p = Position_I = 0.0000001 (≈0)
  output_limit = 150.0
  deadband = 0.0004 rad (仅FOC_POSITION_LOOP)
```

### 5.5 MIT PD控制 (torque_control)

```
τ = Kp×(p_des - θ) + Kd×(v_des - ω) + t_ff
i_q_ref = τ / KT_OUT

KT_OUT = 1.48 N·m/A (输出端力矩常数)
Kp, Kd, p_des, v_des, t_ff 由CAN下发
```

### 5.6 SVPWM

```
输入: Vα, Vβ (反Park输出)
输出: Ta, Tb, Tc (TIM1 CH1/2/3 比较值)

算法: 七段式对称SVPWM
  1. 扇区判断 (位运算, 6扇区)
  2. 基本矢量时间 Tx, Ty
  3. 过调制保护: if(Tx+Ty > T_pwm-T_sample) 线性缩放
  4. 占空比: Ta=(T-Tx-Ty)/4, Tb=Ta+Tx/2, Tc=Tb+Ty/2
  5. 相序处理: NEGATIVE时交换CH2/CH3

PWM模式: PWM2 (高有效), 中心对齐
死区: 12 counts ≈ 50ns
```

---

## 6. 编码器与位置管理

### 6.1 双编码器架构

```
encoder1 (angleOutter) - 电机端:
  ├─ 用途: FOC电角度计算 + 速度反馈
  ├─ CPR: 16,777,216 (24-bit)
  ├─ elec_offset: 1.719367 (校准后从Flash加载)
  └─ 计算:
      elec_pos = (mech_pos/CPR) × 8 × 2π - elec_offset
      mech_vel = Δcount × 2π × f_ISR / CPR / 4

encoder2 (angleInner) - 输出端:
  ├─ 用途: 位置环反馈
  ├─ CPR: 16,777,216
  ├─ mech_offset: 336.764 (零位, Flash存储)
  └─ 计算:
      pos_abs = 2π × (rotations + mech_pos/CPR)
      mech_vel = Δcount × 2π × f_ISR / CPR / 4
```

### 6.2 圈数跟踪

```c
// encoder.c:102-111
if(delta_mech_pos > CPR/2) {
    rotations -= 1;
    delta_mech_pos -= CPR;
} else if(delta_mech_pos < -CPR/2) {
    rotations += 1;
    delta_mech_pos += CPR;
}
pos_abs = 2π × (rotations + mech_pos/CPR);
```

**限制:** 上电后 `rotations=0`, 多圈绝对位置丢失。

### 6.3 校准流程

```
order_phases():  // 相序检测
  1. 施加D轴电压 θ=0, 等待1s
  2. 旋转电压矢量 0→4π (2圈电角度)
  3. 记录 theta_start, theta_end
  4. direction = (theta_end > theta_start)
  5. phase_order = direction

calibrate():  // 电角度偏移
  1. order_phases()
  2. 正向旋转 θ=0→2π×Np, 记录 error_f[2688]
  3. 反向旋转 θ=2π×Np→0, 记录 error_b[2688]
  4. offset = average(error_f + error_b)
  5. elec_offset = offset
  6. cali_finish = 1
```

---

## 7. 通信协议

### 7.1 CAN协议 (FDCAN)

**消息ID:**
```
0x000 + FDCAN_ID (1-32)  // 下行指令
0x100 + FDCAN_ID         // 上行回复
0x700 + FDCAN_ID         // 主动上报
```

**下行指令 (0x000+ID):**

| DLC | 命令 | 数据格式 | 功能 |
|-----|------|---------|------|
| 8 | MIT PD | [p_des, v_des, Kp, Kd, t_ff] | 位置/速度/力矩混合控制 |
| 8 | 位置模式 | [pos_cmd(4B), vel_ff(2B), iq_ff(2B)] | 位置+前馈 |
| 8 | 速度模式 | [vel_cmd(4B), iq_ff(4B)] | 速度+力矩前馈 |
| 4 | 力矩模式 | [iq_cmd(4B)] | 直接力矩控制 |
| 8 | 参数设置 | [index(2B), value(4B)] | 写参数 |
| 2 | 参数读取 | [index(2B)] | 读参数 |
| 1 | 使能/失能 | [0x01/0x00] | 进入/退出MOTOR_MODE |
| 1 | 清除错误 | [0x01] | 清除Err1/Err2 |

**上行回复 (0x100+ID):**
```
DLC=8:
  [0:1] pos (int16, 0.01rad)
  [2:3] vel (int16, 0.01rad/s)
  [4:5] torque (int16, 0.01N·m)
  [6]   TEMP_MOS (int8, °C)
  [7]   TEMP_MOTOR (int8, °C)
```

**主动上报 (0x700+ID):**
```
DLC=8:
  [0:3] pos_abs (float32, rad)
  [4:7] mech_vel (float32, rad/s)
```

### 7.2 UART调试协议

**USART1 (调试串口):**
```
REST_MODE菜单:
  c - 进入校准模式
  m - 进入电机模式
  e - 编码器显示模式
  s - 参数设置模式
  h - 回零模式
  z - 保存当前位置为零位
  ESC - 返回REST_MODE

MOTOR_MODE命令:
  i<value> - 设置电流 (A)
  v<value> - 设置速度 (rad/s)
  p<value> - 设置位置 (rad)

SETUP_MODE命令:
  c<mode> - 控制模式 (0=MIT, 1=位置, 2=速度, 3=电流)
  p<value> - 位置环Kp
  v<value> - 速度环Kp
  ...
```

**USART6 (VOFA+):**
```
FireWater协议:
  [float32 × N] + [0x00, 0x00, 0x80, 0x7F]

数据通道 (8通道):
  CH1: pos_abs
  CH2: mech_vel
  CH3: Q_axis_current
  CH4: D_axis_current
  CH5: vbus
  CH6: TEMP_MOS
  CH7: TEMP_MOTOR
  CH8: (预留)

发送频率: 1kHz (主循环1ms任务)
```

---

## 8. 数据存储 (Flash NVM)

### 8.1 存储布局

```
地址: 0x081E0000 (Sector 7, 128KB)
参数数量: 140个 (uint32_t)

关键参数:
  [0]   FDCAN_ID
  [1]   motor_calibrated
  [2]   encoder_cali_finish
  [3]   phase_order
  [4]   elec_offset
  [5]   mech_offset
  [6-8] phase_resistance, inductance, flux
  [9-11] Position_P/I/D
  [12-14] Velocity_P/I/D
  [15-17] Current_P/I/D
  ...
```

### 8.2 读写流程

```
Read_MotorData():
  1. 读取140个uint32_t
  2. 转换为float (uint2float)
  3. 覆盖全局变量 (p_motor_g, p_encoder_g, PID参数)

Write_MotorData():
  1. 全局变量 → uint32_t (float2uint)
  2. HAL_FLASH_Unlock()
  3. FLASH_Erase_Sector(7) // 阻塞1-2秒!
  4. FLASH_Program_FlashWord(140次)
  5. HAL_FLASH_Lock()
```

**已知问题:** `Write_MotorData` 在FDCAN中断中被调用, 阻塞1-2秒。

---

## 9. 故障诊断与保护

### 9.1 故障分级

```
一级故障 (Err1, 严重, 立即关断PWM):
  0x0001 - 相电流过流 (硬件/软件)
  0x0002 - 母线过压
  0x0004 - MOS过温
  0x0008 - 电机过温
  0x0010 - 母线过流
  0x0020 - 过载
  0x0040 - 编码器故障
  0x0080 - 其他

二级故障 (Err2, 一般, 不关断):
  0x01 - CAN接收超时
  0x02 - 母线欠压
  0x04 - 超速

三级警告 (Warning):
  0x01 - MOS温度警告
  0x02 - 电机温度警告
```

### 9.2 保护阈值

```c
// Diag.c
#define TEMP_MOSOVER      100.0f  // MOS过温关断
#define TEMP_MOSWARNING    90.0f  // MOS温度警告
#define TEMP_MOTOROVER    100.0f  // 电机过温关断
#define TEMP_MOTORWARNING  90.0f  // 电机温度警告

#define VBUS_OVER          60.0f  // 母线过压
#define VBUS_LOW           20.0f  // 母线欠压

#define CURRENT_OVER       60.0f  // 相电流过流 (软件)
// 硬件过流: BKIN引脚 (PE15)
```

### 9.3 诊断流程

```
errorDiag() [每ISR周期]:
  1. 滑窗滤波 (3点) 电流/电压/温度
  2. 判断阈值
  3. 设置 Err1/Err2/Warning
  4. if(Err1 != 0) disablePWM()

errorReport() [主循环]:
  1. 检查 Err1/Err2/Warning
  2. printf 错误信息
  3. HAL_Delay(200)
```

**已知问题:** 温度判断顺序错误 (Warning阈值先判断), 100°C不会关断PWM。

---

## 10. 状态机设计

### 10.1 状态定义

```c
#define REST_MODE           0  // 菜单/待机
#define CALIBRATION_MODE    1  // 校准
#define MOTOR_MODE          2  // 电机运行
#define SETUP_MODE          4  // 参数设置
#define ENCODER_MODE        5  // 编码器显示
#define HOMING_MODE         6  // 回零
```

### 10.2 状态转换

```
                    ┌─────────────┐
                    │ REST_MODE   │ (上电默认)
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ↓                  ↓                  ↓
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│CALIBRATION   │   │ MOTOR_MODE   │   │ HOMING_MODE  │
│(校准)        │   │(运行)        │   │(回零)        │
└──────────────┘   └──────────────┘   └──────┬───────┘
        │                  │                  │
        │                  │                  │ 到达零位
        └──────────────────┴──────────────────┘
                           │
                           ↓
                    ┌─────────────┐
                    │ REST_MODE   │
                    └─────────────┘

触发条件:
  - UART命令 (c/m/h/ESC)
  - CAN使能/失能
  - 严重故障 (Err1) → 应自动切换到REST (当前未实现)
```

### 10.3 各状态行为

**REST_MODE:**
- PWM关断
- 不执行控制环
- 响应UART菜单命令

**CALIBRATION_MODE:**
- 主循环执行 `calibrate()`
- TIM1 ISR仅采样, 不执行控制环
- 完成后写Flash, 返回REST

**MOTOR_MODE:**
- PWM开启
- 根据 `controlMode` 执行控制环:
  - MIT_PD: torque_control → CurrentLoop
  - FOC_CURRENT_LOOP: CurrentLoop
  - FOC_VELOCITY_LOOP: VelocityLoop → CurrentLoop
  - FOC_POSITION_LOOP: PositionLoop → VelocityLoop → CurrentLoop
  - FOC_POSITION_LOOP_PP: 轨迹插补 → PositionLoop → ...
- CAN超时检测 (CAN_TIMEOUT=500ms)

**HOMING_MODE:**
- 位置环目标 = `mech_offset` (零位)
- 到达零位 (误差<0.05rad) → 自动切换REST

---

## 11. 主循环任务调度

```c
while(1) {
    // 100us任务 (u8_100usFlag, 由TIM1 ISR设置)
    if(u8_100usFlag) {
        Pack_ActiveReport_Current();  // CAN主动上报(电流)
        CAN_SendMessage();
        u8_100usFlag = 0;
    }

    // 1ms任务 (u8_1msFlag, 由TIM1 ISR每10次设置)
    if(u8_1msFlag) {
        Calc_current_rms();           // RMS电流计算
        temperatureSample();          // NTC温度采样
        LoadData();                   // VOFA+数据打包
        HAL_UART_Transmit_DMA();      // VOFA+发送
        Pack_ActiveReport();          // CAN主动上报(位置/速度)
        CAN_SendMessage();
        u8_1msFlag = 0;
    }

    // 校准任务 (caliOn_flag, 由CALIBRATION_MODE设置)
    if(caliOn_flag) {
        enablePWM();
        calibrate();                  // 阻塞5+秒
        MeasureResistance();          // 阻塞1+秒
        MeasureInductance();          // 阻塞1+秒
        disablePWM();
        Write_MotorData();            // 阻塞1-2秒
        caliOn_flag = 0;
    }

    // 错误报告
    errorReport();                    // 含HAL_Delay(200)
}
```

**已知问题:** 校准/测量期间主循环阻塞10+秒, 100us/1ms任务停止。

---

## 12. 关键数据结构

### 12.1 Motor_t (电机参数)

```c
typedef struct _Motor_t {
    // 用户配置
    float IMax;                    // 最大电流 (A)
    uint8_t pole_pairs;            // 极对数
    float phase_resistance;        // 相电阻 (Ω)
    float phase_inductance;        // 相电感 (H)
    float phase_order;             // 相序 (0=反, 1=正)

    // 采样值
    volatile float phase_a/b/c_current;
    volatile float D/Q_axis_current;
    volatile float vbus;

    // 指令
    float i_d_ref, i_q_ref;
    uint8_t controlMode;

    // 错误
    uint16_t Err1;
    uint8_t Err2, Warning;
} Motor_t;
```

### 12.2 Encoder_t (编码器)

```c
typedef struct _Encoder_t {
    uint32_t cpr;                  // 分辨率
    float one_div_cpr;             // 1/CPR

    // 位置
    volatile float elec_pos;       // 电角度 (rad)
    volatile float pos_abs;        // 绝对位置 (rad, 含圈数)
    volatile float mech_abs;       // 机械位置 (rad, 单圈)
    int32_t mech_pos;              // 原始计数
    volatile int32_t rotations;    // 圈数

    // 速度
    volatile float mech_vel;       // 机械速度 (rad/s)
    float elec_vel;                // 电角速度 (rad/s)

    // 校准
    float elec_offset;             // 电角度偏移
    float mech_offset;             // 零位
    uint8_t cali_finish;
} Encoder_t;
```

### 12.3 pid_t (PID控制器)

```c
typedef struct _pid_t {
    // 参数
    float kp, ki, kd;
    float integtal_limit, output_limit;
    float deadband;
    float feedforward_ratio;

    // 状态
    volatile float P, I, D;
    volatile float feedforward;
    float target;
    volatile float feedback;
    volatile float err, err_last;

    // 输出
    float output;
} pid_t;
```

### 12.4 ControllerStruct (电流环控制器)

```c
typedef struct {
    // PI参数
    float k_d, k_q;
    float ki_d, ki_q;

    // 状态
    float d_int, q_int;
    float v_d, v_q;

    // 前馈
    float v_d_ff, v_q_ff;

    // MIT PD
    float kp, kd, t_ff;
    float p_des, v_des;
    float theta_mech, dtheta_mech;
} ControllerStruct;
```

---

## 13. 已知问题汇总 (来自Review文档)

### 13.1 致命/严重 (需立即修复)

| 编号 | 问题 | 影响 | 修复 |
|------|------|------|------|
| I-01 | USART2优先级(0)>TIM1(1) | RS485抢占FOC | 改NVIC优先级 |
| F-01 | elec_pos未mod 2π | arm_sin精度下降 | 加fmodf_pos |
| F-03 | Q轴PI用D轴增益 | dq独立调节时错误 | ki_q, k_q |
| F-06 | 位置/速度环编码器不同 | 单位不一致 | 确认是否需×GR |
| F-14 | 未校准可运行 | 换相错误 | 入口检查cali_finish |
| B-01 | D轴滤波用Q轴值 | dq耦合 | 改D_axis_current_filt |
| B-02 | Flash在中断中擦写 | FOC停摆1-2秒 | 移到主循环 |

### 13.2 中等 (尽快修复)

| 编号 | 问题 | 影响 |
|------|------|------|
| I-06 | ADC软件触发 | 采样时刻不确定 |
| F-10 | 积分清零策略 | 定位恢复慢 |
| F-11 | 位置/速度环同频 | 无频率分离 |
| B-07 | RMS电流计算错误 | 采样同一值1000次 |
| B-08 | 温度判断顺序错 | 100°C不关断 |

---

## 14. 性能指标

### 14.1 控制性能

```
电流环带宽: 1kHz (设计值)
速度环带宽: 100Hz (设计值)
位置环带宽: ~30Hz (Kp=300, 估算)

电流环响应时间: ~1ms
速度环响应时间: ~10ms
位置环响应时间: ~30ms

最大电流: 60A (软件限制)
最大速度: 150 rad/s (输出端, 位置环限制)
最大加速度: 200 rad/s² (速度斜坡)
```

### 14.2 CPU占用率

```
TIM1 ISR (MOTOR_MODE, 电流环):
  典型: 35us @ 10kHz → 35%
  最坏: 55us @ 10kHz → 55%

主循环:
  100us任务: ~5us
  1ms任务: ~20us
  总计: ~2%

剩余: ~43% 可用
```

### 14.3 通信性能

```
CAN:
  上行回复: 每次指令 (事件触发)
  主动上报: 10kHz (100us任务)
  最大吞吐: ~1Mbps (实际)

UART:
  调试串口: 115200 bps
  VOFA+: 1kHz × 8通道 × 4B = 32KB/s
  编码器RS485: 2.5Mbps, 7B × 10kHz = 70KB/s
```

---

## 15. 开发与调试

### 15.1 编译配置

```
工具链: ARMCLANG 6.x (Keil MDK-ARM)
优化: -O2
FPU: -mfpu=fpv5-d16 -mfloat-abi=hard
定义: USE_HAL_DRIVER, STM32H743xx

链接脚本: FIVE.sct
  IROM1: 0x08000000, 0x00200000 (2MB)
  IRAM1: 0x24000000, 0x00080000 (512KB AXI SRAM)
  IRAM2: 0x20000000, 0x00020000 (128KB DTCM)
```

### 15.2 调试接口

```
SWD: PA13/PA14 (SWDIO/SWCLK)
UART1: PB14/PB15 (调试串口, printf输出)
UART6: PC6/PC7 (VOFA+波形)

DWT计数器: 用于ISR执行时间测量
  ISR_start = DWT_CYCCNT;
  ISR_time_us = (DWT_CYCCNT - ISR_start) / (MCU_SYSCLK/1e6);
```

### 15.3 常用调试命令

```
UART1菜单:
  m - 进入电机模式
  i10 - 设置电流10A
  v5 - 设置速度5rad/s
  p3.14 - 设置位置π
  ESC - 停止

CAN命令 (Python/CANoe):
  使能: ID=0x001, DLC=1, Data=[0x01]
  力矩: ID=0x001, DLC=4, Data=[iq(float32)]
  位置: ID=0x001, DLC=8, Data=[pos(f32), vel_ff(i16), iq_ff(i16)]
```

---

## 16. 未来改进方向

### 16.1 短期 (P0/P1问题修复)

1. 修复中断优先级倒挂
2. 修复FOC算法bug (F-01, F-03, B-01)
3. Flash操作移出中断
4. 添加未校准运行保护
5. 修复温度保护判断顺序

### 16.2 中期 (架构优化)

1. 状态机移到主循环
2. ADC改为硬件触发
3. 启用dq轴前馈补偿
4. 位置环频率降低到625Hz
5. 添加看门狗

### 16.3 长期 (功能增强)

1. 实现真正的过调制算法
2. 添加弱磁控制
3. 实现无感FOC (观测器)
4. 添加振动抑制
5. 实现自适应参数辨识

---

*Architecture & Design Document - Generated by Claude*
