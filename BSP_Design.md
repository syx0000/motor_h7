# BSP 设计文档

## 1. 项目概述

本项目是基于 STM32H7xx (Cortex-M7, 480MHz) 的 FOC 电机控制器 BSP，使用 STM32CubeMX 生成 + 用户自定义代码的混合架构，开发环境为 Keil MDK-ARM。

应用场景：三相无刷电机 FOC 矢量控制，支持电流环/速度环/位置环/MIT PD控制/PP轨迹规划等多种控制模式，通过 CAN FD 和 RS485 实现上位机通信。

---

## 2. 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32H7xx (Cortex-M7) |
| Flash | 2 MB (0x08000000) |
| SRAM | D1 AXI SRAM 128 KB (0x20000000) + D2 AXI SRAM 512 KB (0x24000000) |
| 外部晶振 HSE | 25 MHz |
| 系统主频 | 480 MHz (PLL1) |
| 供电模式 | LDO, Voltage Scale 0 (最高性能) |
| 工具链 | Keil MDK-ARM (ARM Compiler) |

---

## 3. 目录结构

```
90_product_260408/
├── Core/
│   ├── Inc/                          # 头文件
│   │   ├── main.h                    # 主头文件, GPIO引脚宏定义
│   │   ├── stm32h7xx_hal_conf.h      # HAL模块使能配置
│   │   ├── stm32h7xx_it.h            # 中断处理函数声明
│   │   ├── adc.h / dma.h / fdcan.h   # 外设头文件
│   │   ├── gpio.h / tim.h / usart.h
│   │   └── memorymap.h
│   └── Src/                          # 源文件
│       ├── main.c                    # 主程序入口, 系统时钟配置, MPU配置
│       ├── system_stm32h7xx.c        # CMSIS SystemInit, FPU使能
│       ├── stm32h7xx_hal_msp.c       # HAL MSP全局初始化
│       ├── stm32h7xx_it.c            # 中断服务函数 (核心控制逻辑所在)
│       ├── adc.c                     # ADC1/ADC2 注入通道配置
│       ├── tim.c                     # TIM1(PWM)/TIM2/TIM6 配置
│       ├── usart.c                   # USART1/2/3/6 配置及DMA关联
│       ├── fdcan.c                   # FDCAN1 配置及滤波器/中断使能
│       ├── dma.c                     # DMA1控制器时钟及中断优先级
│       ├── gpio.c                    # GPIO初始化
│       └── memorymap.c              # 内存映射 (当前为空)
├── Drivers/
│   ├── STM32H7xx_HAL_Driver/         # ST HAL库 (124个.c文件)
│   └── CMSIS/                        # ARM CMSIS核心文件
└── MDK-ARM/
    └── FIVE/FIVE.sct                 # 链接脚本 (Scatter file)
```

---

## 4. 时钟树设计

### 4.1 时钟源与PLL配置

```
HSE (25 MHz)
  │
  ├─► PLL1 (主系统时钟)
  │     PLLM=5  → VCO_IN = 5 MHz
  │     PLLN=192 → VCO_OUT = 960 MHz
  │     PLLP=2  → SYSCLK = 480 MHz
  │     PLLQ=3  → PLL1Q = 320 MHz
  │     PLLR=2  → PLL1R = 480 MHz
  │
  └─► PLL2 (外设时钟: ADC + FDCAN)
        PLL2M=5  → VCO_IN = 5 MHz
        PLL2N=100 → VCO_OUT = 500 MHz
        PLL2P=5  → ADC_CLK = 100 MHz
        PLL2Q=5  → FDCAN_CLK = 100 MHz
```

### 4.2 总线时钟分频

| 时钟域 | 分频 | 频率 |
|--------|------|------|
| SYSCLK | /1 | 480 MHz |
| AHB (HCLK) | /2 | 240 MHz |
| APB1 (D2) | /2 | 120 MHz |
| APB2 (D2) | /2 | 120 MHz |
| APB3 (D1) | /2 | 120 MHz |
| APB4 (D3) | /2 | 120 MHz |
| ADC_CLK | PLL2P | 100 MHz |
| FDCAN_CLK | PLL2Q | 100 MHz |

### 4.3 UART时钟源

| UART | 时钟源 | 频率 |
|------|--------|------|
| USART1 | D2PCLK2 (APB2) | 120 MHz |
| USART2 | D2PCLK1 (APB1) | 120 MHz |
| USART3 | D2PCLK1 (APB1) | 120 MHz |
| USART6 | D2PCLK2 (APB2) | 120 MHz |

---

## 5. 内存布局

### 5.1 Scatter File (FIVE.sct)

```
LR_IROM1 0x08000000 0x00200000       ; Flash加载区 (2MB)
  ER_IROM1 0x08000000 0x00200000      ; 代码执行区
    *.o (RESET, +First)               ; 向量表置顶
    *(InRoot$$Sections)
    .ANY (+RO +XO)

  RW_IRAM1 0x20000000 0x00020000      ; D1 AXI SRAM (128KB) - 主RW/ZI数据
    .ANY (+RW +ZI)

  RW_IRAM2 0x24000000 0x00080000      ; D2 AXI SRAM (512KB) - 溢出RW/ZI数据
    .ANY (+RW +ZI)
```

### 5.2 MPU配置

- Region 0: 全4GB空间, 禁止访问 (SubRegionDisable=0x87, 保护未映射区域)
- 默认背景区域: 特权模式可访问 (MPU_PRIVILEGED_DEFAULT)

---

## 6. 外设配置详细设计

### 6.1 TIM1 — 三相PWM (电机控制核心)

| 参数 | 值 | 说明 |
|------|-----|------|
| 计数模式 | 中心对齐模式1 | 对称PWM, 减少谐波 |
| 预分频 | 0 | 定时器时钟 = APB2×2 = 240 MHz |
| 自动重装值 | 11999 | 中心对齐: f_PWM = 240M/(2×12000) = 10 kHz |
| 重复计数器 | 1 | 中心对齐模式下每PWM周期有2次计数事件(上溢+下溢), RCR=1→每2次事件中断一次→ISR=10kHz ✓ |
| PWM模式 | PWM2 | 向下计数时有效 |
| 死区时间 | 12 (约50ns @240MHz) | 防止上下桥臂直通 |
| 互补输出 | 3通道 (CH1/CH1N, CH2/CH2N, CH3/CH3N) | 三相全桥驱动 |
| Break输入 | PE15 (BKIN), 高电平有效 | 硬件过流保护 |
| 主输出 | BDTR[MOE]=1, AOE=1 | 自动输出使能 |

PWM引脚映射:

| 通道 | 高侧 | 低侧 |
|------|-------|-------|
| A相 | PE9 (CH1) | PE8 (CH1N) |
| B相 | PE11 (CH2) | PE10 (CH2N) |
| C相 | PE13 (CH3) | PE12 (CH3N) |

### 6.2 TIM2 — 辅助定时器

| 参数 | 值 |
|------|-----|
| 计数模式 | 中心对齐模式1 |
| 预分频 | 0 |
| 自动重装值 | 11999 |
| 用途 | 与TIM1同步, 辅助计时 |

### 6.3 TIM6 — 基本定时器

| 参数 | 值 |
|------|-----|
| 预分频 | 119 → 计数频率 = 120MHz/120 = 1 MHz |
| 自动重装值 | 65535 |
| 用途 | 微秒级时间基准 (DWT辅助) |

### 6.4 ADC1 — 电流/电压采样

| 参数 | 值 |
|------|-----|
| 分辨率 | 16-bit |
| 时钟 | PLL2P/4 = 25 MHz |
| 转换模式 | 注入通道, 软件触发 |
| 过采样 | 16倍 (常规通道) |
| 采样时间 | 1.5 cycles (注入通道, 快速采样) |

注入通道分配:

| Rank | 通道 | 引脚 | 信号 |
|------|------|------|------|
| 1 | CH4 | PC4 | CUR_B (B相电流) |
| 2 | CH7 | PA7 | CUR_A (A相电流) |
| 3 | CH8 | PC5 | CUR_C (C相电流) |
| 4 | CH3 | PA6 | VDC (母线电压) |

### 6.5 ADC2 — 温度采样

| 参数 | 值 |
|------|-----|
| 分辨率 | 16-bit |
| 采样时间 | 16.5 cycles (温度信号变化慢, 可用较长采样) |

注入通道分配:

| Rank | 通道 | 引脚 | 信号 |
|------|------|------|------|
| 1 | CH5 | PB1 | TEMP_MOS (MOS管温度) |
| 2 | CH9 | PB0 | TEMP_MOTOR (电机温度) |

### 6.6 USART1 — 串口调试/命令行

| 参数 | 值 |
|------|-----|
| 波特率 | 921600 |
| 数据位 | 8-bit, 无校验, 1停止位 |
| 传输方式 | 中断接收 (逐字节) |
| 引脚 | PB14(TX) / PB15(RX) |
| 功能 | 串口命令行交互 (状态机菜单: 校准/电机/编码器/设置/回零) |

### 6.7 USART2 — RS485 编码器通信

| 参数 | 值 |
|------|-----|
| 波特率 | 2,500,000 (2.5 Mbps) |
| 传输方式 | DMA收发 + 空闲中断检测 |
| 引脚 | PA2(TX) / PA3(RX) |
| 方向控制 | PA1 (RS485_DIR), 软件切换收发 |
| DMA通道 | RX: DMA1_Stream0 (最高优先级), TX: DMA1_Stream1 (最高优先级) |
| 功能 | 高速编码器数据读取 (每个PWM周期发送查询并接收角度数据) |

数据协议: 发送1字节查询 → 接收8字节响应 (内端角度3B + 外端角度3B + 保留1B + CRC 1B)

### 6.8 USART3 — 备用串口

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| 引脚 | PB10(TX) / PB11(RX) |
| 传输方式 | 轮询 |

### 6.9 USART6 — VOFA+ 数据可视化

| 参数 | 值 |
|------|-----|
| 波特率 | 921600 |
| 传输方式 | DMA发送 |
| 引脚 | PC6(TX) / PC7(RX) |
| DMA通道 | TX: DMA1_Stream3 (低优先级) |
| 功能 | 实时波形数据上传 (VOFA+ JustFloat协议) |

### 6.10 FDCAN1 — CAN FD 通信

| 参数 | 值 |
|------|-----|
| 帧格式 | FD + BRS (比特率切换) |
| 仲裁段波特率 | 100M / (5×(16+3+1)) = 1 Mbps |
| 数据段波特率 | 100M / (1×(16+3+1)) = 5 Mbps |
| 引脚 | PA11(RX) / PA12(TX) |
| RX FIFO0 | 10个元素, 32字节数据帧 |
| TX FIFO | 10个元素, 32字节数据帧 |
| 滤波器 | 全通 (ID=0x0000, Mask=0x0000) |
| 功能 | 电机控制指令接收, 状态主动上报 |

### 6.11 GPIO 功能分配

| 引脚 | 方向 | 功能 |
|------|------|------|
| PA1 | Output | RS485_DIR (收发方向控制) |
| PD4 | Output | EN_GATE (栅极驱动使能, 上电置高) |
| PB4 | Output | ERR_RUN (故障指示灯) |
| PB5 | Output | LED_RUN (运行指示灯, ISR执行期间置高) |
| PH0/PH1 | AF | HSE 晶振 |
| PA13/PA14 | AF | SWD 调试接口 |

### 6.12 DMA 通道分配

| DMA通道 | 外设 | 方向 | 优先级 | 中断优先级 |
|---------|------|------|--------|-----------|
| DMA1_Stream0 | USART2_RX | 外设→内存 | Very High | 0 |
| DMA1_Stream1 | USART2_TX | 内存→外设 | Very High | 0 |
| DMA1_Stream3 | USART6_TX | 内存→外设 | Low | 2 |

---

## 7. 中断体系设计

### 7.1 中断优先级分配

NVIC 使用 4-bit 优先级, 默认分组: 4-bit 抢占 + 0-bit 子优先级。

| 优先级 | 中断源 | 功能 |
|--------|--------|------|
| 0 (最高) | DMA1_Stream0 (USART2_RX) | RS485编码器数据接收 |
| 0 | DMA1_Stream1 (USART2_TX) | RS485编码器查询发送 |
| 0 | USART2 | RS485空闲/发送完成中断 |
| 1 | TIM1_UP | **核心控制中断** (10kHz, RCR=1中心对齐模式) |
| 1 | FDCAN1_IT0 | CAN FD 接收中断 |
| 1 | USART1 | 串口命令行接收 |
| 2 | DMA1_Stream3 (USART6_TX) | VOFA数据发送 |
| 2 | USART6 | VOFA串口中断 |
| 15 | SysTick | HAL时基 (1ms) |

### 7.2 TIM1 更新中断 — 核心控制ISR (10 kHz)

这是整个系统的核心, 在 `stm32h7xx_it.c:TIM1_UP_IRQHandler` 中实现:

```
TIM1_UP_IRQHandler (每100us执行一次)
│
├── LED_RUN 置高 (ISR执行指示)
├── DWT计时开始
├── 设置 100us / 1ms 时间标志
├── RS485方向切换为发送
├── startJADC() — 启动ADC注入转换
├── DMA发送编码器查询 (USART2)
├── errorDiag() — 故障诊断
├── encoderSample() — 编码器采样
├── voltageSample() — 母线电压采样
├── currentSample() — 三相电流采样
│
├── 状态机 (FSMstate) 分支:
│   ├── REST_MODE — 空闲, 等待命令
│   ├── CALIBRATION_MODE — 编码器/电机参数校准
│   ├── MOTOR_MODE — 电机运行
│   │   ├── MIT_PD — MIT PD力矩控制
│   │   ├── FOC_CURRENT_LOOP — 电流环 (含扫频激励)
│   │   ├── FOC_VELOCITY_LOOP — 速度环 + 电流环
│   │   ├── FOC_POSITION_LOOP — 位置环 + 速度环 + 电流环
│   │   └── FOC_POSITION_LOOP_PP — PP轨迹规划 + 位置环
│   ├── SETUP_MODE — 参数设置
│   ├── ENCODER_MODE — 编码器调试
│   └── HOMING_MODE — 回零模式
│
├── 清除TIM1状态寄存器
├── DWT计时结束, 计算ISR耗时
└── LED_RUN 置低
```

### 7.3 USART1 接收中断 — 命令行解析

实现了完整的串口命令行交互状态机:

- REST_MODE下: `c`校准 / `m`电机 / `e`编码器 / `s`设置 / `h`回零 / `z`存零位 / `ESC`返回
- MOTOR_MODE下: `i<val>`设电流 / `v<val>`设速度 / `p<val>`设位置
- SETUP_MODE下: `c<val>`控制模式 / `b<val>`电流环带宽 / `P/I`速度PI / `M/N`位置PI / `Q/W`电流PI 等
- CALIBRATION_MODE下: `e`编码器校准 / `m`电机参数辨识 / `a`全部校准

### 7.4 USART2 中断 — RS485 编码器通信

- 空闲中断: 检测到一帧数据接收完成, 停止DMA, 拷贝数据到应用缓冲区, 解析内外端角度, CRC校验
- 发送完成中断: 自动切换RS485方向为接收

### 7.5 FDCAN1 接收回调

- FIFO0新消息中断 → `HAL_FDCAN_RxFifo0Callback` → `CAN_MsgProcess()` 处理CAN指令

---

## 8. 启动流程

```
上电/复位
  │
  ├── 1. startup_stm32h7xx.s
  │     ├── 初始化栈指针 (MSP)
  │     ├── 复制.data段到SRAM
  │     ├── 清零.bss段
  │     └── 调用 SystemInit()
  │
  ├── 2. SystemInit() [system_stm32h7xx.c]
  │     ├── 使能FPU (CP10/CP11 Full Access)
  │     ├── 配置Flash等待周期
  │     ├── 复位RCC到默认状态
  │     └── 禁用FMC Bank1
  │
  ├── 3. main() [main.c]
  │     ├── MPU_Config() — 内存保护配置
  │     ├── HAL_Init() — HAL初始化, SysTick配置
  │     ├── SystemClock_Config() — PLL1配置, 480MHz
  │     ├── PeriphCommonClock_Config() — PLL2配置 (ADC/FDCAN时钟)
  │     │
  │     ├── ── 外设初始化 ──
  │     ├── MX_GPIO_Init()
  │     ├── MX_DMA_Init()
  │     ├── MX_TIM1_Init() / MX_TIM2_Init()
  │     ├── MX_USART1_UART_Init() (开启接收中断)
  │     ├── MX_TIM6_Init()
  │     ├── MX_FDCAN1_Init() (配置滤波器, 启动FDCAN, 使能接收中断)
  │     ├── MX_ADC1_Init() / MX_ADC2_Init()
  │     ├── MX_USART2_UART_Init() (开启DMA空闲接收)
  │     ├── MX_USART6_UART_Init()
  │     ├── MX_USART3_UART_Init()
  │     │
  │     ├── ── 用户初始化 ──
  │     ├── EN_GATE 置高 (使能栅极驱动)
  │     ├── enableADC() — ADC校准并使能
  │     ├── TIM1 BDTR MOE+AOE, 使能更新中断, 启动计数器
  │     ├── TIM6 启动
  │     ├── RS485 首次DMA发送 (触发编码器通信)
  │     ├── Motor_Init() — 电机参数初始化
  │     ├── DWT_Init() — DWT周期计数器初始化
  │     ├── Encoder_Init() — 编码器初始化
  │     ├── Read_MotorData() — 从Flash读取校准参数
  │     ├── init_controller_params() — PI控制器参数初始化
  │     ├── Pid.Init() — 位置环/速度环PID初始化
  │     ├── CalcCurrentOffset() — 电流采样零偏校准
  │     ├── TIM1 开始产生更新事件 (控制环路启动)
  │     ├── enter_menu_state() — 进入命令行菜单
  │     ├── SweepSine_Init() / SweepSine_Start() — 扫频激励初始化
  │     │
  │     └── ── 主循环 ──
  │           ├── 100us任务: CAN电流上报
  │           ├── 1ms任务: RMS电流计算, 温度采样, 到位判断, VOFA数据发送, CAN状态上报
  │           ├── 状态切换处理
  │           ├── 校准流程 (编码器校准/电阻电感辨识/Flash存储)
  │           └── 故障上报
```

---

## 9. 电源管理

| 项目 | 配置 |
|------|------|
| 供电方式 | LDO (PWR_LDO_SUPPLY) |
| 电压调节 | Scale 0 (最高性能, 支持480MHz) |
| VDD | 3.3V |
| 低功耗模式 | 未使用 (实时控制系统, 持续运行) |

---

## 10. 关键设计参数汇总

| 参数 | 值 |
|------|-----|
| PWM频率 | 10 kHz (中心对齐) |
| 控制环频率 | 电流环 10 kHz, 速度环 5 kHz, 位置环 2.5 kHz |
| 死区时间 | ~50 ns |
| ADC分辨率 | 16-bit |
| ADC采样时间 | 1.5 cycles (电流) / 16.5 cycles (温度) |
| 编码器通信速率 | 2.5 Mbps RS485 |
| CAN通信速率 | 仲裁段 1 Mbps / 数据段 5 Mbps |
| 调试串口速率 | 921600 bps |
| ISR最大执行时间 | 通过DWT实测, LED_RUN指示 |
| Flash参数存储 | 内部Flash, 校准数据持久化 |

---

## 11. 外设依赖关系图

```
                    RCC (时钟使能)
                         │
          ┌──────────────┼──────────────┐
          │              │              │
        GPIO           DMA1          SYSCFG
          │              │
    ┌─────┼─────┐    ┌───┼───┐
    │     │     │    │   │   │
  UART  FDCAN  TIM  S0  S1  S3
  1/2/   1     1/   │   │   │
  3/6          2/6  │   │   │
    │               │   │   │
    └───────────────┘   │   │
    USART2 ◄── DMA_RX ──┘   │
    USART2 ◄── DMA_TX ──────┘ (Stream1)
    USART6 ◄── DMA_TX ──────── (Stream3)

  ADC1 ──► 电流采样 (CUR_A/B/C) + 电压采样 (VDC)
  ADC2 ──► 温度采样 (TEMP_MOTOR/TEMP_MOS)
  TIM1 ──► 三相PWM + 控制中断 (10kHz)
  FDCAN1 ──► CAN FD 通信
  USART1 ──► 串口命令行 (中断)
  USART2 ──► RS485 编码器 (DMA)
  USART6 ──► VOFA+ 波形 (DMA)
```

---

## 12. 注意事项与设计约束

1. TIM1更新中断是系统核心, 优先级为1, 必须保证在100us内执行完毕, 通过DWT计时和LED_RUN监控
2. RS485 (USART2) 的DMA中断优先级为0 (高于TIM1), 确保编码器数据不丢失
3. FDCAN滤波器配置为全通模式 (ID和Mask均为0), 接收所有CAN帧, 由软件过滤
4. ADC使用注入通道而非常规通道, 支持在TIM1中断中同步触发转换, 保证采样时刻精确
5. EN_GATE引脚上电默认低电平 (驱动禁止), 初始化完成后才置高, 防止上电瞬间误动作
6. Flash参数存储用于保存编码器校准数据、电机参数、零位等, 掉电不丢失
7. CAN超时机制: 超过CAN_TIMEOUT个控制周期未收到指令, 自动清零所有控制目标 (安全保护)
8. 错误处理: Error_Handler() 关闭全局中断并死循环, 需要复位恢复
