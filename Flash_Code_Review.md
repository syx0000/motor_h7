# Flash Code Review Report

**Project:** STM32H743VIT FOC Motor Driver (FIVE)  
**Date:** 2026-04-17  
**Review Scope:** Flash read/write/erase operations,校准数据持久化，地址冲突检查  

---

## 1. 项目Flash架构概述

| 项目 | 详情 |
|------|------|
| MCU | STM32H743VITx (Cortex-M7, 2MB Flash dual-bank) |
| 数据存储区 | Bank2 Sector7 (`0x081E0000 ~ 0x081FFFFF`, 128KB) |
| 固件大小 | ~115KB (全部位于Bank1) |
| 数据大小 | 128 Bytes (32 x uint32_t) |
| 写入单位 | FLASHWORD = 32字节 (256-bit), 共3次写入 |

### 数据布局 (flash_buffer[0..31])

| Index | 内容 | 写入Flash | 读回Flash |
|-------|------|-----------|-----------|
| [0] | elec_offset (电角度偏移) | Y | Y (cali_finish==1时) |
| [1] | phase_order (相序) | Y | Y (cali_finish==1时) |
| [2] | cali_finish (编码器校准完成标志) | Y | Y |
| [3] | phase_resistance (相电阻) | Y | Y (motor_calibrated==1时) |
| [4] | phase_inductance (相电感) | Y | Y (motor_calibrated==1时) |
| [5] | motor_calibrated (电机校准完成标志) | Y | Y |
| [6] | lastError (上次错误码, 写入时清零) | Y | Y |
| [7] | encoder2 mech_offset (机械角度偏移) | Y | Y |
| [8-9] | p_min, p_max (位置限幅) | Y | Y |
| [10-11] | w_min, w_max (速度限幅) | Y | Y |
| [12-13] | iq_min, iq_max (扭矩限幅) | Y | Y |
| [14-15] | KP_MIN, KP_MAX | Y | Y |
| [16-17] | KD_MIN, KD_MAX | Y | Y |
| **[18]** | **FDCAN_ID** | **Y** | **N (BUG)** |
| [19-23] | 保留 (写0) | Y | - |
| [24-31] | 保留 (写0) | **N (未写入)** | - |

### Write_MotorData 调用位置

| 调用位置 | 执行上下文 | 触发条件 |
|----------|-----------|---------|
| `main.c:384` | 主循环 | 校准流程完成后 |
| `stm32h7xx_it.c:640` | **FDCAN中断 (ISR)** | UART收到'z'命令设置零位 |
| `can_rv.c:511` | **FDCAN中断 (ISR)** | CAN收到0xFC命令设置零位 |
| `can_rv.c:812` | **FDCAN中断 (ISR)** | CAN写参数后bSaveDataFlag==true |

### 调用链 (中断上下文)

```
FDCAN1_IT0_IRQHandler()          // stm32h7xx_it.c:271
  -> HAL_FDCAN_IRQHandler()      // HAL驱动
    -> HAL_FDCAN_RxFifo0Callback() // stm32h7xx_it.c:1030
      -> CAN_MsgProcess()        // can_rv.c:478
        -> Write_MotorData()     // flash.c:42  <-- 在ISR中执行flash擦除+编程!
```

---

## 2. 发现问题汇总

| 编号 | 严重度 | 问题 | 文件:行号 |
|------|--------|------|-----------|
| F-01 | **致命** | 中断上下文执行Flash擦除/编程，阻塞FOC电流环 | can_rv.c:511,812 / stm32h7xx_it.c:640 |
| F-02 | **致命** | Flash操作无中断保护，存在重入风险 | flash.c:42-125 |
| F-03 | **严重** | 掉电无保护：擦除后写入前断电导致数据全丢 | flash.c:92-121 |
| F-04 | **严重** | 无数据有效性校验 (无magic/CRC) | flash.c:127-157 |
| F-05 | **中等** | FDCAN_ID 写入Flash但从未读回 | flash.c:67 vs flash.c:127-157 |
| F-06 | **中等** | 校准后无条件写Flash，未判断是否真正执行了校准 | main.c:384 |
| F-07 | **低** | flash_buffer缺少32字节对齐声明 | flash.c:8 |
| F-08 | **低** | Flash_Init未配对Lock，异常路径未必能Lock | flash.c:10-25 |
| F-09 | **建议** | 无磨损均衡机制 | flash.c 整体 |

---

## 3. 问题详细分析

### F-01 [致命] 中断上下文执行Flash擦除/编程

**问题描述：**

`Write_MotorData()` 通过以下路径在FDCAN中断处理函数中被直接调用：

```c
// stm32h7xx_it.c:1030 - FDCAN中断回调
void HAL_FDCAN_RxFifo0Callback(...) {
    CAN_MsgProcess(RxHeader.Identifier, FDCAN1_RX_BUF); // :1037
}

// can_rv.c:511 - 0xFC命令直接调用
p_encoder2_g->mech_offset = p_encoder2_g->mech_abs;
Write_MotorData();  // 在ISR中擦除128KB扇区!

// can_rv.c:812 - 参数保存
if(bSaveDataFlag == true)
    Write_MotorData();  // 在ISR中擦除128KB扇区!
```

**危害：**

STM32H743 擦除一个128KB扇区耗时约 **1~2秒**。在此期间：
- TIM1更新中断(FOC电流环)被阻塞 -> **电机失控，可能产生危险电流/力矩**
- SysTick中断被阻塞 -> HAL_Delay功能异常
- 其他通信中断被阻塞

**这是校准偏移失效的最可能原因：** CAN消息在校准过程中到达，触发ISR中的`Write_MotorData`，此时校准尚未完成，写入的是不完整的校准数据。

**修复建议：**

```c
// 方案: 中断中设置标志位，主循环中统一处理
volatile bool bFlashWriteRequest = false;

// can_rv.c 中替换所有 Write_MotorData() 调用:
// Write_MotorData();  // 删除
bFlashWriteRequest = true; // 改为设置标志

// main.c 主循环中添加:
if (bFlashWriteRequest) {
    bFlashWriteRequest = false;
    Write_MotorData();
}
```

---

### F-02 [致命] Flash操作无中断保护，存在重入风险

**问题描述：**

`Write_MotorData()` 内部无任何中断屏蔽操作。如果正在主循环中执行 `Write_MotorData()`（校准后），同时CAN中断到达并再次调用 `Write_MotorData()`，将发生：

1. 主循环中：Flash_EraseSector 正在执行
2. ISR中：再次调用 Flash_EraseSector -> HAL_FLASH_Unlock再次执行
3. Flash控制器状态被破坏，可能导致HardFault或数据写入错误

虽然HAL内部有 `__HAL_LOCK` 保护，但这只是软件锁（检查一个变量），**在中断嵌套场景下不安全**。ISR检测到锁就直接返回 `HAL_BUSY`，但 `Write_MotorData` 中没有处理这个返回值对应的重入情况。

**修复建议：**

```c
void Write_MotorData(void)
{
    __disable_irq();  // 关闭全局中断

    // ... 现有的擦除和编程代码 ...

    HAL_FLASH_Lock();
    __enable_irq();   // 恢复全局中断
}
```

> 注意：关中断期间(1~2秒)仍会导致FOC电流环停摆。更好的方案是结合F-01，确保Write_MotorData只在安全时机(电机未运行/已DisablePWM后)调用。

---

### F-03 [严重] 掉电无保护

**问题描述：**

`Write_MotorData()` 的执行流程：

```
Step 1: 擦除整个128KB扇区 (数据变为0xFF)
Step 2: 写入 flash_buffer[0..7]   到 0x081E0000
Step 3: 写入 flash_buffer[8..15]  到 0x081E0020
Step 4: 写入 flash_buffer[16..23] 到 0x081E0040
```

如果在 Step 1 完成后、Step 2~4 之前掉电：
- Flash中全部为 `0xFFFFFFFF`
- 下次上电 `Read_MotorData()` 读取 `cali_finish = 0xFFFFFFFF != 1`
- **电角度偏移不加载**，回退到 `Encoder_Init()` 中的硬编码默认值 `1.719367`
- 如果当前电机的实际偏移不是这个值 -> **FOC换相角度错误 -> 电机运行异常**

如果在 Step 2 完成后、Step 3~4 前掉电：
- `cali_finish = 1`，`elec_offset` 正常
- 但 `flash_buffer[8..17]` 为 `0xFFFFFFFF`
- `p_min/p_max/w_min/w_max/iq_min/iq_max` 全部被解析为 NaN 或极大值
- **限幅参数失效**

**修复建议：**

```c
// 方案1 (简单): 在最后一个flashword中写入magic number
#define FLASH_MAGIC 0xA5A5A5A5

// Write_MotorData 末尾:
flash_buffer[31] = FLASH_MAGIC;
// 第四次写入 flash_buffer[24..31]
HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DATA_FLASH_ADDR+96,
                  (uint32_t)(&flash_buffer[24]));

// Read_MotorData 开头:
if (*(uint32_t*)(DATA_FLASH_ADDR + 4*31) != FLASH_MAGIC) {
    printf("Flash data invalid, using defaults\n");
    return;  // 使用初始化默认值
}
```

```c
// 方案2 (推荐): 双页交替写入
// 使用两个不同的flash地址区域交替写入，每次写入新区域后才擦除旧区域
// 确保任何时刻至少有一份完整的有效数据
```

---

### F-04 [严重] 无数据有效性校验

**问题描述：**

`Read_MotorData()` 直接从Flash地址读取数据，不做任何有效性检查：

```c
void Read_MotorData(void) {
    for(uint16_t i=0; i<32; i++)
        flash_buffer[i] = *(uint32_t*)(DATA_FLASH_ADDR + 4*i);

    p_encoder_g->cali_finish = flash_buffer[2];
    // ... 直接使用，无校验
}
```

可能读到无效数据的场景：
- 全新芯片首次运行（Flash全为0xFF）
- 掉电导致写入不完整
- Flash位翻转（罕见但可能）
- 调试器意外擦除

当 `cali_finish` 恰好读到 `1`（如某次写入只完成了前32字节），而 `elec_offset` 读到损坏的浮点数值时，电机将使用错误的换相角度。

**修复建议：**

```c
void Read_MotorData(void) {
    for(uint16_t i=0; i<32; i++)
        flash_buffer[i] = *(uint32_t*)(DATA_FLASH_ADDR + 4*i);

    // 1. 检查magic number
    if (flash_buffer[31] != FLASH_MAGIC) {
        printf("Flash: no valid data\r\n");
        return;
    }

    // 2. 检查浮点数合理性
    float offset = uint2float(flash_buffer[0]);
    if (isnan(offset) || isinf(offset) || offset < 0 || offset > PI_TIMES_2) {
        printf("Flash: invalid elec_offset\r\n");
        return;
    }

    // 3. 正常加载...
}
```

---

### F-05 [中等] FDCAN_ID 写入但未读回

**问题描述：**

```c
// flash.c:67 - Write时写入了FDCAN_ID
flash_buffer[18] = FDCAN_ID;

// flash.c:127-157 - Read时没有恢复FDCAN_ID
// 缺少: FDCAN_ID = flash_buffer[18];
```

**后果：** 每次上电，`FDCAN_ID` 始终恢复为 `fdcan.c:24` 中的默认值 `1`，通过CAN修改的节点地址无法持久保存。

**修复：**

```c
// flash.c Read_MotorData() 末尾添加:
if (flash_buffer[18] != 0 && flash_buffer[18] != 0xFFFFFFFF) {
    FDCAN_ID = (uint8_t)flash_buffer[18];
}
```

---

### F-06 [中等] 校准后无条件写Flash

**问题描述：**

```c
// main.c:367-389
if(p_encoder_g->cali_start == 1) {
    p_encoder_g->cali_start = 0;
    calibrate();
}
if(p_motor_g->cali_start == 1) {
    p_motor_g->cali_start = 0;
    Motor.MeasureResistance();
    Motor.MeasureInductance();
}
disablePWM();

/*校准后Flash参数写入*/
Write_MotorData();  // 无论是否真正执行了校准，都会擦写Flash!
```

如果进入 `CALIBRATION_MODE` 但 `cali_start` 都不为1，仍然会执行一次无意义的Flash擦写操作，增加Flash磨损和掉电数据丢失风险。

**修复：**

```c
bool bNeedSave = false;
if(p_encoder_g->cali_start == 1) {
    p_encoder_g->cali_start = 0;
    calibrate();
    bNeedSave = true;
}
if(p_motor_g->cali_start == 1) {
    p_motor_g->cali_start = 0;
    Motor.MeasureResistance();
    Motor.MeasureInductance();
    bNeedSave = true;
}
disablePWM();
if(bNeedSave) {
    Write_MotorData();
}
```

---

### F-07 [低] flash_buffer 缺少对齐声明

**问题描述：**

```c
// flash.c:8
uint32_t flash_buffer[32]; // 位于 0x24009b7c (AXI SRAM)
```

STM32H7的 `HAL_FLASH_Program` 要求源数据地址为 **32-bit aligned**（当前满足，因为`uint32_t`自然4字节对齐）。但HAL内部按word逐个拷贝到flash控制器FIFO时，若AXI SRAM存在cache一致性问题，可能读到stale数据。

如果使能了D-Cache（STM32H7常见配置），`flash_buffer` 在 AXI SRAM (`0x24xxxxxx`) 中，写入前应确保cache已刷新。

**修复建议：**

```c
// 添加对齐声明和cache-safe属性
__ALIGNED(32) uint32_t flash_buffer[32];

// 或者在Write_MotorData中写入前刷新cache:
SCB_CleanDCache_by_Addr((uint32_t*)flash_buffer, sizeof(flash_buffer));
```

---

### F-08 [低] Flash_Init 异常路径处理不完整

**问题描述：**

```c
HAL_StatusTypeDef Flash_Init(void) {
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;  // 返回错误，但Flash可能处于中间状态
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
    return HAL_OK;  // 返回OK，但Flash处于Unlock状态
}
```

`Flash_Init` 成功后Flash处于解锁状态，如果后续 `Flash_EraseSector` 或 `HAL_FLASH_Program` 失败并return，部分异常路径会调用 `HAL_FLASH_Lock()`，但依赖每个错误分支都正确处理。

当前 `Write_MotorData` 中各错误路径都有 `HAL_FLASH_Lock()`，这点是正确的。但建议使用 `goto cleanup` 模式统一处理。

---

### F-09 [建议] 无磨损均衡

128KB扇区的擦写寿命约 10,000 次（STM32H7 datasheet 典型值）。

当前每次保存参数都擦除整个扇区。如果通过CAN频繁修改参数（每个参数修改都触发 `bSaveDataFlag -> Write_MotorData`），Flash寿命可能快速耗尽。

建议：只有在参数实际发生变化时才执行写入，或使用多page轮转写入机制。

---

## 4. 地址冲突检查

### 4.1 Flash地址空间

```
0x08000000 +--------------------------+
           |                          |
           |  固件代码 (RO)           |  ~115KB
           |  (Bank1 Sector 0~1)      |
           |                          |
0x0801CE04 +--------------------------+  <- 代码结束 (约)
           |                          |
           |  未使用空间              |
           |                          |
0x081E0000 +--------------------------+
           |  参数数据区 (RW)         |  128 Bytes 实际使用
           |  Bank2 Sector7           |  128KB 扇区
0x081FFFFF +--------------------------+
```

**结论：代码区与数据区无地址冲突。** 固件仅占用约115KB，远小于Bank1的1MB容量，与Bank2 Sector7 (`0x081E0000`) 之间有约1.76MB的安全间距。

### 4.2 Flash写入地址对齐

| 写入操作 | 目标地址 | 32字节对齐 | 源buffer索引 |
|----------|----------|-----------|-------------|
| 第1次 | `0x081E0000` | 0x00 % 32 = 0 | [0..7] |
| 第2次 | `0x081E0020` | 0x20 % 32 = 0 | [8..15] |
| 第3次 | `0x081E0040` | 0x40 % 32 = 0 | [16..23] |

**结论：所有写入目标地址均满足256-bit (32字节) 对齐要求。**

### 4.3 RAM地址

| 变量 | 地址 | 区域 |
|------|------|------|
| `flash_buffer[32]` | `0x24009B7C` | AXI SRAM (0x24000000~0x2407FFFF) |

**结论：RAM区域使用正常，与Flash地址无冲突。**

### 4.4 Linker Scatter文件

```
LR_IROM1 0x08000000 0x00200000  {        // 加载区: 整个2MB Flash
  ER_IROM1 0x08000000 0x00200000  {      // 代码/只读区
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
    .ANY (+XO)
  }
}
```

**隐患：** Scatter文件将整个2MB (`0x00200000`) 分配给代码区，理论上代码段可以延伸到 `0x081FFFFF`，覆盖参数数据区。当前固件仅115KB不会触发问题，但如果未来固件膨胀超过 `0x081E0000 - 0x08000000 = 1,966,080` 字节（~1920KB），代码将覆盖参数区。

**建议修改scatter文件，为参数区保留空间：**

```
LR_IROM1 0x08000000 0x001E0000  {        // 限制代码区到 Bank2 Sector7 之前
  ER_IROM1 0x08000000 0x001E0000  {
    ...
  }
}
```

---

## 5. 初始化时序分析

```
main() {
    // 外设初始化...
    Motor_Init();       // motor_calibrated = 0
    Encoder_Init();     // cali_finish = 0, elec_offset = 1.719367(硬编码默认值)

    Read_MotorData();   // 从Flash加载，覆盖默认值（如果数据有效）

    init_controller_params();  // 使用加载后的参数初始化控制器
    // ...
    TIM1->CR1 ^= TIM_CR1_UDIS; // 启动FOC中断
}
```

**时序分析：** 初始化顺序正确。先用默认值初始化结构体，再从Flash读取覆盖。问题在于 Flash 数据无效时（F-03/F-04），会使用 `Encoder_Init()` 中的硬编码值 `elec_offset = 1.719367`，这个值不一定适用于当前电机。

---

## 6. 校准偏移失效根因分析

综合以上发现，**电角度/机械角度偏移校准有时失效**的可能原因排序：

### 原因1 (最可能): ISR中Flash操作打断校准

校准过程耗时数秒（`calibration.c` 中多处 `HAL_Delay`）。期间如果CAN消息到达并触发 `CAN_MsgProcess`：

- **场景A:** CAN参数修改触发 `Write_MotorData` -> 擦除Flash -> 写入**未完成校准**的数据 -> 校准完成后主循环的 `Write_MotorData` 写入正确数据。但如果此时恰好掉电/复位，Flash中保存的是错误数据。

- **场景B:** ISR中的 `Write_MotorData` 与主循环的 `Write_MotorData` 发生重入 -> Flash控制器状态被破坏 -> 数据损坏。

### 原因2: 掉电时机不巧

校准完成后 `Write_MotorData` 执行期间（擦除~2秒 + 编程），如果掉电，数据丢失。

### 原因3: Flash数据无校验

Flash中存储了损坏数据但 `cali_finish` 恰好为1，导致加载了错误的 `elec_offset`。

---

## 7. 修复优先级建议

| 优先级 | 修复项 | 预估工作量 |
|--------|--------|-----------|
| **P0 - 立即修复** | F-01: 将ISR中的Write_MotorData改为设标志位 | 0.5h |
| **P0 - 立即修复** | F-02: Flash操作期间关中断或确保在安全上下文执行 | 0.5h |
| **P1 - 尽快修复** | F-04: 添加magic number有效性校验 | 1h |
| **P1 - 尽快修复** | F-05: Read_MotorData中补充FDCAN_ID读回 | 5min |
| **P2 - 计划修复** | F-03: 实现双区交替写入的掉电保护 | 4h |
| **P2 - 计划修复** | F-06: 校准后条件性写入Flash | 15min |
| **P3 - 优化** | F-07: flash_buffer对齐 + D-Cache flush | 15min |
| **P3 - 优化** | Scatter文件限制代码区范围 | 5min |
| **P3 - 优化** | F-09: 写入前比较数据是否变化 | 1h |

---

## 8. 涉及文件清单

| 文件路径 | 关注点 |
|----------|--------|
| `UserSrc/Src/flash.c` | Flash读写核心实现 |
| `UserSrc/Inc/flash.h` | Flash地址宏定义 |
| `Core/Src/main.c:157,384` | Flash读取/校准后写入 |
| `Core/Src/stm32h7xx_it.c:640,1030` | 中断中调用Flash写入 |
| `UserSrc/Src/can_rv.c:511,812` | CAN消息处理中调用Flash写入 |
| `FOC/calibration.c:190-191` | 校准结果赋值 |
| `FOC/encoder.c:53-59` | 编码器默认值初始化 |
| `FOC/motor.c:238,245` | 电机校准标志设置 |
| `Core/Inc/fdcan.c:24` | FDCAN_ID默认值 |
| `MDK-ARM/FIVE/FIVE.sct` | Linker scatter文件 |

---

*Report generated by code review - Claude*
