# Motor H7 Project

## 工程概述
- 项目：STM32H7 FOC 电机控制工程
- MCU：STM32H743VITx
- IDE：Keil MDK-ARM (uVision)
- CubeMX 工程文件：FIVE.ioc
- Keil 工程文件：MDK-ARM/FIVE.uvprojx

## 目录结构
- `Core/` — STM32 HAL 初始化代码（CubeMX 生成）
- `Drivers/` — CMSIS 和 HAL 驱动库
- `FOC/` — FOC 电机控制算法
- `MotionControl/` — 运动控制模块
- `UserSrc/` — 用户应用代码
- `MDK-ARM/` — Keil 工程及编译输出

## Git 环境
- 远程仓库：https://github.com/syx0000/motor_h7.git
- 主分支：main
- 用户：yxsui <yxsui2@iflytek.com>
- 协议：HTTPS

## 编码规范

### 代码风格
- **命名规范**
  - 函数：PascalCase（如 `EncoderSample`, `CurrentSample`, `EnablePWM`）
  - 宏定义：全大写+下划线（如 `V_WINDOW_N`, `RES_DIVIDE_MOS`, `CLAMP`）
  - 变量：camelCase（如 `motorState`, `canTimeout`）
  
- **格式规范**
  - 缩进：Tab（4空格宽度）
  - 行尾：CRLF（Windows）
  - 编码：UTF-8（无BOM）
  - 关键字后加空格：`if (`, `for (`, `while (`, `switch (`
  - 赋值运算符前后加空格：`a = b`
  - 最多连续2个空行
  - 文件末尾保留换行符

- **语言规范**
  - C 语言（嵌入式 ARM）
  - 使用 STM32 HAL 库
  - 中文注释
  - 浮点常量添加 `f` 后缀（如 `0.5f`, `3.14f`）
  - ISR 与主循环共享变量必须添加 `volatile`
  - 避免严格别名违规（使用 union 进行类型双关）

## 编译方案

### 编译环境
- IDE：Keil MDK-ARM (uVision)
- 编译器：ARM Compiler 6 (ARMCLANG)
- 优化级别：-O2
- 目标：STM32H743VITx

### 编译标准
- **目标：0 error, 0 warning**
- 编译输出：
  - `MDK-ARM/FIVE.axf` — ELF 可执行文件
  - `MDK-ARM/FIVE.hex` — Intel HEX 格式（~189KB）
  - `MDK-ARM/FIVE.bin` — 二进制格式（~67KB）
  
### 编译产物管理
- `MDK-ARM/FIVE/` 目录下的所有编译产物（.o, .d, .crf, .axf, .map 等）不纳入 Git 管理
- `.gitignore` 已配置忽略规则
- 工程文件 `FIVE.uvprojx` 和 `FIVE.ioc` 纳入版本控制
