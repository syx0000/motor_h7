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
- C 语言（嵌入式 ARM）
- 使用 STM32 HAL 库
- 中文注释
