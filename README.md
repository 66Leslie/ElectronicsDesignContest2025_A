**中文** | [English](README.en.md)

# 三相逆变与SPWM控制平台

本仓库包含 2025 全国大学生电子设计大赛 A 题的工程实现（获国家二等奖），基于 STM32G474 微控制器完成三相逆变器的正弦脉宽调制（SPWM）控制、锁相与数据采集，并附带 PLECS 仿真模型与硬件配置文件。

## 目录结构
- **Core/**：STM32CubeMX 生成的启动与外设初始化代码，以及应用层源码。
  - `Core/user/`：用户算法，包括按键处理、数据处理、SOGI-PLL/QSG 锁相模块与主控制循环实现。  
  - `Core/Src/`：外设驱动、OLED 显示等支撑代码。
- **Drivers/**：STM32G4 HAL 与 CMSIS 设备支持库。
- **cmake/**：构建脚本，负责将 CubeMX 生成的源码与用户算法编译为可执行固件。
- **three_phase_spwm.ioc**：CubeMX 工程文件，可用于重新生成外设初始化代码。
- **A.plecs / Three_phase_inverter.plecs / triple_phase.plecs**：三相逆变器与控制环节的 PLECS 仿真模型。
- **CMakeLists.txt / CMakePresets.json**：项目顶层构建配置。

> 说明：仓库包含一个采用空间矢量脉宽调制（SVPWM）实现的分支，用于进一步提升调制利用率；如需体验，请切换到对应分支。

## 环境准备
- CMake ≥ 3.22
- GNU Arm Embedded Toolchain（`arm-none-eabi-gcc`、`arm-none-eabi-gdb` 等）
- Ninja 或 Make 构建后端
- （可选）STM32CubeMX 以调整 `three_phase_spwm.ioc` 并重新生成代码

## 快速构建
```bash
# 生成构建目录并配置
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/stm32cubemx/arm-none-eabi.cmake -G Ninja

# 编译固件
cmake --build build
```
构建完成后，生成的固件与中间文件位于 `build/` 目录，可使用 ST-Link、OpenOCD 或其他支持的刷写工具下载到目标板。

## 主要功能概览
- 三相 SPWM 发生器与门极驱动控制。
- SOGI-PLL/QSG 基于双正交信号生成器的同步锁相。
- DMA 采样的多路 ADC 电流/电压测量与数据处理。
- OLED 显示与按键输入，用于运行状态观察与参数调节。

## 许可与致谢
本工程基于 STM32CubeMX 生成代码，相关 HAL/CMSIS 库遵循其上游授权条款。若无额外许可证文件，代码以 AS-IS 形式提供。
