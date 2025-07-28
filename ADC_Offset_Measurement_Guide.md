# ADC偏置测量功能使用指南

## 功能概述

本功能在系统上电初始化时自动测量ADC的电压和电流偏置，确保在PWM关闭状态下获得准确的零点偏置值。

## 实现原理

1. **自动测量**: 系统上电后进入`PowerUp_Check_State`状态，自动开始偏置测量
2. **TIM8触发**: 启动TIM8产生ADC触发信号，但保持shutdown口(PE7)为低电平关断PWM输出
3. **安全保护**: 测量期间PWM逻辑上关闭，物理上也无输出，确保安全
4. **多次采样**: 采集100次ADC数据取平均值，提高测量精度
5. **动态更新**: 测量完成后更新全局偏置变量`VacOffset`和`IacOffset`

## 测量过程

### 自动测量流程
1. 系统上电后自动进入偏置测量状态
2. 调用`Start_TIM8_For_Offset_Measurement()`启动TIM8
3. 保持shutdown口(PE7)为低电平，确保PWM物理输出关闭
4. TIM8产生TRGO信号触发ADC采样
5. 连续采集100次ADC数据：
   - ADC1_IN1: A相电流
   - ADC1_IN2: AB线电压
   - ADC2_IN3: B相电流
   - ADC2_IN4: BC线电压
6. 计算各通道的平均偏置值
7. 更新全局偏置变量
8. 完成后进入等待状态

### 测量参数
- **采样次数**: 100次 (可通过`OFFSET_SAMPLE_COUNT`宏调整)
- **电压偏置**: 使用AB和BC线电压的平均值
- **电流偏置**: 使用A相和B相电流的平均值
- **默认偏置**: 2047.0 (12位ADC中点值)

## 显示界面

### 测量进行中
```
Measuring Offsets...
Progress: 45/100
PWM: OFF (Required)
Please wait...

Current Offsets:
V:2047.0 I:2047.0
```

### 测量完成后
显示正常的运行界面，偏置值已更新到实际测量值。

## 手动重新测量

### 触发条件
- 按键组合: 同时按下KEY1+KEY2
- 前提条件: PWM必须处于关闭状态

### 操作步骤
1. 确保PWM已关闭 (按KEY3关闭PWM)
2. 同时按下KEY1和KEY2
3. 系统自动重新开始偏置测量
4. 等待测量完成

### 错误处理
如果在PWM开启状态下尝试重新测量，系统会显示错误信息：
```
Cannot measure offset while PWM is enabled
```

## 调试信息

### 串口输出示例
```
[regulator]info:Offset measurement reset - starting new measurement
[regulator]info:ADC Offset Measurement Complete:
[regulator]info:  Voltage Offset (AB): 2045.3, (BC): 2049.1, Final: 2047.2
[regulator]info:  Current Offset (A): 2046.8, (B): 2048.2, Final: 2047.5
[regulator]info:Offset measurement complete: VacOffset=2047.2, IacOffset=2047.5
```

## 技术细节

### 相关文件
- `Core/user/user_regulator.h`: 函数声明和宏定义
- `Core/user/user_regulator.c`: 功能实现

### 关键变量
```c
// 动态偏置变量
float VacOffset;  // 线电压偏置
float IacOffset;  // 相电流偏置

// 测量状态变量
static uint16_t offset_sample_count;
static uint8_t offset_measurement_complete;
```

### 关键函数
```c
void Measure_ADC_Offsets(void);                    // 执行偏置测量
uint8_t Is_Offset_Measurement_Complete(void);      // 检查测量状态
void Reset_Offset_Measurement(void);               // 重置测量
void Start_TIM8_For_Offset_Measurement(void);      // 启动TIM8用于偏置测量
```

## 注意事项

1. **安全要求**: 偏置测量期间shutdown口保持低电平，PWM物理输出被关断
2. **TIM8运行**: 测量期间TIM8运行产生ADC触发信号，但无PWM输出
3. **测量环境**: 建议在无负载或轻负载条件下进行测量
4. **测量时机**: 系统上电后自动进行，也可手动触发重新测量
5. **精度保证**: 通过100次采样平均值提高测量精度
6. **状态保护**: 只有在PWM逻辑关闭状态下才允许重新测量

## 故障排除

### 常见问题
1. **测量卡住**: 检查ADC DMA是否正常工作
2. **偏置异常**: 检查硬件电路是否正常
3. **无法重新测量**: 确保PWM已关闭

### 检查方法
1. 观察OLED显示的测量进度
2. 查看串口调试信息
3. 检查测量完成标志位

## 技术说明

### Shutdown口工作原理
- **PE7引脚**: 控制PWM驱动器的使能/关断
- **低电平(0V)**: 关断PWM输出，驱动器进入高阻态
- **高电平(3.3V)**: 使能PWM输出，驱动器正常工作
- **安全设计**: 即使TIM8产生PWM信号，shutdown低电平也能确保无输出

### TIM8与ADC触发关系
```
TIM8 Counter -> TIM8_TRGO -> ADC1/ADC2/ADC3 External Trigger
```
- TIM8计数器产生TRGO信号
- TRGO信号触发ADC转换
- 偏置测量期间需要TIM8运行来触发ADC
- 但通过shutdown口控制确保无PWM输出

## 扩展功能

### 可能的改进
1. 增加偏置值的合理性检查
2. 添加偏置值的非易失性存储
3. 支持更多ADC通道的偏置测量
4. 增加温度补偿功能
5. 添加偏置值的统计分析功能
