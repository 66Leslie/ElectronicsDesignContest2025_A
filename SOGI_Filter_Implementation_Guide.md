# SOGI滤波器实现指南

## 概述

本文档描述了在三相SPWM逆变器项目中实现的SOGI（二阶广义积分器）滤波器，用于处理50Hz交流电流信号中的噪声。

## 实现原理

### SOGI滤波器特点
1. **精准调谐**：中心频率精确设置在50Hz
2. **零相位延迟**：在50Hz频率点无相位延迟
3. **高噪声抑制**：大幅衰减其他频率的噪声
4. **正交输出**：同时生成90度相差的正交信号

### 核心算法
SOGI滤波器使用以下离散时间方程：

```
v_err = input - v_alpha
z1 = z1 + (k*v_err - z2) * ω_nominal * Ts
v_alpha = v_alpha + z1 * Ts  
z2 = z2 + v_alpha * ω_nominal * Ts
v_beta = z2 * Ts
```

其中：
- `k = 1.41f` (SOGI增益系数)
- `ω_nominal = 2π × 50` (标称角频率)
- `Ts = 1/10000` (采样周期，10kHz)

## 代码实现

### 1. 滤波器实例定义
```c
// 为A相和B相电流分别创建SOGI滤波器实例
static SOGI_State sogi_filter_ia; // A相电流的SOGI滤波器
static SOGI_State sogi_filter_ib; // B相电流的SOGI滤波器
```

### 2. 初始化函数
```c
static void SOGI_Filter_Init(void)
{
    // 重置A相电流滤波器的状态
    sogi_filter_ia.v_alpha = 0.0f;
    sogi_filter_ia.v_beta = 0.0f;
    sogi_filter_ia.z1 = 0.0f;
    sogi_filter_ia.z2 = 0.0f;

    // 重置B相电流滤波器的状态  
    sogi_filter_ib.v_alpha = 0.0f;
    sogi_filter_ib.v_beta = 0.0f;
    sogi_filter_ib.z1 = 0.0f;
    sogi_filter_ib.z2 = 0.0f;
}
```

### 3. 滤波器更新函数
```c
static float SOGI_Filter_Update(SOGI_State *sogi, float input)
{
    float v_err = input - sogi->v_alpha;
    
    // 更新SOGI内部状态
    sogi->z1 = sogi->z1 + (SOGI_K_GAIN * v_err - sogi->z2) * OMEGA_NOMINAL * SAMPLING_PERIOD;
    sogi->v_alpha = sogi->v_alpha + sogi->z1 * SAMPLING_PERIOD;
    sogi->z2 = sogi->z2 + sogi->v_alpha * OMEGA_NOMINAL * SAMPLING_PERIOD;
    sogi->v_beta = sogi->z2 * SAMPLING_PERIOD;

    return sogi->v_alpha; // 返回滤波后的信号
}
```

## 集成到控制系统

### ADC回调函数中的应用
```c
// 1. 获取原始电流值
float current_A_raw = ((int16_t)adc_ac_buf[0] - IacOffset) * I_MeasureGain;
float current_B_raw = ((int16_t)adc_ac_buf[2] - IacOffset) * I_MeasureGain;

// 2. 应用SOGI滤波器
float current_A_filtered = SOGI_Filter_Update(&sogi_filter_ia, current_A_raw);
float current_B_filtered = SOGI_Filter_Update(&sogi_filter_ib, current_B_raw);

// 3. 使用滤波后的值进行控制
Current_Controller_AlphaBeta_Update(current_reference_peak, current_A_filtered, current_B_filtered);

// 4. 使用滤波后的值计算RMS
current_A_sum += current_A_filtered * current_A_filtered;
current_B_sum += current_B_filtered * current_B_filtered;
```

## 预期效果

### 滤波前的问题
- 电流信号包含高频噪声
- PI控制器响应不稳定
- PWM输出存在抖动

### 滤波后的改善
- 50Hz基波信号保持完整
- 高频噪声被大幅抑制
- 控制器响应更加平滑
- PWM输出更加稳定

## 调试和验证

### 1. 信号观察
可以通过串口输出观察滤波前后的信号：
```c
printf("Raw: %.3f, Filtered: %.3f\r\n", current_A_raw, current_A_filtered);
```

### 2. 频率响应验证
- 在50Hz时，滤波器增益接近1，相位延迟接近0
- 在其他频率时，增益显著降低

### 3. 控制性能评估
- 观察PI控制器输出的稳定性
- 检查PWM占空比的平滑度
- 测量输出电流的THD（总谐波失真）

## 参数调整

如果需要调整滤波器性能，可以修改以下参数：

1. **SOGI_K_GAIN**：影响滤波器的阻尼特性
   - 增大：响应更快，但可能引入振荡
   - 减小：更稳定，但响应较慢

2. **采样频率**：当前为10kHz，足够处理50Hz信号
   - 确保采样频率至少是信号频率的20倍

## 注意事项

1. **初始化顺序**：确保在使用前调用`SOGI_Filter_Init()`
2. **数值稳定性**：SOGI算法在正常参数下是稳定的
3. **计算开销**：每次更新需要约10个浮点运算，在10kHz下可接受
4. **内存使用**：每个滤波器实例占用16字节（4个float）

## 扩展应用

除了电流滤波，SOGI滤波器还可以用于：
- 电压信号滤波
- 锁相环中的信号预处理
- 谐波检测和分析
- 功率计算中的基波提取

## 总结

SOGI滤波器为50Hz交流信号提供了理想的滤波解决方案，在保持基波信号完整性的同时有效抑制噪声，显著改善了控制系统的性能。
