# SOGI滤波器使用示例

## 快速开始

### 1. 基本使用流程

```c
// 在user_regulator.c中，SOGI滤波器已经集成到ADC回调函数中
void user_regulator_adc_callback(const ADC_HandleTypeDef* hadc)
{
    // ... 其他代码 ...
    
    // 获取原始电流值
    float current_A_raw = ((int16_t)adc_ac_buf[0] - IacOffset) * I_MeasureGain;
    float current_B_raw = ((int16_t)adc_ac_buf[2] - IacOffset) * I_MeasureGain;
    
    // 应用SOGI滤波器 - 自动去除噪声，保留50Hz基波
    float current_A_filtered = SOGI_Filter_Update(&sogi_filter_ia, current_A_raw);
    float current_B_filtered = SOGI_Filter_Update(&sogi_filter_ib, current_B_raw);
    
    // 使用滤波后的干净信号进行控制
    Current_Controller_AlphaBeta_Update(current_reference_peak, current_A_filtered, current_B_filtered);
    
    // ... 其他代码 ...
}
```

### 2. 运行测试

如果您想验证SOGI滤波器的效果，可以在main函数中添加测试：

```c
int main(void)
{
    // ... 系统初始化 ...
    
    // 运行SOGI滤波器测试（可选）
    #ifdef DEBUG
    SOGI_Filter_Test();
    #endif
    
    // 进入主循环
    while (1)
    {
        user_regulator_main();
        
        // 实时监控滤波效果（可选）
        #ifdef DEBUG
        static uint32_t last_monitor = 0;
        if (HAL_GetTick() - last_monitor > 1000) {
            // 这里可以添加监控代码
            last_monitor = HAL_GetTick();
        }
        #endif
    }
}
```

## 观察滤波效果

### 1. 通过串口观察

在ADC回调函数中添加调试输出：

```c
// 在user_regulator_adc_callback中添加
#ifdef SOGI_DEBUG
static uint32_t debug_counter = 0;
debug_counter++;
if (debug_counter >= 1000) {  // 每1000次输出一次，避免过于频繁
    debug_counter = 0;
    printf("Raw_A:%.3f,Filt_A:%.3f,Raw_B:%.3f,Filt_B:%.3f\r\n",
           current_A_raw, current_A_filtered, current_B_raw, current_B_filtered);
}
#endif
```

### 2. 通过OLED显示

可以在显示函数中添加滤波效果显示：

```c
void Display_Manual_Mode_Page(void)
{
    // ... 现有显示代码 ...
    
    // 添加滤波器状态显示
    OLED_Println(OLED_6X8, "SOGI: ON, 50Hz Filter");
    
    // ... 其他显示代码 ...
}
```

## 性能调优

### 1. 调整SOGI参数

如果需要调整滤波器性能，可以修改sogi_pll.h中的参数：

```c
// 在sogi_pll.h中
#define SOGI_K_GAIN             1.41f        // 增益系数
#define SAMPLING_FREQ           10000.0f     // 采样频率
#define GRID_FREQ_NOMINAL       50.0f        // 标称频率
```

### 2. 参数调整指南

- **增大SOGI_K_GAIN**：
  - 优点：响应更快，跟踪性能更好
  - 缺点：可能引入振荡，稳定性降低
  - 建议范围：1.0 - 2.0

- **减小SOGI_K_GAIN**：
  - 优点：更稳定，噪声抑制更好
  - 缺点：响应较慢，动态性能下降
  - 建议范围：0.5 - 1.0

### 3. 频率适应性

如果电网频率不是严格的50Hz，可以考虑：

```c
// 动态调整标称频率（高级功能）
void Adjust_SOGI_Frequency(float detected_freq)
{
    // 这需要修改SOGI算法，使其能够适应频率变化
    // 当前实现针对50Hz优化，如需此功能请联系开发者
}
```

## 故障排除

### 1. 滤波效果不明显

**可能原因**：
- 输入信号噪声频率接近50Hz
- SOGI参数设置不当
- 采样频率不足

**解决方案**：
```c
// 检查采样频率是否足够
#if SAMPLING_FREQ < (GRID_FREQ_NOMINAL * 20)
#warning "采样频率可能不足，建议至少为信号频率的20倍"
#endif

// 检查输入信号范围
if (fabsf(current_A_raw) > 10.0f) {
    // 输入信号可能超出正常范围
    printf("Warning: Input signal out of range: %.3f\r\n", current_A_raw);
}
```

### 2. 系统不稳定

**可能原因**：
- SOGI增益过大
- 数值溢出
- 初始化不正确

**解决方案**：
```c
// 添加数值范围检查
static float SOGI_Filter_Update_Safe(SOGI_State *sogi, float input)
{
    // 限制输入范围
    if (input > 100.0f) input = 100.0f;
    if (input < -100.0f) input = -100.0f;
    
    // 调用原始更新函数
    float output = SOGI_Filter_Update(sogi, input);
    
    // 检查输出是否合理
    if (!isfinite(output)) {
        // 如果输出不是有限数，重新初始化滤波器
        SOGI_Filter_Init();
        return 0.0f;
    }
    
    return output;
}
```

### 3. 相位延迟问题

SOGI滤波器在50Hz时理论上无相位延迟，但如果观察到相位问题：

```c
// 检查系统时序
void Check_Timing(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t interval = current_time - last_time;
    
    if (interval > 1) {  // 期望1ms间隔
        printf("Warning: Timing issue detected, interval: %lu ms\r\n", interval);
    }
    
    last_time = current_time;
}
```

## 扩展应用

### 1. 多频率滤波

如果需要同时滤波多个频率：

```c
// 为不同频率创建不同的SOGI滤波器
static SOGI_State sogi_50hz;   // 50Hz基波
static SOGI_State sogi_150hz;  // 150Hz三次谐波
static SOGI_State sogi_250hz;  // 250Hz五次谐波

// 分别进行滤波
float fundamental = SOGI_Filter_Update(&sogi_50hz, input);
float harmonic_3rd = SOGI_Filter_Update(&sogi_150hz, input);
float harmonic_5th = SOGI_Filter_Update(&sogi_250hz, input);
```

### 2. 自适应滤波

```c
// 根据信号质量自动调整滤波强度
void Adaptive_SOGI_Filter(float *input, float *output, float signal_quality)
{
    static float adaptive_gain = SOGI_K_GAIN;
    
    if (signal_quality < 0.5f) {
        // 信号质量差，增强滤波
        adaptive_gain = SOGI_K_GAIN * 0.8f;
    } else {
        // 信号质量好，减弱滤波以保持响应速度
        adaptive_gain = SOGI_K_GAIN * 1.2f;
    }
    
    // 使用自适应增益更新滤波器
    // 注意：这需要修改SOGI_Filter_Update函数以接受可变增益
}
```

## 总结

SOGI滤波器已经成功集成到您的三相SPWM逆变器控制系统中，为50Hz电流信号提供了专业级的噪声抑制。通过本指南，您可以：

1. 理解SOGI滤波器的工作原理
2. 监控和调试滤波效果
3. 根据需要调整参数
4. 解决可能遇到的问题
5. 扩展到更复杂的应用

如果您在使用过程中遇到任何问题，请参考故障排除部分或联系技术支持。
