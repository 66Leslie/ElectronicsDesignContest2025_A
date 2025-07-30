/*
 * sogi_filter_example.c
 *
 *  Created on: 2025年7月29日
 *      Author: Augment Agent
 *  Description: SOGI滤波器使用示例，用于处理50Hz带有1.65偏置的正弦信号
 */

#include "data_process.h"
#include "main.h"

// ============================================================================
// SOGI滤波器使用示例
// ============================================================================

// 全局变量定义
SOGICompositeFilter_t sogi_filter;  // SOGI复合滤波器实例

/*
 * 函数名：void SOGI_Filter_Example_Init(void)
 * 作用：初始化SOGI滤波器示例
 * 备注：配置用于处理20kHz采样的50Hz正弦信号（带1.65V偏置）
 */
void SOGI_Filter_Example_Init(void)
{
    // 初始化SOGI复合滤波器
    // 参数说明：
    // - 目标频率：50Hz
    // - 采样频率：20kHz
    // - 阻尼系数：1.414 (√2，最优响应)
    // - 限幅最大变化：0.5V (根据信号特性调整)
    // - 一阶低通滤波系数：0.2f (alpha值)
    // - 初始值：1.65V (信号偏置值)
    SOGICompositeFilter_Init(&sogi_filter,
                            SOGI_TARGET_FREQ,      // 50Hz
                            SOGI_SAMPLING_FREQ,    // 20kHz
                            SOGI_DAMPING_FACTOR,   // 1.414
                            0.5f,                  // 限幅变化量
                            0.2f,                  // 一阶低通滤波系数
                            1.65f);                // 初始值（偏置）
}

/*
 * 函数名：float SOGI_Filter_Process_Signal(float input_signal)
 * 作用：处理输入信号
 * 输入：input_signal - 原始输入信号（50Hz正弦波 + 1.65V偏置 + 噪声）
 * 返回值：float - 滤波后的信号（提取的50Hz基波分量）
 * 备注：适用于20kHz采样率下的实时信号处理
 */
float SOGI_Filter_Process_Signal(float input_signal)
{
    // 使用SOGI复合滤波器处理信号
    float filtered_signal = SOGICompositeFilter_Update(&sogi_filter, input_signal);
    
    return filtered_signal;
}

/*
 * 函数名：void SOGI_Filter_Reset(void)
 * 作用：重置SOGI滤波器状态
 * 备注：在需要重新开始滤波时调用
 */
void SOGI_Filter_Reset(void)
{
    SOGICompositeFilter_Reset(&sogi_filter);
}

/*
 * 函数名：void SOGI_Filter_Test_Function(void)
 * 作用：SOGI滤波器测试函数
 * 备注：生成测试信号并验证滤波效果
 */
void SOGI_Filter_Test_Function(void)
{
    // 测试参数
    const uint16_t test_samples = 400;  // 测试样本数（对应20ms，一个50Hz周期）
    const float amplitude = 2.0f;       // 信号幅值
    const float dc_offset = 1.65f;      // 直流偏置
    const float noise_level = 0.1f;     // 噪声水平
    
    // 初始化滤波器
    SOGI_Filter_Example_Init();
    
    // 生成测试信号并进行滤波
    for (uint16_t i = 0; i < test_samples; i++)
    {
        // 生成50Hz正弦信号 + 偏置 + 噪声
        float time = (float)i / SOGI_SAMPLING_FREQ;
        float clean_signal = amplitude * sinf(2.0f * M_PI * SOGI_TARGET_FREQ * time) + dc_offset;
        
        // 添加随机噪声（简化的噪声模拟）
        float noise = noise_level * (((float)(i % 100) / 50.0f) - 1.0f);
        float noisy_signal = clean_signal + noise;
        
        // 进行滤波
        float filtered_signal = SOGI_Filter_Process_Signal(noisy_signal);
        
        // 这里可以添加数据记录或输出代码
        // 例如：通过UART输出、存储到数组等
        (void)filtered_signal;  // 避免编译器警告
    }
}

// ============================================================================
// 高级SOGI滤波器功能
// ============================================================================

/*
 * 函数名：float SOGI_Get_Quadrature_Component(void)
 * 作用：获取SOGI滤波器的正交分量
 * 返回值：float - 正交分量（相位滞后90度的信号）
 * 备注：可用于相位检测、功率计算等应用
 */
float SOGI_Get_Quadrature_Component(void)
{
    return sogi_filter.sogi_filter.y2;
}

/*
 * 函数名：float SOGI_Get_Signal_Amplitude(void)
 * 作用：计算信号幅值
 * 返回值：float - 信号幅值
 * 备注：使用同相和正交分量计算瞬时幅值
 */
float SOGI_Get_Signal_Amplitude(void)
{
    float in_phase = sogi_filter.sogi_filter.y1;      // 同相分量
    float quadrature = sogi_filter.sogi_filter.y2;    // 正交分量
    
    // 计算幅值：sqrt(I^2 + Q^2)
    return sqrtf(in_phase * in_phase + quadrature * quadrature);
}

/*
 * 函数名：float SOGI_Get_Signal_Phase(void)
 * 作用：计算信号相位
 * 返回值：float - 信号相位（弧度）
 * 备注：使用同相和正交分量计算瞬时相位
 */
float SOGI_Get_Signal_Phase(void)
{
    float in_phase = sogi_filter.sogi_filter.y1;      // 同相分量
    float quadrature = sogi_filter.sogi_filter.y2;    // 正交分量
    
    // 计算相位：atan2(Q, I)
    return atan2f(quadrature, in_phase);
}

// ============================================================================
// 自适应SOGI滤波器（可选功能）
// ============================================================================

/*
 * 函数名：void SOGI_Adaptive_Frequency_Update(float estimated_freq)
 * 作用：自适应频率更新
 * 输入：estimated_freq - 估计的频率值
 * 备注：可根据实际电网频率变化动态调整SOGI滤波器参数
 */
void SOGI_Adaptive_Frequency_Update(float estimated_freq)
{
    // 限制频率范围（45Hz - 55Hz）
    if (estimated_freq < 45.0f) estimated_freq = 45.0f;
    if (estimated_freq > 55.0f) estimated_freq = 55.0f;
    
    // 更新SOGI滤波器的角频率
    sogi_filter.sogi_filter.omega = 2.0f * M_PI * estimated_freq;
}

/*
 * 函数名：float SOGI_Estimate_Frequency(void)
 * 作用：估计信号频率
 * 返回值：float - 估计的频率值（Hz）
 * 备注：基于相位变化率估计频率（简化实现）
 */
float SOGI_Estimate_Frequency(void)
{
    static float last_phase = 0.0f;
    static uint32_t last_time = 0;
    
    float current_phase = SOGI_Get_Signal_Phase();
    uint32_t current_time = HAL_GetTick();  // 假设使用HAL库
    
    if (last_time != 0)
    {
        float phase_diff = current_phase - last_phase;
        float time_diff = (float)(current_time - last_time) / 1000.0f;  // 转换为秒
        
        // 处理相位跳变
        if (phase_diff > M_PI) phase_diff -= 2.0f * M_PI;
        if (phase_diff < -M_PI) phase_diff += 2.0f * M_PI;
        
        // 计算频率
        float estimated_freq = fabsf(phase_diff) / (2.0f * M_PI * time_diff);
        
        last_phase = current_phase;
        last_time = current_time;
        
        return estimated_freq;
    }
    
    last_phase = current_phase;
    last_time = current_time;
    
    return SOGI_TARGET_FREQ;  // 返回默认频率
}
