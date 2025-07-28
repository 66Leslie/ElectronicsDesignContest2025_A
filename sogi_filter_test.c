/*
 * sogi_filter_test.c
 * 
 * SOGI滤波器测试程序
 * 用于验证50Hz信号滤波效果
 * 
 * Created: 2025-07-28
 * Author: Augment Agent
 */

#include "sogi_pll.h"
#include <stdio.h>
#include <math.h>

// 测试参数
#define TEST_SAMPLES 1000       // 测试样本数
#define TEST_FREQ_50HZ 50.0f    // 50Hz基波
#define TEST_FREQ_NOISE 1000.0f // 1kHz噪声
#define NOISE_AMPLITUDE 0.1f    // 噪声幅度
#define SIGNAL_AMPLITUDE 1.0f   // 信号幅度

// 测试用SOGI滤波器实例
static SOGI_State test_sogi_filter;

// SOGI滤波器更新函数（从user_regulator.c复制）
static float SOGI_Filter_Update(SOGI_State *sogi, float input)
{
    float v_err = input - sogi->v_alpha;

    // 更新SOGI内部状态 (离散形式)
    sogi->z1 = sogi->z1 + (SOGI_K_GAIN * v_err - sogi->z2) * OMEGA_NOMINAL * SAMPLING_PERIOD;
    sogi->v_alpha = sogi->v_alpha + sogi->z1 * SAMPLING_PERIOD;
    sogi->z2 = sogi->z2 + sogi->v_alpha * OMEGA_NOMINAL * SAMPLING_PERIOD;
    sogi->v_beta = sogi->z2 * SAMPLING_PERIOD;

    return sogi->v_alpha;
}

// SOGI滤波器初始化函数
static void SOGI_Filter_Init_Test(SOGI_State *sogi)
{
    sogi->v_alpha = 0.0f;
    sogi->v_beta = 0.0f;
    sogi->z1 = 0.0f;
    sogi->z2 = 0.0f;
}

/**
 * @brief 生成测试信号（50Hz + 噪声）
 * @param sample_index 样本索引
 * @return 测试信号值
 */
static float Generate_Test_Signal(int sample_index)
{
    float time = (float)sample_index / SAMPLING_FREQ;
    
    // 50Hz基波信号
    float signal_50hz = SIGNAL_AMPLITUDE * sinf(2.0f * M_PI * TEST_FREQ_50HZ * time);
    
    // 高频噪声
    float noise = NOISE_AMPLITUDE * sinf(2.0f * M_PI * TEST_FREQ_NOISE * time);
    
    // 合成信号
    return signal_50hz + noise;
}

/**
 * @brief 计算信号的RMS值
 * @param data 信号数据数组
 * @param length 数据长度
 * @return RMS值
 */
static float Calculate_RMS(float *data, int length)
{
    float sum = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += data[i] * data[i];
    }
    return sqrtf(sum / length);
}

/**
 * @brief 计算信号的THD（总谐波失真）
 * @param original 原始信号数组
 * @param filtered 滤波后信号数组
 * @param length 数据长度
 * @return THD百分比
 */
static float Calculate_THD_Improvement(float *original, float *filtered, int length)
{
    float original_rms = Calculate_RMS(original, length);
    float filtered_rms = Calculate_RMS(filtered, length);
    
    // 计算噪声抑制比
    float noise_reduction_db = 20.0f * log10f(original_rms / filtered_rms);
    return noise_reduction_db;
}

/**
 * @brief SOGI滤波器测试主函数
 */
void SOGI_Filter_Test(void)
{
    printf("=== SOGI滤波器测试开始 ===\r\n");
    printf("测试参数:\r\n");
    printf("  采样频率: %.0f Hz\r\n", SAMPLING_FREQ);
    printf("  基波频率: %.0f Hz\r\n", TEST_FREQ_50HZ);
    printf("  噪声频率: %.0f Hz\r\n", TEST_FREQ_NOISE);
    printf("  信号幅度: %.2f\r\n", SIGNAL_AMPLITUDE);
    printf("  噪声幅度: %.2f\r\n", NOISE_AMPLITUDE);
    printf("  测试样本: %d\r\n\r\n", TEST_SAMPLES);

    // 初始化SOGI滤波器
    SOGI_Filter_Init_Test(&test_sogi_filter);

    // 分配测试数据数组
    static float input_signal[TEST_SAMPLES];
    static float filtered_signal[TEST_SAMPLES];
    static float error_signal[TEST_SAMPLES];

    // 生成测试数据并进行滤波
    printf("正在生成测试数据并进行滤波...\r\n");
    
    for (int i = 0; i < TEST_SAMPLES; i++) {
        // 生成带噪声的测试信号
        input_signal[i] = Generate_Test_Signal(i);
        
        // 应用SOGI滤波器
        filtered_signal[i] = SOGI_Filter_Update(&test_sogi_filter, input_signal[i]);
        
        // 计算滤波误差
        float ideal_signal = SIGNAL_AMPLITUDE * sinf(2.0f * M_PI * TEST_FREQ_50HZ * i / SAMPLING_FREQ);
        error_signal[i] = filtered_signal[i] - ideal_signal;
        
        // 每100个样本输出一次进度
        if (i % 100 == 0) {
            printf("进度: %d/%d (%.1f%%)\r\n", i, TEST_SAMPLES, (float)i * 100.0f / TEST_SAMPLES);
        }
    }

    // 计算性能指标
    float input_rms = Calculate_RMS(input_signal, TEST_SAMPLES);
    float filtered_rms = Calculate_RMS(filtered_signal, TEST_SAMPLES);
    float error_rms = Calculate_RMS(error_signal, TEST_SAMPLES);
    
    // 理论50Hz信号的RMS值
    float theoretical_rms = SIGNAL_AMPLITUDE / sqrtf(2.0f);  // 正弦波RMS = 峰值/√2
    
    printf("\r\n=== 测试结果 ===\r\n");
    printf("输入信号RMS:     %.4f\r\n", input_rms);
    printf("滤波后信号RMS:   %.4f\r\n", filtered_rms);
    printf("理论50Hz RMS:    %.4f\r\n", theoretical_rms);
    printf("滤波误差RMS:     %.4f\r\n", error_rms);
    
    // 计算滤波效果
    float noise_reduction = (input_rms - filtered_rms) / input_rms * 100.0f;
    float accuracy = (1.0f - fabsf(filtered_rms - theoretical_rms) / theoretical_rms) * 100.0f;
    
    printf("\r\n=== 性能评估 ===\r\n");
    printf("噪声抑制率:     %.2f%%\r\n", noise_reduction);
    printf("基波保持精度:   %.2f%%\r\n", accuracy);
    printf("滤波器误差:     %.4f (%.2f%%)\r\n", error_rms, error_rms / theoretical_rms * 100.0f);
    
    // 输出部分样本数据用于分析
    printf("\r\n=== 样本数据 (前20个) ===\r\n");
    printf("Index\tInput\t\tFiltered\tError\r\n");
    for (int i = 0; i < 20; i++) {
        printf("%d\t%.4f\t\t%.4f\t\t%.4f\r\n", i, input_signal[i], filtered_signal[i], error_signal[i]);
    }
    
    // 评估结果
    printf("\r\n=== 测试评估 ===\r\n");
    if (accuracy > 95.0f && error_rms < 0.05f) {
        printf("✓ SOGI滤波器性能优秀！\r\n");
    } else if (accuracy > 90.0f && error_rms < 0.1f) {
        printf("✓ SOGI滤波器性能良好。\r\n");
    } else {
        printf("⚠ SOGI滤波器性能需要优化。\r\n");
    }
    
    printf("=== SOGI滤波器测试完成 ===\r\n\r\n");
}

/**
 * @brief 实时SOGI滤波器性能监控
 * 可以在主循环中调用，监控实际运行中的滤波效果
 */
void SOGI_Filter_Monitor(float raw_current, float filtered_current)
{
    static float raw_sum = 0.0f;
    static float filtered_sum = 0.0f;
    static int sample_count = 0;
    static uint32_t last_report_time = 0;
    
    // 累积样本
    raw_sum += raw_current * raw_current;
    filtered_sum += filtered_current * filtered_current;
    sample_count++;
    
    // 每秒报告一次
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_report_time >= 1000) {
        if (sample_count > 0) {
            float raw_rms = sqrtf(raw_sum / sample_count);
            float filtered_rms = sqrtf(filtered_sum / sample_count);
            float noise_reduction = (raw_rms - filtered_rms) / raw_rms * 100.0f;
            
            printf("SOGI Monitor - Raw RMS: %.3f, Filtered RMS: %.3f, Noise Reduction: %.1f%%\r\n",
                   raw_rms, filtered_rms, noise_reduction);
        }
        
        // 重置计数器
        raw_sum = 0.0f;
        filtered_sum = 0.0f;
        sample_count = 0;
        last_report_time = current_time;
    }
}
