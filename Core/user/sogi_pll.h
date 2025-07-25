/*
 * sogi_pll.h
 *
 *  Created on: 2025年7月20日
 *      Author: Augment Agent
 *  Description: 单相二阶广义积分器锁相环（SOGI-PLL）头文件
 *               用于电源课题2：单相逆变器设计
 */

#ifndef SOGI_PLL_H
#define SOGI_PLL_H

#include "main.h"
#include "math.h"

// ============================================================================
// SOGI-PLL 系统参数定义
// ============================================================================
#define SAMPLING_FREQ           10000.0f    // 10kHz ADC采样频率
#define SAMPLING_PERIOD         (1.0f / SAMPLING_FREQ)  // 采样周期
#define GRID_FREQ_NOMINAL       50.0f       // 标称电网频率 50Hz
#define OMEGA_NOMINAL           (2.0f * M_PI * GRID_FREQ_NOMINAL)  // 标称角频率

// ============================================================================
// SOGI-PLL 调节参数
// ============================================================================
#define SOGI_K_GAIN             1.41f        // SOGI增益系数 'k' (恢复标准值)
#define PLL_KP                  0.32f       // PLL PI控制器比例增益 (从很小的值开始寻找临界增益)
#define PLL_KI                  0.15f        // PLL PI控制器积分增益 (暂时设为0，先调P后调I)

// ============================================================================
// 信号处理参数
// ============================================================================
#define DC_OFFSET_VOLTAGE       1.65f       // 输入信号直流偏移量 (V)
#define SIGNAL_AMPLITUDE_PEAK   1.0f        // 信号峰值幅度 (V)
#define ANGLE_2PI               (2.0f * M_PI)  // 2π常数

// ============================================================================
// SOGI状态变量结构体
// ============================================================================
typedef struct {
    float v_alpha;      // α轴电压分量
    float v_beta;       // β轴电压分量 (正交分量)
    float z1;           // SOGI内部状态变量1
    float z2;           // SOGI内部状态变量2
} SOGI_State;

// ============================================================================
// PLL状态变量结构体
// ============================================================================
typedef struct {
    float vd;           // d轴电压分量 (Park变换后)
    float vq;           // q轴电压分量 (Park变换后，用于PI控制)
    float omega;        // 估计的角频率 (rad/s)
    float theta;        // 估计的相位角 (rad)
    float sin_theta;    // sin(theta) - 用于SPWM生成
    float cos_theta;    // cos(theta) - 用于Park变换
    
    // PI控制器状态变量
    float pi_integrator;    // PI积分器累积值
    float pi_output;        // PI控制器输出
} PLL_State;

// ============================================================================
// SOGI-PLL 函数声明
// ============================================================================

/**
 * @brief 初始化SOGI-PLL模块
 * @note 重置所有状态变量，omega初始化为标称值
 */
void SOGI_PLL_Init(void);

/**
 * @brief SOGI-PLL主运行函数
 * @param adc_input ADC输入原始值 (包含DC偏移)
 * @param result 输出PLL状态结构体指针
 * @note 该函数应在10kHz中断中调用
 */
void SOGI_PLL_Run(float adc_input, PLL_State* result);

/**
 * @brief 获取当前锁相环状态
 * @return 指向内部PLL状态的指针 (只读)
 */
const PLL_State* SOGI_PLL_GetState(void);

/**
 * @brief 获取估计的频率 (Hz)
 * @return 频率值 (Hz)
 */
float SOGI_PLL_GetFrequency(void);

/**
 * @brief 获取估计的幅值
 * @return 信号幅值 (V)
 */
float SOGI_PLL_GetAmplitude(void);

/**
 * @brief 检查PLL是否已锁定
 * @return 1: 已锁定, 0: 未锁定
 */
uint8_t SOGI_PLL_IsLocked(void);

// ============================================================================
// 调试用外部变量声明
// ============================================================================
extern uint16_t lock_stable_counter;

#endif // SOGI_PLL_H
