/*
 * sogi_qsg.h
 *
 *  Created on: 2025年7月21日
 *      Author: Augment Agent
 *  Description: 基于老师高效算法的二阶广义积分器-正交信号发生器（SOGI-QSG）
 *               这是一个无需PI调参的高效锁相模块，直接从ADC输入生成同步的sin/cos信号
 */

#ifndef SOGI_QSG_H
#define SOGI_QSG_H

#include "main.h"
#include "math.h"     // 使用标准数学库

// ============================================================================
// 核心算法参数 (来自老师代码，针对10kHz采样率和50Hz基波频率优化)
// ============================================================================
#define QSG_COEFF_A1        0.9685841f    // 第一个滤波器系数
#define QSG_COEFF_A2        0.969541f     // 第二个滤波器系数
#define QSG_MIN_AMPLITUDE   0.1f          // 最小幅值阈值，避免除零错误
#define QSG_ADC_REF_VOLTAGE 3.3f          // ADC参考电压
#define QSG_ADC_MAX_VALUE   4095.0f       // 12位ADC最大值
#define QSG_DC_OFFSET_V     1.65f         // 直流偏置电压 (3.3V/2)

// ============================================================================
// QSG状态结构体，封装所有内部变量
// ============================================================================
typedef struct {
    // 核心状态变量
    float v_alpha;          // 输入信号 (同相分量)
    float v_beta;           // 正交信号 (滞后90度分量)
    float v_alpha_prev;     // 上一次的输入信号，用于差分计算
    
    // 输出信号
    float sin_theta;        // 归一化后的 sin(θ)，范围 [-1, 1]
    float cos_theta;        // 归一化后的 cos(θ)，范围 [-1, 1]
    float amplitude;        // 信号幅值 (V)
    
    // 状态标志
    uint8_t is_locked;      // 锁定状态标志
    uint32_t stable_count;  // 稳定计数器
} SogiQsg_t;

// ============================================================================
// 函数声明
// ============================================================================

/**
 * @brief 初始化 SOGI-QSG 模块
 * @param qsg 指向QSG状态结构体的指针
 * @note 重置所有状态变量，准备开始锁相
 */
void SogiQsg_Init(SogiQsg_t *qsg);

/**
 * @brief 更新 SOGI-QSG 状态，计算新的 sin/cos 值
 * @param qsg 指向模块状态结构体的指针
 * @param adc_raw_value 当前ADC原始采样值 (0-4095)
 * @note 该函数应在10kHz ADC中断中调用
 *       内部会自动处理ADC到电压的转换和直流偏置去除
 */
void SogiQsg_Update(SogiQsg_t *qsg, uint16_t adc_raw_value);

/**
 * @brief 更新 SOGI-QSG 状态 (浮点电压输入版本)
 * @param qsg 指向模块状态结构体的指针
 * @param input_voltage 已经去除直流偏置的交流电压值 (V)
 * @note 如果您已经在外部完成了ADC到电压的转换，可以使用此函数
 */
void SogiQsg_UpdateVoltage(SogiQsg_t *qsg, float input_voltage);

/**
 * @brief 获取当前的sin值
 * @param qsg 指向QSG状态结构体的指针
 * @return 归一化的sin(θ)值，范围 [-1, 1]
 */
static inline float SogiQsg_GetSin(const SogiQsg_t *qsg)
{
    return qsg->sin_theta;
}


/**
 * @brief 获取当前信号幅值
 * @param qsg 指向QSG状态结构体的指针
 * @return 信号幅值 (V)
 */
static inline float SogiQsg_GetAmplitude(const SogiQsg_t *qsg)
{
    return qsg->amplitude;
}

/**
 * @brief 检查QSG是否已锁定
 * @param qsg 指向QSG状态结构体的指针
 * @return 1: 已锁定, 0: 未锁定
 */
static inline uint8_t SogiQsg_IsLocked(const SogiQsg_t *qsg)
{
    return qsg->is_locked;
}

/**
 * @brief 将cos值转换为PWM占空比 (0-1范围)
 * @param qsg 指向QSG状态结构体的指针
 * @return PWM占空比，范围 [0, 1]
 */
static inline float SogiQsg_GetPwmDuty(const SogiQsg_t *qsg)
{
    return (qsg->cos_theta + 1.0f) / 2.0f;
}

/**
 * @brief 将cos值转换为互补PWM占空比 (0-1范围)
 * @param qsg 指向QSG状态结构体的指针
 * @return 互补PWM占空比，范围 [0, 1]
 */
static inline float SogiQsg_GetPwmDutyInverse(const SogiQsg_t *qsg)
{
    return (1.0f - qsg->cos_theta) / 2.0f;
}

#endif // SOGI_QSG_H
