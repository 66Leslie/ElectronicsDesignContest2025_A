/*
 * sogi_qsg.c
 *
 *  Created on: 2025年7月21日
 *      Author: Augment Agent
 *  Description: 基于老师高效算法的二阶广义积分器-正交信号发生器（SOGI-QSG）实现
 *               核心算法来自老师代码中的高效锁相"黑科技"
 */

#include "sogi_qsg.h"

// ============================================================================
// 私有常量定义
// ============================================================================
#define LOCK_STABLE_THRESHOLD   200    // 锁定稳定性阈值 (连续1000次更新幅值稳定)
#define AMPLITUDE_CHANGE_LIMIT  0.01f   // 幅值变化限制 (1%)

// ============================================================================
// SOGI-QSG初始化函数
// ============================================================================
void SogiQsg_Init(SogiQsg_t *qsg)
{
    // 重置核心状态变量
    qsg->v_alpha = 0.0f;
    qsg->v_beta = 0.0f;
    qsg->v_alpha_prev = 0.0f;
    
    // 重置输出信号
    qsg->sin_theta = 0.0f;
    qsg->cos_theta = 1.0f;  // 初始相位为0度
    qsg->amplitude = 0.0f;
    
    // 重置状态标志
    qsg->is_locked = 0;
    qsg->stable_count = 0;
}

// ============================================================================
// SOGI-QSG更新函数 (ADC原始值输入版本)
// ============================================================================
void SogiQsg_Update(SogiQsg_t *qsg, uint16_t adc_raw_value)
{
    // 1. ADC原始值转换为电压值
    float voltage = ((float)adc_raw_value * QSG_ADC_REF_VOLTAGE) / QSG_ADC_MAX_VALUE;
    
    // 2. 去除直流偏置，得到交流分量
    float ac_voltage = voltage - QSG_DC_OFFSET_V;
    
    // 3. 调用电压输入版本的更新函数
    SogiQsg_UpdateVoltage(qsg, ac_voltage);
}

// ============================================================================
// SOGI-QSG更新函数 (电压输入版本) - 核心算法实现
// ============================================================================
void SogiQsg_UpdateVoltage(SogiQsg_t *qsg, float input_voltage)
{
    float norm_factor;
    float prev_amplitude = qsg->amplitude;
    
    // ============================================================================
    // 核心算法：老师的高效二阶广义积分器
    // 这几行代码是整个锁相系统的核心，无需PI调参即可实现精确锁相
    // ============================================================================
    
    // 1. 更新 alpha 轴 (当前输入信号)
    qsg->v_alpha = input_voltage;
    
    // 2. 更新 beta 轴 (生成90度滞后的正交信号)
    //    这是老师算法的核心：通过数字滤波器生成精确的正交分量
    qsg->v_beta = (qsg->v_beta + qsg->v_alpha_prev - QSG_COEFF_A1 * qsg->v_alpha) * QSG_COEFF_A2;
    
    // 3. 保存当前 alpha 值，供下次计算使用
    qsg->v_alpha_prev = qsg->v_alpha;
    
    // ============================================================================
    // 幅值计算和归一化
    // ============================================================================
    
    // 4. 计算信号幅值 (使用标准数学库的平方根函数)
    qsg->amplitude = sqrtf(qsg->v_alpha * qsg->v_alpha + qsg->v_beta * qsg->v_beta);
    
    // 5. 归一化，得到单位圆上的 sin 和 cos 值
    if (qsg->amplitude > QSG_MIN_AMPLITUDE) {
        norm_factor = 1.0f / qsg->amplitude;
        qsg->cos_theta = qsg->v_alpha * norm_factor;  // cos(θ) = α/|α+jβ|
        qsg->sin_theta = qsg->v_beta * norm_factor;   // sin(θ) = β/|α+jβ|
        // 检查幅值是否稳定 (锁定判据)
        float amplitude_change = fabsf(qsg->amplitude - prev_amplitude);
        if (amplitude_change < AMPLITUDE_CHANGE_LIMIT * qsg->amplitude) {
            qsg->stable_count++;
            if (qsg->stable_count >= LOCK_STABLE_THRESHOLD) {
                qsg->is_locked = 1;
                qsg->stable_count = LOCK_STABLE_THRESHOLD; // 防止溢出
            }
        } else {
            qsg->stable_count = 0;
            qsg->is_locked = 0;
        }
    } else {
        // 信号太小，无法锁定
        qsg->cos_theta = 1.0f;
        qsg->sin_theta = 0.0f;
        qsg->is_locked = 0;
        qsg->stable_count = 0;
    }
}
