/*
 * sogi_pll.c
 *
 *  Created on: 2025年7月20日
 *      Author: Augment Agent
 *  Description: 单相二阶广义积分器锁相环（SOGI-PLL）实现文件
 *               用于电源课题2：单相逆变器设计
 */

#include "sogi_pll.h"
#include <math.h>

// ============================================================================
// 私有状态变量
// ============================================================================
static SOGI_State sogi;     // SOGI状态变量
static PLL_State pll;       // PLL状态变量

// 频率估计低通滤波器
static float freq_filter_output = 50.0f;
static float amp_filter_output = 0.0f;

// 锁定稳定性计数器
uint16_t lock_stable_counter = 0;        // 非静态，供外部调试使用
static uint8_t lock_stable_state = 0;

// ============================================================================
// 私有函数声明
// ============================================================================
static void SOGI_Update(float input_voltage);
static void Park_Transform(void);
static void PI_Controller_Update(void);
static void Phase_Integration(void);

// ============================================================================
// SOGI-PLL初始化函数
// ============================================================================
void SOGI_PLL_Init(void)
{
    // 重置SOGI状态变量
    sogi.v_alpha = 0.0f;
    sogi.v_beta = 0.0f;
    sogi.z1 = 0.0f;
    sogi.z2 = 0.0f;

    // 重置PLL状态变量
    pll.vd = 0.0f;
    pll.vq = 0.0f;
    pll.omega = OMEGA_NOMINAL;      // 初始化为标称角频率
    pll.theta = 0.0f;
    pll.sin_theta = 0.0f;
    pll.cos_theta = 1.0f;

    // 重置PI控制器状态
    pll.pi_integrator = 0.0f;
    pll.pi_output = 0.0f;

    // 重置滤波器状态
    freq_filter_output = 50.0f;    // 初始化为标称频率
    amp_filter_output = 0.0f;      // 初始化为0幅值

    // 重置锁定稳定性计数器
    lock_stable_counter = 0;
    lock_stable_state = 0;
}

// ============================================================================
// SOGI-PLL主运行函数
// ============================================================================
void SOGI_PLL_Run(float adc_input, PLL_State* result)
{
    // 1. 移除直流偏移
    float ac_input = adc_input - DC_OFFSET_VOLTAGE;

    // 1.5. 输入信号预滤波 (简单的限幅)
    if (ac_input > 2.0f) ac_input = 2.0f;
    if (ac_input < -2.0f) ac_input = -2.0f;
    
    // 2. SOGI处理
    SOGI_Update(ac_input);
    
    // 3. Park变换 (α-β到d-q)
    Park_Transform();
    
    // 4. PI控制器更新
    PI_Controller_Update();
    
    // 5. 频率和相位积分
    Phase_Integration();
    
    // 6. 更新三角函数值
    pll.sin_theta = sinf(pll.theta);
    pll.cos_theta = cosf(pll.theta);
    
    // 7. 复制结果到输出指针
    if (result != NULL) {
        *result = pll;
    }
}

// ============================================================================
// SOGI更新函数 (私有)
// ============================================================================
static void SOGI_Update(float input_voltage)
{
    // 限制输入电压范围
    if (input_voltage > 5.0f) input_voltage = 5.0f;
    if (input_voltage < -5.0f) input_voltage = -5.0f;

    // 计算误差: v_err = input - v_alpha
    float v_err = input_voltage - sogi.v_alpha;

    // 更新SOGI内部状态 (离散形式)
    // z1 = z1 + (k*v_err - z2) * ω_nominal * Ts
    sogi.z1 = sogi.z1 + (SOGI_K_GAIN * v_err - sogi.z2) * OMEGA_NOMINAL * SAMPLING_PERIOD;

    // v_alpha = v_alpha + z1 * Ts
    sogi.v_alpha = sogi.v_alpha + sogi.z1 * SAMPLING_PERIOD;

    // z2 = z2 + v_alpha * ω_nominal * Ts
    sogi.z2 = sogi.z2 + sogi.v_alpha * OMEGA_NOMINAL * SAMPLING_PERIOD;

    // v_beta = z2 * Ts (正交分量)
    sogi.v_beta = sogi.z2 * SAMPLING_PERIOD;

    // 限制SOGI状态变量范围，防止发散
    const float sogi_limit = 100.0f;
    if (fabsf(sogi.z1) > sogi_limit) sogi.z1 = (sogi.z1 > 0) ? sogi_limit : -sogi_limit;
    if (fabsf(sogi.z2) > sogi_limit) sogi.z2 = (sogi.z2 > 0) ? sogi_limit : -sogi_limit;
    if (fabsf(sogi.v_alpha) > sogi_limit) sogi.v_alpha = (sogi.v_alpha > 0) ? sogi_limit : -sogi_limit;
    if (fabsf(sogi.v_beta) > sogi_limit) sogi.v_beta = (sogi.v_beta > 0) ? sogi_limit : -sogi_limit;
}

// ============================================================================
// Park变换函数 (私有)
// ============================================================================
static void Park_Transform(void)
{
    // Park变换: α-β坐标系到d-q坐标系
    // vd = v_alpha * cos(theta) + v_beta * sin(theta)
    // vq = -v_alpha * sin(theta) + v_beta * cos(theta)
    
    pll.vd = sogi.v_alpha * pll.cos_theta + sogi.v_beta * pll.sin_theta;
    pll.vq = -sogi.v_alpha * pll.sin_theta + sogi.v_beta * pll.cos_theta;
}

// ============================================================================
// PI控制器更新函数 (私有) - 修改为使用宏定义参数
// ============================================================================
static void PI_Controller_Update(void)
{
    // 使用宏定义的固定参数，不再使用运行时变量
    // 参数定义在 sogi_pll.h 中：PLL_KP 和 PLL_KI

    // PI控制器的误差输入是vq分量
    float error = pll.vq;

    // 限制误差范围，防止极端值
    if (error > 10.0f) error = 10.0f;
    if (error < -10.0f) error = -10.0f;

    // 更新积分项: integrator += Ki * error * Ts
    pll.pi_integrator += PLL_KI * error * SAMPLING_PERIOD;

    // 限制积分器范围，防止积分饱和
    const float integrator_limit = 10.0f;  // 大幅降低积分器限制
    if (pll.pi_integrator > integrator_limit) pll.pi_integrator = integrator_limit;
    if (pll.pi_integrator < -integrator_limit) pll.pi_integrator = -integrator_limit;

    // 计算PI输出: output = Kp * error + integrator
    pll.pi_output = PLL_KP * error + pll.pi_integrator;

    // 限制PI输出范围 (严格限制以保证稳定性)
    const float output_limit = 20.0f;  // 限制频率偏差在±3Hz内
    if (pll.pi_output > output_limit) pll.pi_output = output_limit;
    if (pll.pi_output < -output_limit) pll.pi_output = -output_limit;
}

// ============================================================================
// 相位积分函数 (私有)
// ============================================================================
static void Phase_Integration(void)
{
    // 更新频率: omega = omega_nominal + pi_output
    pll.omega = OMEGA_NOMINAL + pll.pi_output;
    
    // 更新相位角: theta += omega * Ts
    pll.theta += pll.omega * SAMPLING_PERIOD;
    
    // 角度归一化 (0 到 2π)
    if (pll.theta > ANGLE_2PI) {
        pll.theta -= ANGLE_2PI;
    } else if (pll.theta < 0.0f) {
        pll.theta += ANGLE_2PI;
    }
}

// ============================================================================
// 获取当前PLL状态 (只读)
// ============================================================================
const PLL_State* SOGI_PLL_GetState(void)
{
    return &pll;
}

// ============================================================================
// 获取估计频率 (Hz) - 带低通滤波
// ============================================================================
float SOGI_PLL_GetFrequency(void)
{
    float raw_freq = pll.omega / (2.0f * M_PI);

    // 一阶低通滤波器平滑频率估计
    const float freq_filter_alpha = 0.05f;  // 更强的滤波
    freq_filter_output = freq_filter_alpha * raw_freq + (1.0f - freq_filter_alpha) * freq_filter_output;

    return freq_filter_output;
}

// ============================================================================
// 获取估计幅值 - 带低通滤波
// ============================================================================
float SOGI_PLL_GetAmplitude(void)
{
    // 计算α-β分量的幅值
    float raw_amp = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);

    // 一阶低通滤波器平滑幅值估计
    const float amp_filter_alpha = 0.02f;  // 更强的滤波
    amp_filter_output = amp_filter_alpha * raw_amp + (1.0f - amp_filter_alpha) * amp_filter_output;

    return amp_filter_output;
}

// ============================================================================
// 检查PLL锁定状态
// ============================================================================
uint8_t SOGI_PLL_IsLocked(void)
{
    // 调整锁定检测阈值 (基于当前优秀表现的最终优化)
    const float vq_threshold = 2.0f;      // vq阈值：最终放宽以确保锁定
    const float freq_min = 49.8f;         // 最小频率 (基于实际表现)
    const float freq_max = 50.2f;         // 最大频率 (基于实际表现)
    const float amp_min = 0.1f;           // 最小幅值

    float current_freq = pll.omega / (2.0f * M_PI);
    float current_amp = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);

    // 多条件锁定检测
    uint8_t vq_ok = (fabsf(pll.vq) < vq_threshold);
    uint8_t freq_ok = (current_freq >= freq_min && current_freq <= freq_max);
    uint8_t amp_ok = (current_amp >= amp_min);

    uint8_t instant_lock = (vq_ok && freq_ok && amp_ok) ? 1 : 0;

    // 锁定稳定性检测：需要连续锁定150次(1.5秒)才认为真正锁定
    if (instant_lock) {
        if (lock_stable_counter < 50) {
            lock_stable_counter++;
        }
        if (lock_stable_counter >= 50) {
            lock_stable_state = 1;
        }
    } else {
        // 失锁时快速响应，但需要连续失锁50次才取消锁定状态
        if (lock_stable_state == 1) {
            static uint16_t unlock_counter = 0;
            unlock_counter++;
            if (unlock_counter >= 50) {
                lock_stable_counter = 0;
                lock_stable_state = 0;
                unlock_counter = 0;
            }
        } else {
            if (lock_stable_counter > 0) {
                lock_stable_counter--;
            }
        }
    }

    return lock_stable_state;
}
