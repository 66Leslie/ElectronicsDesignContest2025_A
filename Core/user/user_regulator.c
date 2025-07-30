//
// Created by 86195 on 25-7-14.
//
#include "user_regulator.h"
#include "data_process.h"  // 添加数据处理滤波器头文件
#include "tim.h"          // 添加TIM支持，用于PWM控制
#include "OLED.h"
#include "key.h"
#include "adc.h"
#include "tim.h"
#include "sogi_pll.h"     // 引入SOGI结构体定义


// ============================================================================
// 三相PWM控制变量 (合并单相和三相PWM控制)
// ============================================================================
volatile float modulation_ratio = 0.5f;     // 调制比 (0.0 - 1.0)
volatile uint8_t pwm_enabled = 0;           // PWM使能标志 (统一控制三相PWM)

// 全局启动命令变量 (供按键控制)
uint16_t variable_freq;
// ============================================================================
// 双环控制系统变量
// ============================================================================
// 控制模式和参考值 (简化变量名)
volatile Control_Mode_t ctrl_mode = CONTROL_MODE_MANUAL;  // 当前控制模式
volatile float v_ref = V_REF_DEFAULT;  // 电压参考值 (RMS)
volatile float i_ref = I_REF_DEFAULT;  // 电流参考值 (RMS)

// 双PI控制器实例
PI_Controller_t voltage_pi;                 // 电压外环PI控制器 (慢环, 50Hz) - 保留兼容性
PI_Controller_t current_pi;                 // 电流内环PI控制器 (快环, 20kHz瞬时值控制)
Voltage_PI_Norm_t voltage_pi_norm;          // 归一化电压PI控制器 (新增)

// 控制输出变量 (简化变量名)
volatile float i_ref_peak = 0.0f;          // 电流峰值指令 (由外环输出)
volatile float i_ref_inst = 0.0f;          // 瞬时电流参考值 (20kHz)
volatile float mod_output = 0.0f;          // 最终调制比输出
volatile float i_fdbk_inst = 0.0f;         // 瞬时电流反馈值

// αβ坐标系电流控制器实例
Current_Controller_AlphaBeta_t CurrConReg;
Modulation_t Modulation;
// ============================================================================
// ADC数据缓冲区 (修改为DMA长度=2的结构)
// ============================================================================
extern uint16_t adc_ac_buf[4];              // AC环路缓冲区 (在main.c中定义，DMA长度=2)
extern uint16_t adc3_reference_buf[1];      // ADC3参考信号缓冲区 (在main.c中定义)

// ============================================================================
volatile float reference_frequency = 50.0f;  // 参考信号频率（固定值）
volatile float reference_amplitude = 0.0f;   // 参考信号幅值
volatile Sync_Mode_t sync_mode = SYNC_MODE_FREE; // 同步模式（固定为自由运行）
// ============================================================================
// 参考信号选择相关变量（固定为内部信号）
// ============================================================================
volatile Reference_Signal_t current_reference_signal = REF_SIGNAL_INTERNAL;  // 固定使用内部参考信号

// ============================================================================
// 状态机相关变量
// ============================================================================
volatile System_State_t system_state = STATE_INIT;  // 当前系统状态
volatile uint32_t state_entry_time = 0;                     // 状态进入时间
volatile uint32_t state_transition_timer = 0;               // 状态转换定时器

// ============================================================================
// 高效锁相模块实例定义
// ============================================================================
SogiQsg_t g_sogi_qsg;  // 全局SOGI-QSG实例，基于老师的高效锁相算法
SOGICompositeFilter_t sogi_filter_ia;  // A相电流SOGI复合滤波器实例
SOGICompositeFilter_t sogi_filter_ib;  // B相电流SOGI复合滤波器实例
SOGICompositeFilter_t sogi_filter_vab; // AB线电压SOGI复合滤波器实例
SOGICompositeFilter_t sogi_filter_vbc; // BC线电压SOGI复合滤波器实例

// ============================================================================
// 显示相关变量
// ============================================================================
volatile Display_Page_t current_page = PAGE_MANUAL;  // 当前显示页面
volatile uint32_t page_update_timer = 0;           // 页面更新定时器

// ============================================================================
// AC采样累加器 (用于计算有效值)
// ============================================================================
volatile float current_A_sum = 0.0f;  //A相电流 
volatile float current_B_sum = 0.0f;  //B相电流 
volatile float voltage_AB_sum = 0.0f; //AB线电压
volatile float voltage_BC_sum = 0.0f; //BC线电压
volatile float voltage_AC_sum = 0.0f; //AC线电压
volatile uint16_t ac_sample_count = 0;         // AC采样计数器

// ============================================================================
// 数据处理变量
// ============================================================================
float ac_current_rms_A = 0.0f;             // A相电流RMS值
float ac_current_rms_B = 0.0f;             // B相电流RMS值
float ac_voltage_rms_AB = 0.0f;            // AB线电压RMS值
float ac_voltage_rms_BC = 0.0f;            // BC线电压RMS值
volatile uint8_t dc_data_ready = 0;        // DC数据就绪标志
// ============================================================================
// 用户调节器初始化函数
// ============================================================================
void user_regulator_init(void)
{
    // 添加调试信息，帮助定位错误位置
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);  // 新增：ADC3校准

    // 启动ADC DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_ac_buf, 2);   // DMA长度改为2
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_ac_buf[2], 2);   //
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_reference_buf, 1);  //

    //HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);       // 启动DAC2通道1 (PA6)
    //HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "Init", OLED_8X16);
    OLED_Update();
    variable_freq = 50.0f;  // 初始化为50Hz

    key_init();

    // 初始化三相PWM控制变量 (合并后统一使用)
    modulation_ratio = 0.0f;  // 从较小的值开始，安全起见
    pwm_enabled = 0;          // 初始状态PWM关闭

    // 初始化AC采样累加器
    current_A_sum = 0.0f;
    current_B_sum = 0.0f;
    voltage_AB_sum = 0.0f;
    voltage_BC_sum = 0.0f;
    ac_sample_count = 0;
    // 初始化双环控制系统（包括PI控制器）
    Dual_Loop_Control_Init();
    // 初始化状态机,进入等待状态
    // 初始化SOGI复合滤波器
    SOGICompositeFilter_Init(&sogi_filter_ia, 
                            SOGI_TARGET_FREQ, SOGI_SAMPLING_FREQ, 
                            SOGI_DAMPING_FACTOR, 0.026f, 0.0f);
    SOGICompositeFilter_Init(&sogi_filter_ib, 
                            SOGI_TARGET_FREQ, SOGI_SAMPLING_FREQ, 
                            SOGI_DAMPING_FACTOR, 0.026f, 0.0f);
    SOGICompositeFilter_Init(&sogi_filter_vab, 
                            SOGI_TARGET_FREQ, SOGI_SAMPLING_FREQ, 
                            SOGI_DAMPING_FACTOR, 0.026f, 0.0f);
    SOGICompositeFilter_Init(&sogi_filter_vbc, 
                            SOGI_TARGET_FREQ, SOGI_SAMPLING_FREQ, 
                            SOGI_DAMPING_FACTOR, 0.026f, 0.0f);
    // 初始化参考信号选择
    current_reference_signal = REF_SIGNAL_INTERNAL;  // 默认使用内部参考信号
}

// ============================================================================
// 用户调节器主循环函数
// ============================================================================
void user_regulator_main(void)
{
    // 1. 处理按键输入
    key_proc();
    // 2. 更新显示
    Update_Disp();
}
// ============================================================================
// ADC转换完成回调函数接口
// ============================================================================
void user_regulator_adc_callback(const ADC_HandleTypeDef* hadc)
{
    // 定义静态变量来跟踪所有ADC的完成状态和50Hz慢速环的计数
    static uint8_t adc_completion_mask = 0;
    static uint16_t slow_loop_counter = 0;

    // --- Part 1: 数据采集与标志位更新 (每次ADC/DMA完成都会进入) ---
    if(hadc->Instance == ADC1) { adc_completion_mask |= (1 << 0); }
    if(hadc->Instance == ADC2) { adc_completion_mask |= (1 << 1); }
    if(hadc->Instance == ADC3) {//全部使用内部信号
            // 内部信号模式：直接数学计算
            static uint32_t internal_counter = 0;

            // 确保频率在合理范围内，避免除零错误
            variable_freq  = _fsat(variable_freq, 100.0f, 1.0f);

            uint32_t period_samples = (uint32_t)(20000.0f / variable_freq);
            internal_counter = (internal_counter + 1) % period_samples;
            float current_angle = 2.0f * M_PI * internal_counter / (float)period_samples;
            g_sogi_qsg.sin_theta = sinf(current_angle);
            g_sogi_qsg.cos_theta = cosf(current_angle);
            g_sogi_qsg.is_locked = 1;  // 内部信号始终锁定
            adc_completion_mask |= (1 << 2);
    }

    // --- Part 2: 门禁检查，确保所有ADC都完成后才执行主逻辑 (10kHz/20kHz) ---
    if (adc_completion_mask != 0b00000111) {
        return; // 等待所有ADC完成
    }
    adc_completion_mask = 0; // 重置掩码，为下个周期做准备

    // --- Part 2.1: ADC数据处理 - 分别计算用于控制和RMS的值 ---

    // 基础转换值（偏置补偿 + ADC转换增益）
    float current_A_base = ((int16_t)adc_ac_buf[0] - IacOffset_A) * MeasureGain;
    float voltage_AB_base = ((int16_t)adc_ac_buf[1] - VacOffset_AB) * MeasureGain;
    float current_B_base = ((int16_t)adc_ac_buf[2] - IacOffset_B) * MeasureGain;
    float voltage_BC_base = ((int16_t)adc_ac_buf[3] - VacOffset_BC) * MeasureGain;

    // // 限幅滤波器：限制变化幅度，平滑信号
    // static float current_A_last = 0.0f;
    // static float current_B_last = 0.0f;
    // static float voltage_AB_last = 0.0f;
    // static float voltage_BC_last = 0.0f;
    // static uint8_t filter_initialized = 0;

    // // 首次运行时初始化滤波器
    // if (!filter_initialized) {
    //     current_A_last = IacOffset_A * MeasureGain;
    //     current_B_last = IacOffset_B * MeasureGain;
    //     voltage_AB_last = VacOffset_AB * MeasureGain;
    //     voltage_BC_last = VacOffset_BC * MeasureGain;
    //     filter_initialized = 1;
    // }

    // // 应用限幅滤波器 - 使用data_process.c中的Limit_Filter函数
    // const float MAX_CHANGE = 0.026f;//sin(0.45)/3.3=0.02591
    // float current_A_filtered = Limit_Filter(current_A_base, &current_A_last, MAX_CHANGE);
    // float current_B_filtered = Limit_Filter(current_B_base, &current_B_last, MAX_CHANGE);
    // float voltage_AB_filtered = Limit_Filter(voltage_AB_base, &voltage_AB_last, MAX_CHANGE);
    // float voltage_BC_filtered = Limit_Filter(voltage_BC_base, &voltage_BC_last, MAX_CHANGE);
    //使用SOGI复合滤波器
    float current_A_filtered = SOGICompositeFilter_Update(&sogi_filter_ia, current_A_base);
    float current_B_filtered = SOGICompositeFilter_Update(&sogi_filter_ib, current_B_base);
    float voltage_AB_filtered = SOGICompositeFilter_Update(&sogi_filter_vab, voltage_AB_base);
    float voltage_BC_filtered = SOGICompositeFilter_Update(&sogi_filter_vbc, voltage_BC_base);

    // 累加基础值的平方（用于RMS计算）- 添加数值保护
    // 限制基础值范围，避免平方后溢出
    float current_A_limited = _fsat(current_A_filtered, 10.0f, -10.0f);
    float voltage_AB_limited = _fsat(voltage_AB_filtered, 10.0f, -10.0f);
    float current_B_limited = _fsat(current_B_filtered, 10.0f, -10.0f);
    float voltage_BC_limited = _fsat(voltage_BC_filtered, 10.0f, -10.0f);

    current_A_sum += current_A_limited * current_A_limited;
    voltage_AB_sum += voltage_AB_limited * voltage_AB_limited;
    current_B_sum += current_B_limited * current_B_limited;
    voltage_BC_sum += voltage_BC_limited * voltage_BC_limited;
    slow_loop_counter++;

    // --- Part 2.2: 50Hz慢速环 (每400次执行一次) ---
    if (slow_loop_counter >= AC_SAMPLE_SIZE)
    {
        slow_loop_counter = 0; // 计数器清零

        // 1. 计算RMS值，应用您标定的传递系数 - 添加数值保护
        // 确保累加器值为正数，避免负数开方
        float current_A_avg = fmaxf(current_A_sum / (float)AC_SAMPLE_SIZE, 0.0f);
        float voltage_AB_avg = fmaxf(voltage_AB_sum / (float)AC_SAMPLE_SIZE, 0.0f);
        float current_B_avg = fmaxf(current_B_sum / (float)AC_SAMPLE_SIZE, 0.0f);
        float voltage_BC_avg = fmaxf(voltage_BC_sum / (float)AC_SAMPLE_SIZE, 0.0f);

        ac_current_rms_A = sqrtf(current_A_avg) * 5.1778f - 0.0111f ;
        ac_voltage_rms_AB = sqrtf(voltage_AB_avg) * 68.011f - 0.1784f;//手动矫正0.1
        ac_current_rms_B = sqrtf(current_B_avg) * 5.1778f - 0.0111f;
        ac_voltage_rms_BC = sqrtf(voltage_BC_avg) * 68.011f - 0.1784f;
        // 真实值 = m * 显示值 + c 低参考值 高参考值 (V_oled1, V_real1) 和 (V_oled2, V_real2)
        //ac_voltage_rms_AB 
        // 应用你通过两点校准计算出的新系数 m 和 c
        const float cal_m = 0.993f; // 示例值：新的斜率
        const float cal_c = 0.185f; // 示例值：新的截距
        ac_voltage_rms_AB = ac_voltage_rms_AB * cal_m + cal_c;
        // 最终结果限制，避免异常值
        ac_current_rms_A = _fsat(ac_current_rms_A, 100.0f, 0.0f);
        ac_voltage_rms_AB = _fsat(ac_voltage_rms_AB, 1000.0f, 0.0f);
        ac_current_rms_B = _fsat(ac_current_rms_B, 100.0f, 0.0f);
        ac_voltage_rms_BC = _fsat(ac_voltage_rms_BC, 1000.0f, 0.0f);

        
        // 2. 清空累加器
        current_A_sum = 0.0f;
        current_B_sum = 0.0f;
        voltage_AB_sum = 0.0f;
        voltage_BC_sum = 0.0f;
        voltage_AC_sum = 0.0f;

        // 3. 执行外环控制器 (电压环或电流环参考值更新)
        if (pwm_enabled) {
            switch (ctrl_mode) {
                case CONTROL_MODE_VOLTAGE:
                    // 使用归一化电压PI控制器，自动适应不同直流母线电压
                    // TODO: 实际应用中应通过ADC测量直流母线电压
                    // 示例: float v_dc_measured = ADC_Read_DC_Bus_Voltage();
                    float v_dc_actual = V_DC_NOMINAL;  // 临时使用标称值，实际应测量

                    mod_output = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref,
                                                       ac_voltage_rms_AB, v_dc_actual);
                    mod_output = _fsat(mod_output, PI_V_OUT_MAX, PI_V_OUT_MIN);
                    break;
                case CONTROL_MODE_CURRENT:
                    // 更新调制比输出用于显示 (取三相调制信号的平均幅值)
                    mod_output = Modulation.Ma;
                    break;
                case CONTROL_MODE_MANUAL:
                default:
                    break;
            }

        }
    }

    // --- Part 2.3: 20kHz快速环 - 核心控制与PWM更新 (每次都执行) ---
    if (!pwm_enabled) {
        // PWM关闭时，清零占空比并返回
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
        return;
    }

    // 瞬时值传递系数应用（使用您标定的传递系数）
    float current_A_calibrated = current_A_filtered * 5.1778f - 0.0111f;
    float current_B_calibrated = current_B_filtered * 5.1778f - 0.0111f;
    // 根据控制模式计算三相PWM占空比
    float duty_A_float = 0.0f, duty_B_float = 0.0f, duty_C_float = 0.0f;
    switch (ctrl_mode) {
        case CONTROL_MODE_CURRENT:
            i_ref_peak = i_ref * 1.414213562f;
            Current_Controller_AlphaBeta_Update(i_ref_peak, current_A_calibrated, current_B_calibrated);
            
            // 2. 限制三相调制信号
            float mod_A = _fsat(Modulation.Ma, 1.0f, -1.0f);
            float mod_B = _fsat(Modulation.Mb, 1.0f, -1.0f);
            float mod_C = _fsat(Modulation.Mc, 1.0f, -1.0f);

            // 3. 计算三相PWM占空比 (调制信号转换为占空比)
            // 修正：αβ控制器输出的调制信号范围是[-1,1]，需要转换为[0,1]再乘以PWM周期
            duty_A_float = ((mod_A + 1.0f) * 0.5f) * PWM_PERIOD_TIM8;
            duty_B_float = ((mod_B + 1.0f) * 0.5f) * PWM_PERIOD_TIM8;
            duty_C_float = ((mod_C + 1.0f) * 0.5f) * PWM_PERIOD_TIM8;
            break;

        case CONTROL_MODE_VOLTAGE:
        case CONTROL_MODE_MANUAL:
        default:
            {
                float final_mod_ratio = (ctrl_mode == CONTROL_MODE_VOLTAGE) ? mod_output : modulation_ratio;

                // 关键点1: 放开调制比上限到 1.1547f，即 2/sqrt(3)，实现SVPWM电压利用率
                // SVPWM原理：通过注入三次谐波等共模分量，提高直流母线电压利用率约15.5%
                // 从标准SPWM的0.866*Vdc提升到1.0*Vdc，线电压RMS可达Vdc/√2
                final_mod_ratio = _fsat(final_mod_ratio, 1.1547f, 0.0f);

                // 获取相位信息
                float cos_theta = g_sogi_qsg.cos_theta;
                float sin_theta = g_sogi_qsg.sin_theta;

                // 步骤1: 生成原始的三相调制信号 (范围: [-final_mod_ratio, +final_mod_ratio])
                float u_a = final_mod_ratio * (cos_theta);                                       // A相: m*cos(θ)
                float u_b = final_mod_ratio * (cos_theta * (-0.5f) + sin_theta * (0.866025f));  // B相: m*cos(θ-120°)
                float u_c = final_mod_ratio * (cos_theta * (-0.5f) - sin_theta * (0.866025f));  // C相: m*cos(θ+120°)

                // 步骤2: 找出三相调制信号的瞬时最大值和最小值 (Min-Max法)
                float u_max = fmaxf(u_a, fmaxf(u_b, u_c));
                float u_min = fminf(u_a, fminf(u_b, u_c));

                // 步骤3: 计算需要注入的共模电压偏置 (三次谐波注入的等效实现)
                float u_offset = -0.5f * (u_max + u_min);

                // 步骤4: 将偏置注入三相，得到新的调制信号 (范围被钳位在 [-1, 1] 区间内)
                float mod_A = u_a + u_offset;
                float mod_B = u_b + u_offset;
                float mod_C = u_c + u_offset;

                // 步骤5: 基于新的调制信号计算PWM占空比
                // (mod_A/B/C + 1.0f) * 0.5f 将 [-1, 1] 的调制信号映射到 [0, 1] 的占空比系数
                duty_A_float = (mod_A + 1.0f) * 0.5f * PWM_PERIOD_TIM8;
                duty_B_float = (mod_B + 1.0f) * 0.5f * PWM_PERIOD_TIM8;
                duty_C_float = (mod_C + 1.0f) * 0.5f * PWM_PERIOD_TIM8;
            }
            break;
    }

    // 统一进行边界检查和PWM设置
    uint32_t duty_cycle_A = (uint32_t)_fsat(duty_A_float, PWM_PERIOD_TIM8, 0.0f);
    uint32_t duty_cycle_B = (uint32_t)_fsat(duty_B_float, PWM_PERIOD_TIM8, 0.0f);
    uint32_t duty_cycle_C = (uint32_t)_fsat(duty_C_float, PWM_PERIOD_TIM8, 0.0f);

    // 设置三相PWM占空比
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty_cycle_A);  // A相
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty_cycle_B);  // B相
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty_cycle_C);  // C相
}
// ============================================================================
// 增量式PI控制器更新函数
// ============================================================================
float PI_Controller_Update_Incremental(PI_Controller_t* pi, float reference, float feedback)
{
    // 1. 计算当前误差
    float error = reference - feedback;
    // 2. 计算输出的"增量"
    // 增量式PI控制器公式: ΔOutput = Kp*(error_k - error_{k-1}) + Ki*error_k
    float p_term = pi->kp * (error - pi->last_error);
    float i_term = pi->ki * error;
    float delta_output = p_term + i_term;
    // 3. 在上一次输出的基础上，累加这个增量
    float old_output = pi->output;
    float new_output = pi->output + delta_output;
    // 添加输出变化率限制，防止突变
    const float max_change = 0.02f;  // 每次最大变化2%
    float output_change = new_output - old_output;
    if (output_change > max_change) {
        new_output = old_output + max_change;
    } else if (output_change < -max_change) {
        new_output = old_output - max_change;
    }
    // 4. 对累加后的最终输出进行限幅，并实现抗积分饱和
    if (new_output > pi->output_max) {
        pi->output = pi->output_max;
        if (i_term > 0) {
            pi->last_error = error;
            return pi->output;
        }
    } else if (new_output < pi->output_min) {
        pi->output = pi->output_min;
        if (i_term < 0) {
            pi->last_error = error;
            return pi->output;
        }
    } else {
        pi->output = new_output;
    }
    // 5. 更新历史误差，为下一次计算做准备
    pi->last_error = error;
    // PID调试输出：适配串口绘图仪格式
#if (PID_DEBUG_PRINTF == 1)
    // 格式: >名称1:数值1,名称2:数值2,...\r\n
    printf(">Set:%.3f,Fdbk:%.3f,Err:%.3f,P:%.3f,I:%.3f,Out:%.3f\r\n",
           reference,
           feedback,
           error,
           p_term,
           i_term,
           pi->output);
#endif
    return pi->output;
}
// ============================================================================
// αβ坐标系电流控制器更新函数 (基于您提供的算法)
// ============================================================================
void Current_Controller_AlphaBeta_Update(float Ia_CMD, float current_A, float current_B)
{
    // 计算第三相电流 (基于基尔霍夫定律: Ia + Ib + Ic = 0)
    float current_C = -(current_A + current_B);

    // 步骤1: Clarke变换 - 将三相电流反馈转换到αβ坐标系
    // α = (2/3)*Ia - (1/3)*Ib - (1/3)*Ic
    float F32alpha = current_A * 0.6666666667f - current_B * 0.3333333334f - current_C * 0.3333333334f;
    // β = (1/√3)*(Ib - Ic)
    float F32beta = (current_B - current_C) * 0.57735026918963f;

    // 步骤2: 生成三相电流指令 (基于锁相环的sin/cos值)
    // A相: Ia_CMD * cos(θ)
    CurrConReg.Ia_CMD = Ia_CMD * g_sogi_qsg.cos_theta;
    // B相: Ia_CMD * cos(θ - 120°) = Ia_CMD * (cosθ * (-0.5) + sinθ * (sqrt(3)/2))
    CurrConReg.Ib_CMD = Ia_CMD * (g_sogi_qsg.cos_theta * (-0.5f) + g_sogi_qsg.sin_theta * (0.8660254f));
    // C相: Ia_CMD * cos(θ + 120°) = Ia_CMD * (cosθ * (-0.5) - sinθ * (sqrt(3)/2))
    CurrConReg.Ic_CMD = Ia_CMD * (g_sogi_qsg.cos_theta * (-0.5f) - g_sogi_qsg.sin_theta * (0.8660254f));
    
    // 步骤3: Clarke变换 - 将三相电流指令转换到αβ坐标系
    // Valpha_CMD = (2/3)*Ia_CMD - (1/3)*Ib_CMD - (1/3)*Ic_CMD
    CurrConReg.Valpha_CMD = CurrConReg.Ia_CMD * 0.6666666667f - (CurrConReg.Ib_CMD + CurrConReg.Ic_CMD) * 0.3333333334f;
    // Vbeta_CMD = (1/√3)*(Ib_CMD - Ic_CMD)
    CurrConReg.Vbeta_CMD  = (CurrConReg.Ib_CMD - CurrConReg.Ic_CMD) * 0.57735026918963f;


    // 步骤4: 计算α轴误差并更新PI控制器
    CurrConReg.Error_alpha = CurrConReg.Valpha_CMD - F32alpha;
    float delta_alpha = (PI_KP_CURRENT_ALPHA * (CurrConReg.Error_alpha - CurrConReg.Error_alpha_Pre)) + (PI_KI_CURRENT_ALPHA * CurrConReg.Error_alpha);
    delta_alpha = _fsat(delta_alpha,0.02,-0.02);//限制最大变化量
    CurrConReg.PI_Out_alpha += delta_alpha;

    // α轴输出限幅
    CurrConReg.PI_Out_alpha = _fsat(CurrConReg.PI_Out_alpha, PI_I_OUT_MAX, PI_I_OUT_MIN);

    // 步骤5: 计算β轴误差并更新PI控制器
    CurrConReg.Error_beta = CurrConReg.Vbeta_CMD - F32beta;
    float delta_beta = (PI_KP_CURRENT_BETA * (CurrConReg.Error_beta - CurrConReg.Error_beta_Pre)) + (PI_KI_CURRENT_BETA * CurrConReg.Error_beta);
    delta_beta = _fsat(delta_beta,0.02,-0.02);//限制最大变化量
    CurrConReg.PI_Out_Beta += delta_beta;

    // β轴输出限幅
    CurrConReg.PI_Out_Beta = _fsat(CurrConReg.PI_Out_Beta, PI_I_OUT_MAX, PI_I_OUT_MIN);
	
	// 更新历史误差，为下一次计算做准备
    CurrConReg.Error_alpha_Pre = CurrConReg.Error_alpha;
    CurrConReg.Error_beta_Pre = CurrConReg.Error_beta;

    // 步骤6: 前馈补偿 (当前设为0)
    CurrConReg.feedforward_a = 0.0f;
    CurrConReg.feedforward_b = 0.0f;
    CurrConReg.feedforward_c = 0.0f;

    // 步骤7: 反Clarke变换 - 将αβ坐标系输出转换回三相调制信号
    // Ma = V_alpha
    Modulation.Ma = CurrConReg.PI_Out_alpha + CurrConReg.feedforward_a;
    // Mb = -0.5 * V_alpha + (√3/2) * V_beta
    Modulation.Mb = -0.5f * CurrConReg.PI_Out_alpha + 0.8660254038f * CurrConReg.PI_Out_Beta + CurrConReg.feedforward_b;
    // Mc = -0.5 * V_alpha - (√3/2) * V_beta
    Modulation.Mc = -0.5f * CurrConReg.PI_Out_alpha - 0.8660254038f * CurrConReg.PI_Out_Beta + CurrConReg.feedforward_c;
    
    // 调试输出 (可选)
#if (ALPHABETA_DEBUG_PRINTF == 1)
    static uint32_t debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 1000) {  // 每1000次输出一次，避免过于频繁
        debug_counter = 0;
        printf(">Ia_ref:%.3f,Ialpha:%.3f,Ibeta:%.3f,Valpha:%.3f,Vbeta:%.3f,Ma:%.3f,Mb:%.3f,Mc:%.3f\r\n",
               Ia_CMD, F32alpha, F32beta,
               CurrConReg.Valpha_CMD, CurrConReg.Vbeta_CMD,
               Modulation.Ma, Modulation.Mb, Modulation.Mc);
    }
#endif
}
// ============================================================================
// 按键处理逻辑函数
// ============================================================================
void key_proc(void)
{
    const key_result_t key_result = key_scan();
    if (key_result.key_state == KEY_PRESS){
        switch (key_result.key_num) {
            case KEY1:  // 参数增加 (+)
                switch (current_page) {
                    case PAGE_MANUAL:
                        // 手动模式：增加调制比 (支持SVPWM范围到1.15)
                        if (modulation_ratio < 1.15f) {
                            modulation_ratio += 0.05f;
                            if (modulation_ratio > 1.15f) modulation_ratio = 1.15f;
                        }
                        break;
                    case PAGE_CV:
                        // CV模式：增加电压参考值
                        Set_Voltage_Reference(v_ref + 1.0f);
                        break;
                    case PAGE_CC:
                        // CC模式：增加电流参考值
                        Set_Current_Reference(i_ref + 0.1f);
                        break;
                    case PAGE_FREQ:
                        // 频率调节页面：增加频率
                        if (variable_freq < 100.0f) {
                            variable_freq += 1.0f;
                            if (variable_freq > 100.0f) variable_freq = 100.0f;
                        }
                        break;
                    default:
                        break;
                }
                break;

            case KEY2:  // 参数减少 (-)
                switch (current_page) {
                    case PAGE_MANUAL:
                        // 手动模式：减少调制比
                        if (modulation_ratio > 0.0f) {
                            modulation_ratio -= 0.05f;
                            if (modulation_ratio < 0.0f) modulation_ratio = 0.0f;
                        }
                        break;
                    case PAGE_CV:
                        // CV模式：减少电压参考值
                        Set_Voltage_Reference(v_ref - 1.0f);
                        break;
                    case PAGE_CC:
                        // CC模式：减少电流参考值
                        Set_Current_Reference(i_ref - 0.1f);
                        break;
                    case PAGE_FREQ:
                        // 频率调节页面：减少频率
                        if (variable_freq > 10.0f) {
                            variable_freq -= 1.0f;
                            if (variable_freq < 10.0f) variable_freq = 10.0f;
                        }
                        break;
                    default:
                        break;
                }
                break;

            case KEY3:  // PWM开启/关闭
                // 切换PWM使能状态
                pwm_enabled = !pwm_enabled;

                if (pwm_enabled) {
                    // 启用PWM输出
                    PWM_Enable();
                } else {
                    // 停止PWM输出
                    PWM_Disable();
                }

                user_regulator_info("PWM %s", pwm_enabled ? "ENABLED" : "DISABLED");
                break;

            case KEY5:  // 页面切换
                // 循环切换页面：Manual -> CV -> CC -> Freq -> Manual
                switch (current_page) {
                    case PAGE_MANUAL:
                        current_page = PAGE_CV;
                        Set_Control_Mode(CONTROL_MODE_VOLTAGE);
                        break;
                    case PAGE_CV:
                        current_page = PAGE_CC;
                        Set_Control_Mode(CONTROL_MODE_CURRENT);
                        break;
                    case PAGE_CC:
                        current_page = PAGE_FREQ;
                        Set_Control_Mode(CONTROL_MODE_MANUAL);  // 频率调节页面使用开环模式
                        break;
                    case PAGE_FREQ:
                        current_page = PAGE_MANUAL;
                        Set_Control_Mode(CONTROL_MODE_MANUAL);
                        break;
                    default:
                        current_page = PAGE_MANUAL;
                        Set_Control_Mode(CONTROL_MODE_MANUAL);
                        break;
                }
                user_regulator_info("Page: %d, Mode: %s", current_page, Get_Control_Mode_Name(ctrl_mode));
                break;

            default:
                break;
        }
    }
}
// ============================================================================
// 更新OLED显示 - 多页面支持128*64
// ============================================================================
void Update_Disp(void)
{
    static uint32_t last_update = 0;
    const uint32_t current_time = HAL_GetTick();

    // 每250ms更新一次显示
    if (current_time - last_update >= 250) {
        last_update = current_time;

        OLED_Clear();

        // 根据当前页面显示对应界面
        switch (current_page) {
            case PAGE_MANUAL:
                Display_Manual_Mode_Page();
                break;
            case PAGE_CV:
                Display_CV_Mode_Page();
                break;
            case PAGE_CC:
                Display_CC_Mode_Page();
                break;
            case PAGE_FREQ:
                Display_Freq_Mode_Page();
                break;
            default:
                // 异常情况，强制回到手动模式
                current_page = PAGE_MANUAL;
                Set_Control_Mode(CONTROL_MODE_MANUAL);
                Display_Manual_Mode_Page();
                break;
        }

        OLED_Update();
    }
}
/**
 * @brief 频率调节页面显示 - 开环频率调节模式
 */
void Display_Freq_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 频率调节页面显示
    OLED_Println(OLED_6X8, "FREQ PWM:%s", pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "Freq:%3dHz", variable_freq);
    OLED_Println(OLED_6X8, "Mod:%.1f%% (Open Loop)", modulation_ratio * 100.0f);
    OLED_Println(OLED_6X8, "V_AB:%.2fV V_BC:%.2fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "I_A:%.2fA I_B:%.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "K1:+ K2:- K3:PWM K5:Page");
}
/**
 * @brief 手动模式显示界面 - 开环调制比控制
 */
void Display_Manual_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 手动模式显示
    OLED_Println(OLED_6X8, "MANUAL PWM:%s", pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "Mod:%.1f%% (Manual)", modulation_ratio * 100.0f);
    OLED_Println(OLED_6X8, "V_AB:%.2fV V_BC:%.2fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "I_A:%.2fA I_B:%.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "Freq:%.1fHz", variable_freq);
    OLED_Println(OLED_6X8, "1/2:+- 3:PWM 5:Page");
}
/**
 * @brief 恒压模式(CV)显示界面 - 三相逆变器
 */
void Display_CV_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 统一使用小字体，可以显示更多信息
    OLED_Println(OLED_6X8, "CV PWM:%s", pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "Set:%.1fV Act:%.2fV", v_ref, ac_voltage_rms_AB);
    OLED_Println(OLED_6X8, "Mod:%.1f%% (Auto)", mod_output * 100.0f);
    OLED_Println(OLED_6X8, "I_A:%.2fA I_B:%.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "AB:%.2fV BC:%.2fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "Freq:%.1fHz (Fixed)", variable_freq);
    OLED_Println(OLED_6X8, "1/2:+- 3:PWM 5:Page");
}
/**
 * @brief 恒流模式(CC)显示界面 - 三相逆变器
 */
void Display_CC_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 统一使用小字体，可以显示更多信息
    OLED_Println(OLED_6X8, "CC Mode PWM:%s", pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "Set:%.2fA Act:%.2fA", i_ref, ac_current_rms_A);
    OLED_Println(OLED_6X8, "Mod:%.1f%% (Auto)", mod_output * 100.0f);
    OLED_Println(OLED_6X8, "A:%.2fA B:%.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "AB:%.1fV BC:%.1fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "Freq: %.1fHz (Fixed)", variable_freq);
    OLED_Println(OLED_6X8, "K1:+ K2:- K3:PWM K5:Page");
}
// ============================================================================
// αβ坐标系电流控制器复位函数
// ============================================================================
void Current_Controller_AlphaBeta_Reset(void)
{
    Current_Controller_AlphaBeta_Init();
    user_regulator_info("Alpha-Beta Current Controller Reset");
}
// ============================================================================
// 饱和函数实现
// ============================================================================
float _fsat(float value, float max_val, float min_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}
// ============================================================================
// αβ坐标系电流控制器初始化函数
// ============================================================================
void Current_Controller_AlphaBeta_Init(void)
{
    // 清零所有控制器状态
    CurrConReg.Ia_CMD = 0.0f;
    CurrConReg.Ib_CMD = 0.0f;
    CurrConReg.Ic_CMD = 0.0f;

    CurrConReg.Valpha_CMD = 0.0f;
    CurrConReg.Vbeta_CMD = 0.0f;

    CurrConReg.Error_alpha = 0.0f;
    CurrConReg.Error_alpha_Pre = 0.0f;
    CurrConReg.Error_beta = 0.0f;
    CurrConReg.Error_beta_Pre = 0.0f;

    CurrConReg.PI_Out_alpha = 0.0f;
    CurrConReg.PI_Out_Beta = 0.0f;

    CurrConReg.feedforward_a = 0.0f;
    CurrConReg.feedforward_b = 0.0f;
    CurrConReg.feedforward_c = 0.0f;

    // 初始化调制信号
    Modulation.Ma = 0.0f;
    Modulation.Mb = 0.0f;
    Modulation.Mc = 0.0f;

    user_regulator_info("Alpha-Beta Current Controller Initialized");
}
/**
 * @brief 停止系统
 */
void USER_Regulator_Stop(void)
{
    // 禁用PWM
    PWM_Disable();
    // 复位控制器
    Dual_Loop_Control_Reset();
}
/**
 * @brief 使能PWM (TIM8三相PWM输出)
 */
void PWM_Enable(void)
{
    // 使能PWM输出 (GPIO控制) - 高电平使能
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

    // 启动TIM8的PWM输出 (不启动中断，因为逻辑已迁移至ADC回调)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // A相
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // B相
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  // C相

    // 设置使能标志
    pwm_enabled = 1;
}
/**
 * @brief 禁用PWM (TIM8三相PWM输出)
 */
void PWM_Disable(void)
{
    // 禁用PWM输出 (GPIO控制) - 低电平关断
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

    // 停止TIM8的PWM输出
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);   // A相
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);   // B相
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);   // C相

    // 清除使能标志
    pwm_enabled = 0;

    // 确保输出为0
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
}
// ============================================================================
// PI控制器初始化函数
// ============================================================================
void PI_Controller_Init(PI_Controller_t* pi, float kp, float ki, float output_min, float output_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->output = 0.0f;          // 初始化输出值为0
    pi->output_max = output_max;
    pi->output_min = output_min;
    pi->last_error = 0.0f;      // 初始化上次误差为0
}
void Dual_Loop_Control_Init(void)
{
    // 初始化归一化电压外环PI控制器 (慢环, 50Hz)
    Voltage_PI_Norm_Init(&voltage_pi_norm, PI_KP_V_NORM, PI_KI_V_NORM,
                        PI_V_OUT_MIN, PI_V_OUT_MAX, V_DC_NOMINAL);

    // 初始化传统电压PI控制器 (保留兼容性)
    PI_Controller_Init(&voltage_pi, 0.03f, 0.012f, PI_V_OUT_MIN, PI_V_OUT_MAX);

    // 初始化电流内环PI控制器 (快环, 20kHz瞬时值控制)
    // 注意：现在使用αβ坐标系控制器，这里保留传统PI控制器以备用
    PI_Controller_Init(&current_pi, PI_KP_CURRENT_ALPHA, PI_KI_CURRENT_ALPHA, PI_I_OUT_MIN, PI_I_OUT_MAX);

    // 初始化αβ坐标系电流控制器
    Current_Controller_AlphaBeta_Init();

    // 设置默认控制模式
    ctrl_mode = CONTROL_MODE_MANUAL;

    // 初始化参考值
    v_ref = V_REF_DEFAULT;
    i_ref = I_REF_DEFAULT;
    i_ref_peak = 0.0f;

    // 初始化输出
    mod_output = 0.0f;
    i_fdbk_inst = 0.0f;
    i_ref_inst = 0.0f;

    user_regulator_info("Dual Loop Control System Initialized");
    user_regulator_debug("Mode: %s, V_ref: %.1fV, I_ref: %.2fA",
                        Get_Control_Mode_Name(ctrl_mode), v_ref, i_ref);
}
void PI_Controller_Reset(PI_Controller_t* pi)
{
    pi->output = 0.0f;    // 重置输出为0
    pi->last_error = 0.0f;
}
void Dual_Loop_Control_Reset(void)
{
    // 复位PI控制器
    PI_Controller_Reset(&voltage_pi);
    PI_Controller_Reset(&current_pi);

    // 复位归一化电压PI控制器
    Voltage_PI_Norm_Reset(&voltage_pi_norm);

    // 复位输出
    i_ref_peak = 0.0f;
    mod_output = 0.0f;
    i_fdbk_inst = 0.0f;
    i_ref_inst = 0.0f;

    // 复位αβ坐标系电流控制器
    Current_Controller_AlphaBeta_Reset();

    user_regulator_info("Dual Loop Control System Reset");
}


void Set_Control_Mode(Control_Mode_t mode)
{
    if (mode >= CONTROL_MODE_COUNT) return;

    Control_Mode_t old_mode = ctrl_mode;
    ctrl_mode = mode;

    // 模式切换时复位控制器，防止积分饱和
    if (old_mode != mode) {
        Dual_Loop_Control_Reset();
    }

    user_regulator_info("Control Mode: %s -> %s",
                       Get_Control_Mode_Name(old_mode), Get_Control_Mode_Name(mode));
}
void Set_Voltage_Reference(float voltage_ref)
{
    // 限制电压参考值范围
    if (voltage_ref < 5.0f) voltage_ref = 5.0f;
    if (voltage_ref > 35.0f) voltage_ref = 35.0f;

    v_ref = voltage_ref;

    user_regulator_debug("Voltage Reference: %.1fV", v_ref);
}
void Set_Current_Reference(float current_ref)
{
    // 限制电流参考值范围
    if (current_ref < 0.1f) current_ref = 0.1f;
    if (current_ref > 5.0f) current_ref = 2.5f;

    i_ref = current_ref;

    user_regulator_debug("Current Reference: %.2fA", i_ref);
}
void Set_Reference_Signal(Reference_Signal_t signal_type)
{
    // 固定使用内部信号，不允许切换
    current_reference_signal = REF_SIGNAL_INTERNAL;
    user_regulator_info("Reference Signal: Internal (Fixed)");
}
const char* Get_Reference_Signal_Name(Reference_Signal_t signal_type)
{
    switch (signal_type) {
        case REF_SIGNAL_EXTERNAL: return "EX";
        case REF_SIGNAL_INTERNAL: return "IN";
        default:                  return "UNKNOWN";
    }
}
const char* Get_Control_Mode_Name(Control_Mode_t mode)
{
    switch (mode) {
        case CONTROL_MODE_MANUAL:  return "MANUAL";
        case CONTROL_MODE_VOLTAGE: return "CV";
        case CONTROL_MODE_CURRENT: return "CC";
        default:                   return "UNKNOWN";
    }
}
const char* Get_State_Name(System_State_t state)
{
    switch (state) {
        case STATE_INIT:        return "INIT";
        case STATE_OPEN_LOOP:   return "OPEN";
        case STATE_CLOSED_LOOP: return "CLOSED";
        case STATE_FAULT:       return "FAULT";
        default:                return "UNKNOWN";
    }
}
// ============================================================================
// 归一化电压PI控制器实现 - 适应不同直流母线电压
// ============================================================================

/**
 * @brief 初始化归一化电压PI控制器
 * @param pi: 归一化PI控制器指针
 * @param kp_norm: 归一化比例增益 (基于标称电压整定)
 * @param ki_norm: 归一化积分增益 (基于标称电压整定)
 * @param output_min: 输出最小值
 * @param output_max: 输出最大值
 * @param v_dc_nominal: 标称直流母线电压 (归一化基准)
 */
void Voltage_PI_Norm_Init(Voltage_PI_Norm_t* pi, float kp_norm, float ki_norm,
                         float output_min, float output_max, float v_dc_nominal)
{
    pi->kp_norm = kp_norm;
    pi->ki_norm = ki_norm;
    pi->output = 0.0f;
    pi->output_max = output_max;
    pi->output_min = output_min;
    pi->last_error_norm = 0.0f;
    pi->v_dc_actual = v_dc_nominal;      // 初始化为标称值
    pi->v_dc_nominal = v_dc_nominal;
}

/**
 * @brief 复位归一化电压PI控制器
 * @param pi: 归一化PI控制器指针
 */
void Voltage_PI_Norm_Reset(Voltage_PI_Norm_t* pi)
{
    pi->output = 0.0f;
    pi->last_error_norm = 0.0f;
}

/**
 * @brief 更新归一化电压PI控制器
 * @param pi: 归一化PI控制器指针
 * @param v_ref: 电压参考值 (RMS)
 * @param v_feedback: 电压反馈值 (RMS)
 * @param v_dc_actual: 当前实际直流母线电压
 * @return: 归一化调制比输出 (0-1)
 *
 * 归一化原理：
 * 1. 将电压误差归一化到标称直流母线电压
 * 2. PI参数基于标称电压整定，自动适应不同直流母线电压
 * 3. 输出调制比自动补偿直流母线电压变化
 */
float Voltage_PI_Norm_Update(Voltage_PI_Norm_t* pi, float v_ref, float v_feedback, float v_dc_actual)
{
    // 更新当前直流母线电压
    pi->v_dc_actual = _fsat(v_dc_actual, V_DC_MAX, V_DC_MIN);

    // 计算电压误差
    float error = v_ref - v_feedback;

    // 归一化误差：将误差归一化到标称直流母线电压
    // 这样PI参数就具有了电压无关性
    float error_norm = error / pi->v_dc_nominal;

    // 增量式PI控制器计算
    float p_term = pi->kp_norm * (error_norm - pi->last_error_norm);
    float i_term = pi->ki_norm * error_norm;
    float delta_output = p_term + i_term;

    // 限制输出变化率，防止突变
    const float max_change = 0.02f;  // 每次最大变化2%
    delta_output = _fsat(delta_output, max_change, -max_change);

    // 累加输出
    float new_output = pi->output + delta_output;

    // 直流母线电压补偿：调制比需要根据实际直流母线电压进行补偿
    // 当直流母线电压高于标称值时，需要更小的调制比产生相同的输出电压
    float voltage_compensation = pi->v_dc_nominal / pi->v_dc_actual;
    new_output *= voltage_compensation;

    // 输出限幅和抗积分饱和
    if (new_output > pi->output_max) {
        pi->output = pi->output_max;
        // 抗积分饱和：如果输出饱和且积分项还在增大，则不更新历史误差
        if (i_term > 0) {
            pi->last_error_norm = error_norm;
            return pi->output;
        }
    } else if (new_output < pi->output_min) {
        pi->output = pi->output_min;
        if (i_term < 0) {
            pi->last_error_norm = error_norm;
            return pi->output;
        }
    } else {
        pi->output = new_output;
    }

    // 更新历史误差
    pi->last_error_norm = error_norm;

    return pi->output;
}

// ============================================================================
// 归一化PI控制器测试函数 (可选，用于验证)
// ============================================================================
/**
 * @brief 测试归一化PI控制器在不同直流母线电压下的表现
 * @note 此函数仅用于开发测试，实际应用中可删除
 */
void Test_Voltage_PI_Normalization(void)
{
    // 测试场景：相同的电压误差，不同的直流母线电压
    float v_ref_test = 15.0f;      // 参考电压15V
    float v_feedback_test = 12.0f; // 反馈电压12V (误差3V)

    // 测试30V直流母线电压 (标称值)
    float mod_30v = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref_test, v_feedback_test, 30.0f);

    // 测试60V直流母线电压 (2倍标称值)
    float mod_60v = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref_test, v_feedback_test, 60.0f);

    // 理论上：mod_60v 应该约等于 mod_30v * 0.5
    // 因为60V直流母线只需要一半的调制比就能产生相同的输出电压

    user_regulator_debug("Norm PI Test: 30V->%.3f, 60V->%.3f, Ratio:%.3f",
                        mod_30v, mod_60v, mod_60v/mod_30v);
}

// ============================================================================
// DC系数保存
//     const float adc2_in3_val = (float)adc2_in3_raw * 3.3f / 4096.0f;
//     float raw_second_current = (adc2_in3_raw - IacOffset_B) * I_MeasureGain;
// ============================================================================
// void Process_DC_Data(void)
// {
//     const uint16_t adc2_in3_raw = adc_dc_buf[0];  // ADC2_IN3 - B相电流 (10kHz采样)
//     const uint16_t adc2_in4_raw = adc_dc_buf[1];  // ADC2_IN4 - BC线电压 (10kHz采样)

//     // 转换为实际物理量（根据您的硬件调整转换系数）
//     // B相电流测量 (ADC2_IN3)
//     const float adc2_in3_val = (float)adc2_in3_raw * 3.3f / 4096.0f;  // ADC2_IN3值
//     float raw_second_current = (adc2_in3_raw - IacOffset_B) * I_MeasureGain;  // B相电流增益

//     // BC线电压测量 (ADC2_IN4)
//     const float adc2_in4_val = (float)adc2_in4_raw * 3.3f / 4096.0f;  // ADC2_IN4值
//     float raw_second_voltage = (adc2_in4_raw - VacOffset_BC) * V_MeasureGain;  // BC线电压增益

//     // 应用高级滤波器
//     dc_voltage = AlphaFilter_Update(&dc_voltage_alpha_filter, raw_second_voltage);  // BC线电压
//     dc_current = AlphaFilter_Update(&dc_current_alpha_filter, raw_second_current);  // B相电流

//     // 将原始值送入简单滑动平均滤波器（保留作为备用）
//     //Update_DC_Filter(adc2_in3_val, adc2_in4_val);
// }
