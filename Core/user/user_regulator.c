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
static void Signal_Quality_Analyze(const float *signal_buffer, uint16_t buffer_size);
// ============================================================================
// 开环模式调制比控制变量
// ============================================================================
volatile float modulation_ratio = 0.5f;     // 调制比 (0.0 - 1.0)
volatile uint8_t pwm_enabled = 0;           // PWM使能标志

// 全局启动命令变量 (供按键控制)
static uint16_t Start_CMD = 1;    // 启动命令 (1=停止, 0=启动) - 初始为停止状态，等待按键启动
// ============================================================================
// 三相逆变器控制变量
// ============================================================================
volatile uint8_t three_phase_pwm_enabled = 0;      // 三相PWM使能标志
volatile float three_phase_modulation_ratio = 0.5f; // 三相调制比 (0.0 - 1.0)

// ============================================================================
// 双环控制系统变量
// ============================================================================
// 控制模式和参考值
volatile Control_Mode_t current_control_mode = CONTROL_MODE_MANUAL;  // 当前控制模式
volatile float voltage_reference = V_REF_DEFAULT;  // 电压参考值 (RMS)
volatile float current_reference = I_REF_DEFAULT;  // 电流参考值 (RMS)

// 双PI控制器实例
PI_Controller_t voltage_pi;                 // 电压外环PI控制器 (慢环, 100Hz)50actual
PI_Controller_t current_pi;                 // 电流内环PI控制器 (快环, 10kHz)50actual

// 控制输出变量
volatile float current_reference_peak = 0.0f;      // 电流峰值指令 (由外环输出)
volatile float pi_modulation_output = 0.0f;        // 最终调制比输出
volatile float current_feedback_instant = 0.0f;    // 瞬时电流反馈值

// ============================================================================
// ADC数据缓冲区 (修改为DMA长度=2的结构)
// ============================================================================
extern uint16_t adc_ac_buf[4];              // AC环路缓冲区 (在main.c中定义，DMA长度=2)
extern uint16_t adc3_reference_buf[1];      // ADC3参考信号缓冲区 (在main.c中定义)

// ============================================================================
// 简化的同步相关变量（移除SOGI-PLL）
// ============================================================================
volatile float reference_frequency = 50.0f;  // 参考信号频率（固定值）
volatile float reference_amplitude = 0.0f;   // 参考信号幅值
volatile Sync_Mode_t sync_mode = SYNC_MODE_FREE; // 同步模式（固定为自由运行）

// ============================================================================
// 参考信号选择相关变量
// ============================================================================
volatile Reference_Signal_t current_reference_signal = REF_SIGNAL_EXTERNAL;  // 当前参考信号选择

// ============================================================================
// 状态机相关变量
// ============================================================================
volatile System_State_t current_system_state = STATE_INIT;  // 当前系统状态
volatile uint32_t state_entry_time = 0;                     // 状态进入时间
volatile uint32_t state_transition_timer = 0;               // 状态转换定时器



// ============================================================================
// 高效锁相模块实例定义
// ============================================================================
SogiQsg_t g_sogi_qsg;  // 全局SOGI-QSG实例，基于老师的高效锁相算法

// ============================================================================
// 显示相关变量
// ============================================================================
volatile Display_Page_t current_page = PAGE_MAIN;  // 当前显示页面
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
volatile uint8_t ac_data_ready = 0;         // AC数据就绪标志
float ac_current_rms_A = 0.0f;              // A相电流RMS值
float ac_current_rms_B = 0.0f;              // B相电流RMS值
float ac_voltage_rms_AB = 0.0f;            // AB线电压RMS值
float ac_voltage_rms_BC = 0.0f;            // BC线电压RMS值
volatile uint8_t dc_data_ready = 0;         // DC数据就绪标志
// volatile uint8_t ref_data_ready = 0;     // 参考信号数据就绪标志 - 已废弃
float dc_voltage_filtered = 0.0f;           // DC电压滤波值
float dc_current_filtered = 0.0f;           // DC电流滤波值
float dc_voltage;                           // DC电压原始值
float dc_current;                           // DC电流原始值

// ============================================================================
// 滑动平均滤波器 (原有的简单滤波器，保留作为备用)
// ============================================================================
float dc_voltage_buffer[DC_FILTER_SIZE] = {0};
float dc_current_buffer[DC_FILTER_SIZE] = {0};
uint8_t dc_filter_index = 0;

// ============================================================================
// 高级滤波器实例 (使用data_process.h中的滤波器)
// ============================================================================
// AC信号复合滤波器
static CompositeFilter_t ac_voltage_composite_filter;
static CompositeFilter_t ac_current_composite_filter;
static float ac_voltage_moving_buffer[8];  // 滑动平均缓冲区
static float ac_current_moving_buffer[8];

// DC信号一阶低通滤波器
static AlphaFilter_t dc_voltage_alpha_filter;
static AlphaFilter_t dc_current_alpha_filter;

// ============================================================================
// 用户调节器初始化函数
// ============================================================================
void user_regulator_init(void)
{
    // 添加调试信息，帮助定位错误位置
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);  // 新增：ADC3校准

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_ac_buf, 2);   // DMA长度改为2
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_ac_buf[2], 2);   // 
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_reference_buf, 1);  // 

    HAL_TIM_Base_Start_IT(&htim1);           // 启用TIM1中断（修复：必须启用中断）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // 第一对互补PWM
    //HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // 第二对互补PWM
    //HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);   // ADC触发通道

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "Init SOGI-QSG", OLED_8X16);
    OLED_Update();

    // 初始化基于老师算法的高效锁相模块
    SogiQsg_Init(&g_sogi_qsg);

    key_init();

    // 初始化调制比
    modulation_ratio = 0.1f;  // 从较小的值开始，安全起见
    pwm_enabled = 0;          // 初始状态PWM关闭

    // 初始化三相PWM控制变量
    three_phase_modulation_ratio = 0.1f;  // 三相调制比初始值
    three_phase_pwm_enabled = 0;          // 初始状态三相PWM关闭

    // 初始化AC采样累加器
    current_A_sum = 0.0f;
    current_B_sum = 0.0f;
    voltage_AB_sum = 0.0f;
    voltage_BC_sum = 0.0f;
    voltage_AC_sum = 0.0f;
    ac_sample_count = 0;

    // 初始化高级滤波器
    Init_Advanced_Filters();

    // 初始化双环控制系统（包括PI控制器）
    Dual_Loop_Control_Init();

    // 初始化状态机和信号质量检测
    State_Machine_Init();

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

    // 1.5. 更新状态机
    State_Machine_Update();

    // 2. 更新显示
    Update_Disp();

    // 3. 处理AC反馈数据 (这部分保留)
    if(ac_data_ready) {
        ac_data_ready = 0;
        Process_AC_Sample();  // 处理单次AC采样并累加
    }

}

// ============================================================================
// TIM1中断回调函数接口 - 10kHz频率 (OC3 8500触发，参考老师代码时序)
// ============================================================================
void user_regulator_tim1_callback(void)
{
    // ========== 1. ADC数据获取和处理 (参考老师代码) ==========
    float current_real = ac_current_rms_A;
    // ========== 2. 软件过流过压保护 (参考老师代码) ==========
//    static int IL_ProCnt = 0;

//     if((current_real > IL_Peak_MAX) || (current_real < -IL_Peak_MAX)) {
//         IL_ProCnt++;
//         if(IL_ProCnt > 2) {
//             pwm_enabled = 0;  // 禁用PWM
//             // 可以在这里设置故障标志
// #if DEBUG_PRINTF_ENABLE && !DEBUG_PLL_ONLY
//             printf("Over Current Protection: I=%.2fA\r\n", current_real);
// #endif
//         }
//     } else {
//         IL_ProCnt = 0;
//     }

    // ========== 3. 锁相环更新 (参考老师代码，使用ADC3参考信号) ==========

    // 获取锁相环输出
    float cos_theta = g_sogi_qsg.cos_theta;

    // ========== 4. 控制算法 (修改为独立电压环和电流环) ==========
    float final_modulation_ratio = 0.0f;

    // 安全检查：如果PWM未使能，输出为0
    if (!pwm_enabled) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        return;
    }

    switch (current_control_mode) {
        case CONTROL_MODE_MANUAL:
            // 手动模式：直接使用手动调制比
            final_modulation_ratio = modulation_ratio;
            break;

    case CONTROL_MODE_VOLTAGE:
            // 独立电压环：使用Process_AC_Data计算的调制比 (50Hz更新)
            // 这里直接使用全局变量，避免在10kHz中断中进行复杂计算
            final_modulation_ratio = pi_modulation_output;
            break;

        case CONTROL_MODE_CURRENT:
            // 独立电流环：使用Process_AC_Data计算的调制比 (50Hz更新)
            // 这里直接使用全局变量，避免在10kHz中断中进行复杂计算
            final_modulation_ratio = pi_modulation_output;
            break;

        default:
            final_modulation_ratio = 0.0f;
            break;
    }

    // ========== 5. 调制比限制 ==========
    if (final_modulation_ratio > 1.0f) final_modulation_ratio = 1.0f;
    if (final_modulation_ratio < -1.0f) final_modulation_ratio = -1.0f;

    // ========== 6. PWM生成 (参考老师代码) ==========
    // 使用锁相环的cos输出生成PWM (类似老师代码的调制策略)
    uint32_t duty_cycle_ch1 = (float)((-cos_theta + 1.0f) *  0.5f) * PWM_PERIOD_TIM1 * final_modulation_ratio;
    uint32_t duty_cycle_ch2 = (float)(( cos_theta + 1.0f) *  0.5f) * PWM_PERIOD_TIM1 * final_modulation_ratio;

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,duty_cycle_ch1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,duty_cycle_ch2);

    // ========== 7. 更新全局变量供显示使用 ==========
    pi_modulation_output = final_modulation_ratio;
    current_feedback_instant = current_real;
}

// ============================================================================
// TIM8中断回调函数 - 三相PWM控制 (10kHz频率)
// ============================================================================
void user_regulator_tim8_callback(void)
{
    // ========== 1. 相位计算统一在ADC3回调中进行（20kHz） ==========
    // 内部信号和外部信号的相位计算都移至ADC3回调，确保时序统一

    // ========== 2. 安全检查：如果三相PWM未使能，输出为0 ==========
    if (!three_phase_pwm_enabled) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
        return;
    }

    // ========== 3. 获取ADC3回调中更新的相位信息（统一20kHz时序） ==========
    float cos_theta = g_sogi_qsg.cos_theta;

    // ========== 4. 控制算法 - 获取最终调制比 ==========
    float final_modulation_ratio = 0.0f;

    switch (current_control_mode) {
        case CONTROL_MODE_MANUAL:
            // 手动模式：直接使用三相调制比
            final_modulation_ratio = three_phase_modulation_ratio;
            break;

        case CONTROL_MODE_VOLTAGE:
            // 独立电压环：使用Process_AC_Data计算的调制比
            final_modulation_ratio = pi_modulation_output;
            break;

        case CONTROL_MODE_CURRENT:
            // 独立电流环：使用Process_AC_Data计算的调制比
            final_modulation_ratio = pi_modulation_output;
            break;

        default:
            final_modulation_ratio = 0.0f;
            break;
    }

    // ========== 5. 调制比限制 (与单相PWM保持一致) ==========
    if (final_modulation_ratio > 1.0f) final_modulation_ratio = 1.0f;
    if (final_modulation_ratio < -1.0f) final_modulation_ratio = -1.0f;

    // ========== 6. 三相SPWM生成 (相位差120°) ==========
    // A相：cos_theta (0°)
    float cos_theta_A = cos_theta;
    // B相：cos(theta - 120°) = cos_theta * cos(120°) + sin_theta * sin(120°)
    float sin_theta = g_sogi_qsg.sin_theta;
    float cos_theta_B = cos_theta * (-0.5f) + sin_theta * (0.866025f);  // cos(120°)=-0.5, sin(120°)=√3/2
    // C相：cos(theta + 120°) = cos_theta * cos(120°) - sin_theta * sin(120°)
    float cos_theta_C = cos_theta * (-0.5f) - sin_theta * (0.866025f);

    // ========== 7. 计算三相PWM占空比 (与单相PWM保持一致，添加边界检查) ==========
    // 使用与TIM1相同的计算方法，确保波形极性一致
    float duty_A_float = ((-cos_theta_A + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;
    float duty_B_float = ((-cos_theta_B + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;
    float duty_C_float = ((-cos_theta_C + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;

    // 边界检查，防止占空比超出范围导致畸变
    if (duty_A_float < 0) duty_A_float = 0;
    if (duty_A_float > PWM_PERIOD_TIM8) duty_A_float = PWM_PERIOD_TIM8;
    if (duty_B_float < 0) duty_B_float = 0;
    if (duty_B_float > PWM_PERIOD_TIM8) duty_B_float = PWM_PERIOD_TIM8;
    if (duty_C_float < 0) duty_C_float = 0;
    if (duty_C_float > PWM_PERIOD_TIM8) duty_C_float = PWM_PERIOD_TIM8;

    uint32_t duty_cycle_A = (uint32_t)duty_A_float;
    uint32_t duty_cycle_B = (uint32_t)duty_B_float;
    uint32_t duty_cycle_C = (uint32_t)duty_C_float;

    // ========== 8. 设置三相PWM占空比 ==========
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty_cycle_A);  // A相
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty_cycle_B);  // B相
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty_cycle_C);  // C相
}
// 文件: user_regulator.c

// ============================================================================
// ADC转换完成回调函数接口 - 【简化版】
// ============================================================================
void user_regulator_adc_callback(const ADC_HandleTypeDef* hadc)
{
    // === 高速AC环路的处理逻辑 ===
    if(hadc->Instance == ADC1)
    {
        ac_data_ready = 1;
    }
    // === DC环路的处理逻辑 ===
    if(hadc->Instance == ADC2)
    {
        dc_data_ready = 1;
    }

    // === ADC3：统一处理内部和外部信号（20kHz时序） ===
    if(hadc->Instance == ADC3)
    {
        uint16_t ref_raw = adc3_reference_buf[0];
        // --- 新增参考信号滤波 ---
        static float ref_filtered = 2048.0f;
        // 使用一阶低通滤波器，滤除高频噪声
        ref_filtered = ref_filtered * 0.95f + (float)ref_raw * 0.05f;

        // 信号质量检测：更新信号缓冲区（使用滤波后的值）
        //Signal_Quality_Update((uint16_t)ref_filtered);

        // 【核心】根据信号模式选择处理方式
        static uint32_t internal_counter = 0;

        if (current_reference_signal == REF_SIGNAL_INTERNAL) {
            // 内部信号模式：直接数学计算（20kHz / 50Hz = 400个点）
            internal_counter++;
            if (internal_counter >= 200) internal_counter = 0;  // 20kHz / 50Hz = 400个点

            // 直接用数学方法计算完美的sin/cos
            float current_angle = 2.0f * M_PI * internal_counter / 200.0f;
            g_sogi_qsg.sin_theta = sinf(current_angle);
            g_sogi_qsg.cos_theta = cosf(current_angle);
            g_sogi_qsg.is_locked = 1;  // 内部信号始终锁定
        } else {
            // 外部信号模式：使用SOGI-QSG处理外部信号
            SogiQsg_Update(&g_sogi_qsg, ref_raw);
        }
    }
}

// ============================================================================
// 按键处理逻辑函数
// ============================================================================
void key_proc(void)
{
    const key_result_t key_result = key_scan();

    // 只处理按键按下事件
    if (key_result.key_state == KEY_PRESS) {
        switch (key_result.key_num) {
            case KEY1:  // 参数增加 (+)
                switch (current_control_mode) {
                    case CONTROL_MODE_MANUAL:
                        // 手动模式：增加调制比
                        if (modulation_ratio < 0.95f) {
                            modulation_ratio += 0.05f;
                            if (modulation_ratio > 0.95f) modulation_ratio = 0.95f;
                            three_phase_modulation_ratio = modulation_ratio;
                        }
                        break;

                    case CONTROL_MODE_VOLTAGE:
                        // CV模式：增加电压参考值
                        Set_Voltage_Reference(voltage_reference + 1.0f);
                        break;

                    case CONTROL_MODE_CURRENT:
                        // CC模式：增加电流参考值
                        Set_Current_Reference(current_reference + 0.1f);
                        break;

                    default:
                        break;
                }
                break;

            case KEY2:  // 参数减少 (-)
                switch (current_control_mode) {
                    case CONTROL_MODE_MANUAL:
                        // 手动模式：减少调制比
                        if (modulation_ratio > 0.0f) {
                            modulation_ratio -= 0.05f;
                            if (modulation_ratio < 0.0f) modulation_ratio = 0.0f;
                            three_phase_modulation_ratio = modulation_ratio;
                        }
                        break;

                    case CONTROL_MODE_VOLTAGE:
                        // CV模式：减少电压参考值
                        Set_Voltage_Reference(voltage_reference - 1.0f);
                        break;

                    case CONTROL_MODE_CURRENT:
                        // CC模式：减少电流参考值
                        Set_Current_Reference(current_reference - 0.1f);
                        break;

                    default:
                        break;
                }
                break;

            case KEY3:  // PWM开启/关闭
                // 切换PWM使能状态
                pwm_enabled = !pwm_enabled;
                three_phase_pwm_enabled = pwm_enabled;

                if (pwm_enabled) {
                    // 启用PWM输出
                    Start_CMD = 0;
                    PWM_Enable();
                    Three_Phase_PWM_Enable();
                } else {
                    // 停止PWM输出
                    Start_CMD = 1;
                    PWM_Disable();
                    Three_Phase_PWM_Disable();
                }

                user_regulator_info("PWM %s", pwm_enabled ? "ENABLED" : "DISABLED");
                break;

            case KEY4:  // 参考信号选择
                // 切换参考信号选择
                if (current_reference_signal == REF_SIGNAL_EXTERNAL) {
                    Set_Reference_Signal(REF_SIGNAL_INTERNAL);
                } else {
                    Set_Reference_Signal(REF_SIGNAL_EXTERNAL);
                }
                user_regulator_info("Reference Signal: %s", Get_Reference_Signal_Name(current_reference_signal));
                break;

            case KEY5:  // 页面切换
                // 循环切换控制模式：Manual -> CV -> CC -> Manual
                if (current_control_mode == CONTROL_MODE_MANUAL) {
                    Set_Control_Mode(CONTROL_MODE_VOLTAGE);  // 开环 -> CV
                } else if (current_control_mode == CONTROL_MODE_VOLTAGE) {
                    Set_Control_Mode(CONTROL_MODE_CURRENT);  // CV -> CC
                } else {
                    Set_Control_Mode(CONTROL_MODE_MANUAL);   // CC -> 开环
                }
                user_regulator_info("Mode: %s", Get_Control_Mode_Name(current_control_mode));
                break;

            default:
                break;
        }
    }

    // 组合按键功能已移除，KEY4和KEY5现在有独立功能
    // KEY4: 切换参考信号选择
    // KEY5: 页面切换
}

// ============================================================================
// 更新OLED显示 - 多页面支持
// ============================================================================
void Update_Disp(void)
{
    static uint32_t last_update = 0;
    const uint32_t current_time = HAL_GetTick();

    // 每250ms更新一次显示
    if (current_time - last_update >= 250) {
        last_update = current_time;

        OLED_Clear();

        // 根据当前控制模式显示对应界面
        switch (current_control_mode) {
            case CONTROL_MODE_MANUAL:
                Display_Manual_Mode_Page();
                break;
            case CONTROL_MODE_VOLTAGE:
                Display_CV_Mode_Page();
                break;
            case CONTROL_MODE_CURRENT:
                Display_CC_Mode_Page();
                break;
            default:
                // 异常情况，强制回到手动模式
                Set_Control_Mode(CONTROL_MODE_MANUAL);
                Display_Manual_Mode_Page();
                break;
        }

        OLED_Update();
    }
}

// ============================================================================
// 针对128×64 OLED优化的显示函数 - 多页面支持
// ============================================================================

/**
 * @brief 手动模式显示界面 - 开环模式 6*8最多显示8行
 */
void Display_Manual_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 统一使用小字体，可以显示更多信息
    OLED_Println(OLED_6X8, "Mod: %.1f%%,PWM:%s", modulation_ratio * 100.0f, pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "V_AB: %.2fV", ac_voltage_rms_AB);
    OLED_Println(OLED_6X8, "I_A: %.2fA", ac_current_rms_A);
    OLED_Println(OLED_6X8, "V_BC: %.2fV", ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "I_B: %.2fA", ac_current_rms_B);
    OLED_Println(OLED_6X8, "Ref: %s", Get_Reference_Signal_Name(current_reference_signal));
    OLED_Println(OLED_6X8, "12+- 3PWM 4Ref 5Page");
}

/**
 * @brief 恒压模式(CV)显示界面 - 三相逆变器
 */
void Display_CV_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 统一使用小字体，可以显示更多信息
    OLED_Println(OLED_6X8, "CV Mode PWM:%s", pwm_enabled ? "ON" : "OFF");
    OLED_Println(OLED_6X8, "Set:%.1fV Act:%.2fV", voltage_reference, ac_voltage_rms_AB);
    OLED_Println(OLED_6X8, "Mod:%.1f%% Ref:%s", pi_modulation_output * 100.0f, Get_Reference_Signal_Name(current_reference_signal));
    OLED_Println(OLED_6X8, "I_A: %.2fA I_B: %.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "AB:%.2fV BC:%.2fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "12+- 3pwm 4Ref 5Page");
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
    OLED_Println(OLED_6X8, "Set:%.2fA Act:%.2fA", current_reference, ac_current_rms_A);
    OLED_Println(OLED_6X8, "Mod:%.1f%% Ref:%s", pi_modulation_output * 100.0f, Get_Reference_Signal_Name(current_reference_signal));
    OLED_Println(OLED_6X8, "A:%.2fA B:%.2fA", ac_current_rms_A, ac_current_rms_B);
    OLED_Println(OLED_6X8, "AB:%.1fV BC:%.1fV", ac_voltage_rms_AB, ac_voltage_rms_BC);
    OLED_Println(OLED_6X8, "K1:+ K2:- K4:Ref K5:Page");
}



// ============================================================================
// AC单次采样处理函数 (累加器方式)
// ============================================================================
void Process_AC_Sample(void)
{
    // 从DMA缓冲区读取当前采样值
    uint16_t current_raw_A  = adc_ac_buf[0];  // ADC1_IN1 (A相电流)
    uint16_t voltage_raw_AB = adc_ac_buf[1];  // ADC1_IN2 (AB线电压)
    uint16_t current_raw_B  = adc_ac_buf[2];  // ADC2_IN3 (B相电流)
    uint16_t voltage_raw_BC = adc_ac_buf[3];  // ADC2_IN4 (BC线电压)

    // 转换为真实物理量：先减去偏置，再乘以增益
    float current_A = (current_raw_A - IacOffset) * I_MeasureGain;  // A相电流
    float voltage_AB = (voltage_raw_AB - VacOffset) * V_MeasureGain;  // AB线电压
    float current_B = (current_raw_B - IacOffset) * I_MeasureGain;  // B相电流
    float voltage_BC = (voltage_raw_BC - VacOffset) * V_MeasureGain;  // BC线电压
    float voltage_AC = voltage_AB - voltage_BC;  // AC线电压

    // 累加平方值
    current_A_sum += current_A * current_A + current_A * current_A;
    voltage_AB_sum += voltage_AB * voltage_AB + voltage_AB * voltage_AB;
    current_B_sum += current_B * current_B + current_B * current_B;
    voltage_BC_sum += voltage_BC * voltage_BC + voltage_BC * voltage_BC;
    voltage_AC_sum += voltage_AC * voltage_AC + voltage_AC * voltage_AC;

    // 增加采样计数
    ac_sample_count++;

    // 检查是否采样完成 (400个点)
    if(ac_sample_count >= AC_SAMPLE_SIZE) {
        Process_AC_Data();
    }
}
// ============================================================================
// AC环路数据处理函数 (200个点采样完成后调用) - 50Hz频率 (10kHz采样率)
// ============================================================================
void Process_AC_Data(void)
{
    // 计算原始RMS值 (第一组：相电流和线电压)
    float raw_current_rms_A  = sqrtf(current_A_sum / (float)AC_SAMPLE_SIZE) * 5.1778f - 0.0111f -0.02f;
    float raw_voltage_rms_AB = sqrtf(voltage_AB_sum / (float)AC_SAMPLE_SIZE) * 48.48504f - 0.15078136f;//48.48504x-0.56078136	
    float raw_current_rms_B  = sqrtf(current_B_sum / (float)AC_SAMPLE_SIZE) * 5.1778f - 0.0111f -0.02f;
    float raw_voltage_rms_BC = sqrtf(voltage_BC_sum / (float)AC_SAMPLE_SIZE) * 68.011f - 0.1784f -0.6f;
    //根据A B 算出 C

    // 应用高级滤波器
    ac_current_rms_A = raw_current_rms_A;

    ac_voltage_rms_AB = raw_voltage_rms_AB;
    float ac_voltage_rms_A = raw_voltage_rms_AB/sqrtf(3.0f);
    ac_current_rms_B = raw_current_rms_B;
    ac_voltage_rms_BC = raw_voltage_rms_BC;

    // 清空累加器，准备下一轮采样
    current_A_sum = 0.0f;
    current_B_sum = 0.0f;
    voltage_AB_sum = 0.0f;
    voltage_BC_sum = 0.0f;
    ac_sample_count = 0;

    // ========== 【修改】独立电压环和电流环控制 (50Hz更新) ==========
    //为反馈值增加滤波器
    // static float voltage_feedback_filtered = 0.0f;
    // static uint8_t filter_init_flag = 0;
    // if(!filter_init_flag) {
    //     voltage_feedback_filtered = raw_voltage_rms;
    //     filter_init_flag = 1;
    // }
    // voltage_feedback_filtered = 0.6f * voltage_feedback_filtered + 0.4f * raw_voltage_rms;
    // // 独立的单环控制

    switch (current_control_mode) {
        case CONTROL_MODE_MANUAL:
            // 手动模式：不执行任何闭环控制
            current_reference_peak = 0.0f;
            break;

        case CONTROL_MODE_VOLTAGE:
            // 独立电压环：直接从电压RMS反馈到调制比 (慢环，50Hz更新)
            if (pwm_enabled) {
                // 电压环PI控制器直接计算调制比
                float voltage_modulation = PI_Controller_Update_Incremental(&voltage_pi, voltage_reference, ac_voltage_rms_AB);

                // 限制调制比到安全范围
                if (voltage_modulation > PI_V_OUT_MAX) voltage_modulation = PI_V_OUT_MAX;
                if (voltage_modulation < PI_V_OUT_MIN) voltage_modulation = PI_V_OUT_MIN;

                // 存储调制比供TIM1中断使用
                pi_modulation_output = voltage_modulation;
            }
            break;

        case CONTROL_MODE_CURRENT:
            // 独立电流环：从电流RMS反馈到调制比 55(快环，50Hz更新)
            if (pwm_enabled) {
                // 电流环PI控制器直接计算调制比
                float current_modulation = PI_Controller_Update_Incremental(&current_pi, current_reference, ac_current_rms_A);

                // 限制调制比到安全范围
                if (current_modulation > PI_I_OUT_MAX) current_modulation = PI_I_OUT_MAX;
                if (current_modulation < PI_I_OUT_MIN) current_modulation = PI_I_OUT_MIN;

                // 存储调制比供TIM1中断使用
                pi_modulation_output = current_modulation;
            }
            break;

        default:
            current_reference_peak = 0.0f;
            pi_modulation_output = 0.0f;
            break;
    }
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

// ============================================================================
// 增量式PI控制器更新函数
// ============================================================================
float PI_Controller_Update_Incremental(PI_Controller_t* pi, float reference, float feedback)
{
    // 1. 计算当前误差
    float error = reference - feedback;

    //添加死区控制：当误差很小时，停止积分累积
    const float deadband = 0.01f;  // 0.1V死区，增大死区减少小幅震荡
    if (fabs(error) < deadband) {
        error = 0.0f;  // 在死区内将误差设为0
    }

    // 2. 计算输出的"增量"u
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
// PI控制器重置函数
// ============================================================================
void PI_Controller_Reset(PI_Controller_t* pi)
{
    pi->output = 0.0f;    // 重置输出为0
    pi->last_error = 0.0f;
}

// ============================================================================
// 高级滤波器初始化函数
// ============================================================================
void Init_Advanced_Filters(void)
{
    // 初始化AC电压复合滤波器
    // 参数：缓冲区，大小，α系数，最大变化量，初始值
    CompositeFilter_Init(&ac_voltage_composite_filter,
                        ac_voltage_moving_buffer,
                        8,          // 滑动平均窗口8点
                        0.2f,       // α系数0.2 (提高响应速度)
                        20.0f,      // 最大变化20V (放宽限制，避免卡死)
                        0.0f);      // 初始值0V

    // 初始化AC电流复合滤波器
    CompositeFilter_Init(&ac_current_composite_filter,
                        ac_current_moving_buffer,
                        8,          // 滑动平均窗口8点
                        0.2f,       // α系数0.2 (提高响应速度)
                        10.0f,      // 最大变化10A (放宽限制，避免卡死)
                        0.0f);      // 初始值0A

    // 初始化DC电压一阶低通滤波器
    // α=0.1 适中滤波效果，平衡响应速度和滤波效果
    AlphaFilter_Init(&dc_voltage_alpha_filter, 0.1f, 0.0f);

    // 初始化DC电流一阶低通滤波器
    AlphaFilter_Init(&dc_current_alpha_filter, 0.1f, 0.0f);
}
// ============================================================================
// 状态机相关函数实现
// ============================================================================
void State_Machine_Init(void)
{
    current_system_state = PowerUp_Check_State;  // 使用正确的状态枚举
    state_entry_time = HAL_GetTick();
    state_transition_timer = 0;
}


void State_Machine_Update(void)
{
    // 参考老师代码的状态机逻辑，基于锁相环过零点控制
    static uint16_t PWM_Delay_Count = 0;    // PWM延时计数器 (10kHz计数)
    static uint16_t DriveOpen_Analysis = 3;  // 驱动状态 (0:可开启, 1:预备状态, 2:已开启, 3:禁止开启)
    static int GridVoltage_State = 0;  // 过零点状态
    static int Last_GridVoltage_State = 1;  // 上次过零点状态

    // 基于锁相环cos_theta的过零点检测
    float cos_theta = g_sogi_qsg.cos_theta;
    if((cos_theta > -0.05f) && (cos_theta < 0.05f)) {
        GridVoltage_State = 0;  // 过零点
    } else {
        GridVoltage_State = 1;  // 非过零点
    }

    // 检测过零点边沿 (从非过零点到过零点的跳变)
    int zero_crossing_detected = (Last_GridVoltage_State == 1) && (GridVoltage_State == 0);
    Last_GridVoltage_State = GridVoltage_State;

    // 状态机逻辑 (参考老师代码，增强过零点控制)
    switch(current_system_state) {
        case PowerUp_Check_State:
            pwm_enabled = 0;  // 禁用PWM
            DriveOpen_Analysis = 3;  // 禁止开启
            current_system_state = Wait_State;
            break;

        case Wait_State:
            pwm_enabled = 0;  // 确保PWM关闭
            if(Start_CMD == 0) {  // 收到启动命令
                current_system_state = Check_State;
            }
            break;

        case Check_State:
            // 检查参考信号状态和锁相环状态
            if(current_reference_signal == REF_SIGNAL_INTERNAL)//去除锁相判断，进行测试
            //if(SogiQsg_IsLocked(&g_sogi_qsg) || current_reference_signal == REF_SIGNAL_INTERNAL)
             {
                DriveOpen_Analysis = 0;  // 可以打开驱动
                current_system_state = Running_State;
                user_regulator_info("State: Check -> Running (Ref: %s, PLL: %s)",
                                   Get_Reference_Signal_Name(current_reference_signal),
                                   g_sogi_qsg.is_locked ? "LOCKED" : "UNLOCKED");
            }
            break;

        case Running_State:
            if(Start_CMD == 1 && (DriveOpen_Analysis == 2)) {
                // 收到停止命令且驱动已开启
                pwm_enabled = 0;  // 立即禁用PWM
                DriveOpen_Analysis = 3;  // 禁止开启
                current_system_state = Stop_State;
                user_regulator_info("State: Running -> Stop");
            } else {
                if(DriveOpen_Analysis == 0) {  // 驱动可以打开，等待过零点
                    if(zero_crossing_detected) {  // 检测到过零点
                        DriveOpen_Analysis = 1;  // 进入预备状态
                        PWM_Delay_Count = 0;     // 重置延时计数器
                    }
                } else if(DriveOpen_Analysis == 1) {  // 预备状态，等待100ms
                    PWM_Delay_Count++;
                    // 等待100ms = 1000个10kHz周期 (50Hz的4个周期)
                    if(PWM_Delay_Count >= 1000) {
                        PWM_Delay_Count = 0;
                        pwm_enabled = 1;  // 使能PWM
                        DriveOpen_Analysis = 2;  // 已经打开驱动
                        user_regulator_info("PWM Enabled after 100ms delay");
                    }
                }
                // DriveOpen_Analysis == 2 时，PWM已开启，保持运行状态
            }
            break;

        case Stop_State:
            pwm_enabled = 0;  // 确保PWM关闭
            DriveOpen_Analysis = 3;  // 禁止开启
            current_system_state = Wait_State;
            break;

        case Permanent_Fault_State:
            pwm_enabled = 0;  // 禁用PWM
            DriveOpen_Analysis = 3;  // 禁止开启
            break;

        default:
            current_system_state = PowerUp_Check_State;
            break;
    }

    // 按键控制启动/停止命令
    // Start_CMD 可以通过按键或其他方式控制
}

// ============================================================================
// 系统控制函数实现 (参考老师代码)
// ============================================================================

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
 * @brief 使能PWM (参考老师代码)
 */
void PWM_Enable(void)
{
    //使能PWM输出 (参考老师代码中的GPIO控制)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

    pwm_enabled = 1;
}

/**
 * @brief 禁用PWM (参考老师代码)
 */
void PWM_Disable(void)
{
    // 禁用PWM输出 (参考老师代码中的GPIO控制)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

    // 设置PWM占空比为0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    pwm_enabled = 0;
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
// 信号质量检测函数实现
// ============================================================================
void Signal_Quality_Update(uint16_t adc_value)
{
    // 静态变量用于信号质量检测和滤波
    static float signal_buffer[SIGNAL_BUFFER_SIZE];  // 信号缓冲区：2个50Hz周期
    static uint16_t buffer_index = 0;                // 缓冲区索引
    static uint16_t sample_count = 0;                // 采样计数器
    static AlphaFilter_t signal_alpha_filter;        // 一阶低通滤波器
    static uint8_t filter_initialized = 0;           // 滤波器初始化标志
    static float dc_offset_estimate = 2048.0f;       // 直流偏置估计
    static AlphaFilter_t dc_offset_filter;           // 直流偏置滤波器

    // 初始化滤波器（仅执行一次）
    if (!filter_initialized) {
        // 初始化信号滤波器：α=0.1，适合50Hz信号的低通滤波
        AlphaFilter_Init(&signal_alpha_filter, 0.1f, (float)adc_value);

        // 初始化直流偏置滤波器：α=0.001，用于慢速跟踪直流偏置
        AlphaFilter_Init(&dc_offset_filter, 0.001f, (float)adc_value);

        // 清空信号缓冲区
        for (uint16_t i = 0; i < SIGNAL_BUFFER_SIZE; i++) {
            signal_buffer[i] = 0.0f;
        }

        filter_initialized = 1;
    }
    

    // 1. 更新直流偏置估计（使用极慢的滤波器）
    dc_offset_estimate = AlphaFilter_Update(&dc_offset_filter, (float)adc_value);

    // 2. 去除直流偏置，得到交流分量
    float ac_signal = (float)adc_value - dc_offset_estimate;

    // 3. 对50Hz信号进行低通滤波，去除高频噪声
    float filtered_signal = AlphaFilter_Update(&signal_alpha_filter, ac_signal);

    // 4. 将滤波后的信号存入缓冲区
    signal_buffer[buffer_index] = filtered_signal;
    buffer_index = (buffer_index + 1) % SIGNAL_BUFFER_SIZE;
    sample_count++;

    // 5. 当收集到足够的数据时，进行信号质量分析
    if (sample_count >= SIGNAL_BUFFER_SIZE) {
        // 每收集满一个缓冲区就进行一次分析
        if (sample_count % (SIGNAL_BUFFER_SIZE / 4) == 0) {  // 每50个点分析一次
            Signal_Quality_Analyze(signal_buffer, SIGNAL_BUFFER_SIZE);
        }
    }

    // 6. 限制采样计数器，防止溢出
    if (sample_count > 10000) {
        sample_count = SIGNAL_BUFFER_SIZE;
    }
}

/**
 * @brief 信号质量分析函数
 * @param signal_buffer 信号缓冲区
 * @param buffer_size 缓冲区大小
 * @note 分析50Hz信号的质量，包括频率、幅度、平滑度等指标
 */
static void Signal_Quality_Analyze(const float *signal_buffer, uint16_t buffer_size)
{
    static Signal_Analysis_t analysis_result;
    static uint32_t analysis_count = 0;

    // 1. 计算信号的基本统计量
    float sum = 0.0f, sum_squares = 0.0f;
    float min_val = signal_buffer[0], max_val = signal_buffer[0];

    for (uint16_t i = 0; i < buffer_size; i++) {
        float val = signal_buffer[i];
        sum += val;
        sum_squares += val * val;

        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
    }

    float mean = sum / buffer_size;
    float variance = (sum_squares / buffer_size) - (mean * mean);
    float std_dev = sqrtf(variance);

    // 2. 计算信号幅度（峰峰值的一半）
    analysis_result.amplitude  = (max_val - min_val) / 2.0f;
    analysis_result.dc_offset  = mean;
    analysis_result.smoothness = std_dev;  // 使用标准差作为平滑度指标

    // 3. 简化的频率检测：计算过零点数量
    uint16_t zero_crossings = 0;
    for (uint16_t i = 1; i < buffer_size; i++) {
        if ((signal_buffer[i-1] >= 0 && signal_buffer[i] < 0) ||
            (signal_buffer[i-1] < 0 && signal_buffer[i] >= 0)) {
            zero_crossings++;
        }
    }
    analysis_result.zero_crossings = zero_crossings;

    // 4. 估算频率（基于过零点）
    // 对于50Hz信号，在400个采样点（2个周期）中应该有约4个过零点 (10kHz采样率)
    float estimated_freq = (float)zero_crossings * 10000.0f / (2.0f * buffer_size);
    analysis_result.frequency = estimated_freq;

    // 5. 信号质量评估
    if (analysis_result.amplitude < MIN_SIGNAL_AMPLITUDE ||
        analysis_result.amplitude > MAX_SIGNAL_AMPLITUDE) {
        analysis_result.quality = SIGNAL_BAD;
    } else if (fabsf(analysis_result.frequency - REFERENCE_FREQ) > FREQUENCY_TOLERANCE) {
        analysis_result.quality = SIGNAL_BAD;
    } else if (analysis_result.smoothness > SMOOTHNESS_THRESHOLD) {
        analysis_result.quality = SIGNAL_BAD;
    } else {
        analysis_result.quality = SIGNAL_GOOD;
    }

    // 6. 调试输出（每100次分析输出一次）
    analysis_count++;
    if (analysis_count % 100 == 0) {
        pll_debug("Signal Analysis: Freq=%.1fHz, Amp=%.1f, Quality=%s",
                 analysis_result.frequency,
                 analysis_result.amplitude,
                 (analysis_result.quality == SIGNAL_GOOD) ? "GOOD" : "BAD");
    }
}

// ============================================================================
// 双环控制系统管理函数
// ============================================================================

/**
 * @brief 初始化双环控制系统
 */
void Dual_Loop_Control_Init(void)
{
    // 初始化电压外环PI控制器 (慢环, 100Hz)
    PI_Controller_Init(&voltage_pi, PI_KP_VOLTAGE, PI_KI_VOLTAGE, PI_V_OUT_MIN, PI_V_OUT_MAX);

    // 初始化电流内环PI控制器 (快环, 10kHz)
    PI_Controller_Init(&current_pi, PI_KP_CURRENT, PI_KI_CURRENT, PI_I_OUT_MIN, PI_I_OUT_MAX);

    // 设置默认控制模式
    current_control_mode = CONTROL_MODE_MANUAL;

    // 初始化参考值
    voltage_reference = V_REF_DEFAULT;
    current_reference = I_REF_DEFAULT;
    current_reference_peak = 0.0f;

    // 初始化输出
    pi_modulation_output = 0.0f;
    current_feedback_instant = 0.0f;

    user_regulator_info("Dual Loop Control System Initialized");
    user_regulator_debug("Mode: %s, V_ref: %.1fV, I_ref: %.2fA",
                        Get_Control_Mode_Name(current_control_mode), voltage_reference, current_reference);
}

/**
 * @brief 复位双环控制系统
 */
void Dual_Loop_Control_Reset(void)
{
    // 复位PI控制器
    PI_Controller_Reset(&voltage_pi);
    PI_Controller_Reset(&current_pi);

    // 复位输出
    current_reference_peak = 0.0f;
    pi_modulation_output = 0.0f;
    current_feedback_instant = 0.0f;

    user_regulator_info("Dual Loop Control System Reset");
}

/**
 * @brief 设置控制模式
 * @param mode 控制模式
 */
void Set_Control_Mode(Control_Mode_t mode)
{
    if (mode >= CONTROL_MODE_COUNT) return;

    Control_Mode_t old_mode = current_control_mode;
    current_control_mode = mode;

    // 模式切换时复位控制器，防止积分饱和
    if (old_mode != mode) {
        Dual_Loop_Control_Reset();
    }

    user_regulator_info("Control Mode: %s -> %s",
                       Get_Control_Mode_Name(old_mode), Get_Control_Mode_Name(mode));
}

/**
 * @brief 设置电压参考值
 * @param voltage_ref 电压参考值 (RMS)
 */
void Set_Voltage_Reference(float voltage_ref)
{
    // 限制电压参考值范围
    if (voltage_ref < 5.0f) voltage_ref = 5.0f;
    if (voltage_ref > 25.0f) voltage_ref = 25.0f;

    voltage_reference = voltage_ref;

    user_regulator_debug("Voltage Reference: %.1fV", voltage_reference);
}

/**
 * @brief 设置电流参考值
 * @param current_ref 电流参考值 (RMS)
 */
void Set_Current_Reference(float current_ref)
{
    // 限制电流参考值范围
    if (current_ref < 0.1f) current_ref = 0.1f;
    if (current_ref > 2.0f) current_ref = 2.0f;

    current_reference = current_ref;

    user_regulator_debug("Current Reference: %.2fA", current_reference);
}

/**
 * @brief 获取控制模式名称
 * @param mode 控制模式
 * @return 模式名称字符串
 */
const char* Get_Control_Mode_Name(Control_Mode_t mode)
{
    switch (mode) {
        case CONTROL_MODE_MANUAL:  return "MANUAL";
        case CONTROL_MODE_VOLTAGE: return "CV";
        case CONTROL_MODE_CURRENT: return "CC";
        default:                   return "UNKNOWN";
    }
}

// ============================================================================
// 参考信号选择函数实现
// ============================================================================

/**
 * @brief 设置参考信号类型
 * @param signal_type 参考信号类型
 */
void Set_Reference_Signal(Reference_Signal_t signal_type)
{
    if (signal_type >= REF_SIGNAL_COUNT) return;

    current_reference_signal = signal_type;

    user_regulator_info("Reference Signal set to: %s", Get_Reference_Signal_Name(signal_type));
}

/**
 * @brief 获取参考信号类型名称
 * @param signal_type 参考信号类型
 * @return 信号类型名称字符串
 */
const char* Get_Reference_Signal_Name(Reference_Signal_t signal_type)
{
    switch (signal_type) {
        case REF_SIGNAL_EXTERNAL: return "EX";
        case REF_SIGNAL_INTERNAL: return "IN";
        default:                  return "UNKNOWN";
    }
}

// ============================================================================
// 三相逆变器控制函数实现
// ============================================================================

/**
 * @brief 使能三相PWM输出 (TIM8)
 */
void Three_Phase_PWM_Enable(void)
{
    //shut down
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_TIM_Base_Start_IT(&htim8);
    // 启动TIM8的PWM输出
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // A相
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // B相
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  // C相

    // 设置使能标志
    three_phase_pwm_enabled = 1;

    three_phase_info("Three Phase PWM Enabled (TIM8)");
}

/**
 * @brief 禁用三相PWM输出 (TIM8)
 */
void Three_Phase_PWM_Disable(void)
{
    //shut down
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    // 停止TIM8的PWM输出
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);   // A相
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);   // B相
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);   // C相

    // 停止TIM8的中断
    HAL_TIM_Base_Stop_IT(&htim8);

    // 清除使能标志
    three_phase_pwm_enabled = 0;

    // 确保输出为0
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

    three_phase_info("Three Phase PWM Disabled (TIM8)");
}

// ============================================================================
// DC系数保存
//     const float adc2_in3_val = (float)adc2_in3_raw * 3.3f / 4096.0f;  
//     float raw_second_current = (adc2_in3_raw - IacOffset) * I_MeasureGain;  
// ============================================================================
// void Process_DC_Data(void)
// {
//     const uint16_t adc2_in3_raw = adc_dc_buf[0];  // ADC2_IN3 - B相电流 (10kHz采样)
//     const uint16_t adc2_in4_raw = adc_dc_buf[1];  // ADC2_IN4 - BC线电压 (10kHz采样)

//     // 转换为实际物理量（根据您的硬件调整转换系数）
//     // B相电流测量 (ADC2_IN3)
//     const float adc2_in3_val = (float)adc2_in3_raw * 3.3f / 4096.0f;  // ADC2_IN3值
//     float raw_second_current = (adc2_in3_raw - IacOffset) * I_MeasureGain;  // B相电流增益

//     // BC线电压测量 (ADC2_IN4)
//     const float adc2_in4_val = (float)adc2_in4_raw * 3.3f / 4096.0f;  // ADC2_IN4值
//     float raw_second_voltage = (adc2_in4_raw - VacOffset) * V_MeasureGain;  // BC线电压增益

//     // 应用高级滤波器
//     dc_voltage = AlphaFilter_Update(&dc_voltage_alpha_filter, raw_second_voltage);  // BC线电压
//     dc_current = AlphaFilter_Update(&dc_current_alpha_filter, raw_second_current);  // B相电流

//     // 将原始值送入简单滑动平均滤波器（保留作为备用）
//     //Update_DC_Filter(adc2_in3_val, adc2_in4_val);
// }
