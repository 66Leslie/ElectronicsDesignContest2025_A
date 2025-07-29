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
// 新增：为电流和电压反馈信号创建SOGI滤波器
// ============================================================================
static SOGI_State sogi_filter_ia;  // A相电流的SOGI滤波器
static SOGI_State sogi_filter_ib;  // B相电流的SOGI滤波器
static SOGI_State sogi_filter_vab; // AB线电压的SOGI滤波器
static SOGI_State sogi_filter_vbc; // BC线电压的SOGI滤波器

// ============================================================================
// 三相PWM控制变量 (合并单相和三相PWM控制)
// ============================================================================
volatile float modulation_ratio = 0.5f;     // 调制比 (0.0 - 1.0)
volatile uint8_t pwm_enabled = 0;           // PWM使能标志 (统一控制三相PWM)

// 全局启动命令变量 (供按键控制)
static uint16_t Start_CMD = 1;    // 启动命令 (1=停止, 0=启动) - 初始为停止状态，等待按键启动

// ============================================================================
// 双环控制系统变量
// ============================================================================
// 控制模式和参考值
volatile Control_Mode_t control_mode = CONTROL_MODE_MANUAL;  // 当前控制模式
volatile float voltage_reference = V_REF_DEFAULT;  // 电压参考值 (RMS)
volatile float current_reference = I_REF_DEFAULT;  // 电流参考值 (RMS)

// 双PI控制器实例
PI_Controller_t voltage_pi;                 // 电压外环PI控制器 (慢环, 50Hz)
PI_Controller_t current_pi;                 // 电流内环PI控制器 (快环, 20kHz瞬时值控制)

// 控制输出变量
volatile float current_reference_peak = 0.0f;      // 电流峰值指令 (由外环输出)
volatile float current_reference_instant = 0.0f;   // 瞬时电流参考值 (20kHz)
volatile float pi_modulation_output = 0.0f;        // 最终调制比输出
volatile float current_feedback_instant = 0.0f;    // 瞬时电流反馈值

// αβ坐标系电流控制器实例
Current_Controller_AlphaBeta_t CurrConReg;
Modulation_t Modulation;
// ============================================================================
// ADC数据缓冲区 (修改为DMA长度=2的结构)
// ============================================================================
extern uint16_t adc_ac_buf[4];              // AC环路缓冲区 (在main.c中定义，DMA长度=2)
extern uint16_t adc3_reference_buf[1];      // ADC3参考信号缓冲区 (在main.c中定义)

// ============================================================================
// 动态偏置变量定义 - 每个传感器独立偏置
// ============================================================================
float VacOffset_AB = DEFAULT_VAC_OFFSET;    // AB线电压偏置 (动态测量)
float VacOffset_BC = DEFAULT_VAC_OFFSET;    // BC线电压偏置 (动态测量)
float IacOffset_A = DEFAULT_IAC_OFFSET;     // A相电流偏置 (动态测量)
float IacOffset_B = DEFAULT_IAC_OFFSET;     // B相电流偏置 (动态测量)

// ============================================================================
// 偏置测量相关变量
// ============================================================================
static uint16_t offset_sample_count = 0;    // 偏置测量采样计数器
static uint32_t voltage_offset_sum_AB = 0;  // AB线电压偏置累加器
static uint32_t voltage_offset_sum_BC = 0;  // BC线电压偏置累加器
static uint32_t current_offset_sum_A = 0;   // A相电流偏置累加器
static uint32_t current_offset_sum_B = 0;   // B相电流偏置累加器
static uint8_t offset_measurement_complete = 0;  // 偏置测量完成标志
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
volatile System_State_t system_state = STATE_INIT;  // 当前系统状态
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

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "Init", OLED_8X16);
    OLED_Update();

    // 初始化基于老师算法的高效锁相模块
    SogiQsg_Init(&g_sogi_qsg);

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
    //State_Machine_Update();
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
    if(hadc->Instance == ADC3) {//锁相或者是内部信号
        uint16_t ref_raw = adc3_reference_buf[0];
        if (current_reference_signal == REF_SIGNAL_INTERNAL) {
            // 内部信号模式：直接数学计算
            static uint32_t internal_counter = 0;
            internal_counter = (internal_counter + 1) % AC_SAMPLE_SIZE;
            float current_angle = 2.0f * M_PI * internal_counter / AC_SAMPLE_SIZE;
            g_sogi_qsg.sin_theta = sinf(current_angle);
            g_sogi_qsg.cos_theta = cosf(current_angle);
            g_sogi_qsg.is_locked = 1;  // 内部信号始终锁定
        } else {
            // 外部信号模式：使用SOGI-QSG处理外部信号
            static float ref_filtered = 2048.0f;
            ref_filtered = ref_filtered * 0.95f + (float)ref_raw * 0.05f;
            SogiQsg_Update(&g_sogi_qsg, ref_filtered);
        }
        adc_completion_mask |= (1 << 2);
    }

    // --- Part 2: 门禁检查，确保所有ADC都完成后才执行主逻辑 (10kHz/20kHz) ---
    if (adc_completion_mask != 0b00000111) {
        return; // 等待所有ADC完成
    }
    adc_completion_mask = 0; // 重置掩码，为下个周期做准备

    // --- Part 2.1: ADC数据处理 - 分别计算用于控制和RMS的值 ---

    // 基础转换值（偏置补偿 + ADC转换增益）
    float current_A_base = ((int16_t)adc_ac_buf[0] - IacOffset_A) * I_MeasureGain;
    float voltage_AB_base = ((int16_t)adc_ac_buf[1] - VacOffset_AB) * V_MeasureGain;
    float current_B_base = ((int16_t)adc_ac_buf[2] - IacOffset_B) * I_MeasureGain;
    float voltage_BC_base = ((int16_t)adc_ac_buf[3] - VacOffset_BC) * V_MeasureGain;

    // 使用限幅滤波器，目标值有1.65V偏置
    // 1.65V对应的ADC数字量 = 1.65/3.3*4096 ≈ 2048
    const float target_offset_adc = TARGET_OFFSET_VOLTAGE / 3.3f * 4096.0f;  // 对应的ADC值

    // 限幅滤波器：限制变化幅度，平滑信号
    static float current_A_last = 0.0f;
    static float current_B_last = 0.0f;
    static float voltage_AB_last = 0.0f;
    static float voltage_BC_last = 0.0f;
    static uint8_t filter_initialized = 0;

    // 首次运行时初始化滤波器
    if (!filter_initialized) {
        current_A_last = target_offset_adc * I_MeasureGain;
        current_B_last = target_offset_adc * I_MeasureGain;
        voltage_AB_last = target_offset_adc * V_MeasureGain;
        voltage_BC_last = target_offset_adc * V_MeasureGain;
        filter_initialized = 1;
    }

    const float MAX_CHANGE = 0.1f;  // 最大变化量限制（V）

    // 应用限幅滤波器
    float current_A_change = current_A_base - current_A_last;
    if (current_A_change > MAX_CHANGE) current_A_change = MAX_CHANGE;
    else if (current_A_change < -MAX_CHANGE) current_A_change = -MAX_CHANGE;
    float current_A_filtered = current_A_last + current_A_change;
    current_A_last = current_A_filtered;

    float current_B_change = current_B_base - current_B_last;
    if (current_B_change > MAX_CHANGE) current_B_change = MAX_CHANGE;
    else if (current_B_change < -MAX_CHANGE) current_B_change = -MAX_CHANGE;
    float current_B_filtered = current_B_last + current_B_change;
    current_B_last = current_B_filtered;

    float voltage_AB_change = voltage_AB_base - voltage_AB_last;
    if (voltage_AB_change > MAX_CHANGE) voltage_AB_change = MAX_CHANGE;
    else if (voltage_AB_change < -MAX_CHANGE) voltage_AB_change = -MAX_CHANGE;
    float voltage_AB_filtered = voltage_AB_last + voltage_AB_change;
    voltage_AB_last = voltage_AB_filtered;

    float voltage_BC_change = voltage_BC_base - voltage_BC_last;
    if (voltage_BC_change > MAX_CHANGE) voltage_BC_change = MAX_CHANGE;
    else if (voltage_BC_change < -MAX_CHANGE) voltage_BC_change = -MAX_CHANGE;
    float voltage_BC_filtered = voltage_BC_last + voltage_BC_change;
    voltage_BC_last = voltage_BC_filtered;

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

        ac_current_rms_A = sqrtf(current_A_avg) * 5.1778f - 0.0111f;
        ac_voltage_rms_AB = sqrtf(voltage_AB_avg) * 68.011f - 0.1784f;
        ac_current_rms_B = sqrtf(current_B_avg) * 5.1778f - 0.0111f;
        ac_voltage_rms_BC = sqrtf(voltage_BC_avg) * 68.011f - 0.1784f;

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
            switch (control_mode) {
                case CONTROL_MODE_VOLTAGE:
                    // 电压环PI控制器直接计算调制比，并存储供快环使用
                    pi_modulation_output = PI_Controller_Update_Incremental(&voltage_pi, voltage_reference, ac_voltage_rms_AB);
                    pi_modulation_output = _fsat(pi_modulation_output, PI_V_OUT_MAX, PI_V_OUT_MIN);
                    break;
                case CONTROL_MODE_CURRENT:             
                    // 更新调制比输出用于显示 (取三相调制信号的平均幅值)
                    pi_modulation_output = Modulation.Ma;
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
    switch (control_mode) {
        case CONTROL_MODE_CURRENT:
            current_reference_peak = current_reference * 1.414213562f;
            Current_Controller_AlphaBeta_Update(current_reference_peak, current_A_calibrated, current_B_calibrated);
            
            // 2. 限制三相调制信号 (增大限制范围以避免饱和)
            float mod_A = _fsat(Modulation.Ma, 1.0f, -1.0f);
            float mod_B = _fsat(Modulation.Mb, 1.0f, -1.0f);
            float mod_C = _fsat(Modulation.Mc, 1.0f, -1.0f);

            // 3. 计算三相PWM占空比 (调制信号转换为占空比)
            // 调制信号范围是[-1,1]，转换为[0,1]再乘以PWM周期
            // 添加50%的直流偏置，使调制信号在PWM中心对称
            duty_A_float = (0.5f + mod_A * 0.5f) * PWM_PERIOD_TIM8;
            duty_B_float = (0.5f + mod_B * 0.5f) * PWM_PERIOD_TIM8;
            duty_C_float = (0.5f + mod_C * 0.5f) * PWM_PERIOD_TIM8;
            break;

        case CONTROL_MODE_VOLTAGE:
        case CONTROL_MODE_MANUAL:
        default: 
            {
                float final_modulation_ratio = (control_mode == CONTROL_MODE_VOLTAGE) ? pi_modulation_output : modulation_ratio;
                final_modulation_ratio = _fsat(final_modulation_ratio, 1.0f, 0.0f);

                // 获取相位信息
                float cos_theta = g_sogi_qsg.cos_theta;
                float sin_theta = g_sogi_qsg.sin_theta;
                
                // 三相SPWM生成 (相位差120°)
                float cos_theta_A = cos_theta;                                               // A相: cos(θ)
                float cos_theta_B = cos_theta * (-0.5f) + sin_theta * (0.866025f);           // B相: cos(θ-120°)
                float cos_theta_C = cos_theta * (-0.5f) - sin_theta * (0.866025f);           // C相: cos(θ+120°)

                // 计算三相PWM占空比
                duty_A_float = ((cos_theta_A + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;
                duty_B_float = ((cos_theta_B + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;
                duty_C_float = ((cos_theta_C + 1.0f) * 0.5f) * PWM_PERIOD_TIM8 * final_modulation_ratio;
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
// 状态机相关函数实现
// ============================================================================
void State_Machine_Init(void)
{
    // 偏置测量已在初始化阶段完成，直接进入等待状态
    system_state = Wait_State;  // 跳过PowerUp_Check_State，直接进入等待状态
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
    switch(system_state) {
        case Wait_State:
            pwm_enabled = 0;  // 确保PWM关闭
            if(Start_CMD == 0) {  // 收到启动命令
                system_state = Check_State;
            }
            break;
        case Check_State:
            // 检查参考信号状态和锁相环状态
            if(current_reference_signal == REF_SIGNAL_INTERNAL)//去除锁相判断，进行测试
            //if(SogiQsg_IsLocked(&g_sogi_qsg) || current_reference_signal == REF_SIGNAL_INTERNAL)
             {
                DriveOpen_Analysis = 0;  // 可以打开驱动
                system_state = Running_State;
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
                system_state = Stop_State;
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
            system_state = Wait_State;
            break;
        case Permanent_Fault_State:
            pwm_enabled = 0;  // 禁用PWM
            DriveOpen_Analysis = 3;  // 禁止开启
            break;
        default:
            system_state = Wait_State;
            break;
    }
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

    // 步骤1: Clarke变换 - 将三相电流反馈转换到αβ坐标系 (修正系数)
    // 标准Clarke变换: α = Ia, β = (1/√3)*(Ia + 2*Ib)
    float F32alpha = current_A;
    float F32beta = (current_A + 2.0f * current_B) * 0.57735026918963f;

    // ============================================================================
    // 【修正 #1】: 修正三相电流指令的生成逻辑 - 直接在αβ坐标系生成指令
    // 避免三相到αβ再到三相的转换误差
    // ============================================================================
    // 步骤2: 直接在αβ坐标系生成电流指令 (基于锁相环的sin/cos值)
    // α轴指令: Ia_CMD * cos(θ) (与A相同相位)
    CurrConReg.Valpha_CMD = Ia_CMD * g_sogi_qsg.cos_theta;
    // β轴指令: Ia_CMD * sin(θ) (滞后α轴90度)
    CurrConReg.Vbeta_CMD = Ia_CMD * g_sogi_qsg.sin_theta;

    // 为了显示目的，计算三相电流指令
    CurrConReg.Ia_CMD = Ia_CMD * g_sogi_qsg.cos_theta;
    CurrConReg.Ib_CMD = Ia_CMD * (g_sogi_qsg.cos_theta * (-0.5f) + g_sogi_qsg.sin_theta * (-0.8660254f));
    CurrConReg.Ic_CMD = Ia_CMD * (g_sogi_qsg.cos_theta * (-0.5f) - g_sogi_qsg.sin_theta * (0.8660254f));

	// ============================================================================
    // 【修正 #2】: 修正增量式PI控制器的实现
    // 原始代码的PI控制器算法不是标准的增量式PI，无法正确累积误差
    // ============================================================================
    // 步骤4: 计算α轴误差并更新PI控制器
    CurrConReg.Error_alpha = CurrConReg.Valpha_CMD - F32alpha;
    float delta_alpha = (PI_KP_CURRENT_ALPHA * (CurrConReg.Error_alpha - CurrConReg.Error_alpha_Pre)) + (PI_KI_CURRENT_ALPHA * CurrConReg.Error_alpha);
    CurrConReg.PI_Out_alpha += delta_alpha;

    // α轴输出限幅
    CurrConReg.PI_Out_alpha = _fsat(CurrConReg.PI_Out_alpha, PI_I_OUT_MAX, PI_I_OUT_MIN);

    // 步骤5: 计算β轴误差并更新PI控制器
    CurrConReg.Error_beta = CurrConReg.Vbeta_CMD - F32beta;
    float delta_beta = (PI_KP_CURRENT_BETA * (CurrConReg.Error_beta - CurrConReg.Error_beta_Pre)) + (PI_KI_CURRENT_BETA * CurrConReg.Error_beta);
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

    // 步骤7: 反Clarke变换 - 将αβ坐标系输出转换回三相调制信号 (修正变换矩阵)
    // 标准反Clarke变换:
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
    if (debug_counter >= 2000) {  // 每2000次输出一次，避免过于频繁 (约100ms一次)
        debug_counter = 0;
        printf(">Iref:%.3f,Ia:%.3f,Ib:%.3f,Ialpha:%.3f,Ibeta:%.3f,Valpha_cmd:%.3f,Vbeta_cmd:%.3f,Valpha_out:%.3f,Vbeta_out:%.3f,Ma:%.3f,Mb:%.3f,Mc:%.3f\r\n",
               Ia_CMD, current_A, current_B, F32alpha, F32beta,
               CurrConReg.Valpha_CMD, CurrConReg.Vbeta_CMD,
               CurrConReg.PI_Out_alpha, CurrConReg.PI_Out_Beta,
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

    // 只处理按键按下事件
    if (key_result.key_state == KEY_PRESS) {
        switch (key_result.key_num) {
            case KEY1:  // 参数增加 (+)
                switch (control_mode) {
                    case CONTROL_MODE_MANUAL:
                        // 手动模式：增加调制比
                        if (modulation_ratio < 0.95f) {
                            modulation_ratio += 0.05f;
                            if (modulation_ratio > 0.95f) modulation_ratio = 0.95f;
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
                switch (control_mode) {
                    case CONTROL_MODE_MANUAL:
                        // 手动模式：减少调制比
                        if (modulation_ratio > 0.0f) {
                            modulation_ratio -= 0.05f;
                            if (modulation_ratio < 0.0f) modulation_ratio = 0.0f;
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

                if (pwm_enabled) {
                    // 启用PWM输出
                    Start_CMD = 0;
                    PWM_Enable();
                } else {
                    // 停止PWM输出
                    Start_CMD = 1;
                    PWM_Disable();
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
                if (control_mode == CONTROL_MODE_MANUAL) {
                    Set_Control_Mode(CONTROL_MODE_VOLTAGE);  // 开环 -> CV
                } else if (control_mode == CONTROL_MODE_VOLTAGE) {
                    Set_Control_Mode(CONTROL_MODE_CURRENT);  // CV -> CC
                } else {
                    Set_Control_Mode(CONTROL_MODE_MANUAL);   // CC -> 开环
                }
                user_regulator_info("Mode: %s", Get_Control_Mode_Name(control_mode));
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

        // 根据当前控制模式显示对应界面
        switch (control_mode) {
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
/**
 * @brief 手动模式显示界面 - 开环模式 6*8最多显示8行
 */
void Display_Manual_Mode_Page(void)
{
    // 使用统一的小字体，避免混用字体导致的行号计算问题
    OLED_SetLine(0);

    // 正常显示模式 (偏置测量已在初始化完成)
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
    // 初始化电压外环PI控制器 (慢环, 100Hz)
    PI_Controller_Init(&voltage_pi, PI_KP_VOLTAGE, PI_KI_VOLTAGE, PI_V_OUT_MIN, PI_V_OUT_MAX);

    // 初始化电流内环PI控制器 (快环, 20kHz瞬时值控制)
    // 注意：现在使用αβ坐标系控制器，这里保留传统PI控制器以备用
    PI_Controller_Init(&current_pi, PI_KP_CURRENT_ALPHA, PI_KI_CURRENT_ALPHA, PI_I_OUT_MIN, PI_I_OUT_MAX);

    // 初始化αβ坐标系电流控制器
    Current_Controller_AlphaBeta_Init();

    // 设置默认控制模式
    control_mode = CONTROL_MODE_MANUAL;

    // 初始化参考值
    voltage_reference = V_REF_DEFAULT;
    current_reference = I_REF_DEFAULT;
    current_reference_peak = 0.0f;

    // 初始化输出
    pi_modulation_output = 0.0f;
    current_feedback_instant = 0.0f;
    current_reference_instant = 0.0f;

    user_regulator_info("Dual Loop Control System Initialized");
    user_regulator_debug("Mode: %s, V_ref: %.1fV, I_ref: %.2fA",
                        Get_Control_Mode_Name(control_mode), voltage_reference, current_reference);
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

    // 复位输出
    current_reference_peak = 0.0f;
    pi_modulation_output = 0.0f;
    current_feedback_instant = 0.0f;
    current_reference_instant = 0.0f;

    // 复位αβ坐标系电流控制器
    Current_Controller_AlphaBeta_Reset();

    user_regulator_info("Dual Loop Control System Reset");
}


void Set_Control_Mode(Control_Mode_t mode)
{
    if (mode >= CONTROL_MODE_COUNT) return;

    Control_Mode_t old_mode = control_mode;
    control_mode = mode;

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
    if (voltage_ref > 25.0f) voltage_ref = 25.0f;

    voltage_reference = voltage_ref;

    user_regulator_debug("Voltage Reference: %.1fV", voltage_reference);
}
void Set_Current_Reference(float current_ref)
{
    // 限制电流参考值范围
    if (current_ref < 0.1f) current_ref = 0.1f;
    if (current_ref > 5.0f) current_ref = 2.5f;

    current_reference = current_ref;

    user_regulator_debug("Current Reference: %.2fA", current_reference);
}
void Set_Reference_Signal(Reference_Signal_t signal_type)
{
    if (signal_type >= REF_SIGNAL_COUNT) return;

    current_reference_signal = signal_type;

    user_regulator_info("Reference Signal set to: %s", Get_Reference_Signal_Name(signal_type));
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
void Reset_Offset_Measurement(void)
{
    // 重置所有偏置测量相关变量
    offset_sample_count = 0;
    voltage_offset_sum_AB = 0;
    voltage_offset_sum_BC = 0;
    current_offset_sum_A = 0;
    current_offset_sum_B = 0;
    offset_measurement_complete = 0;

    // 重置所有偏置值为默认值
    VacOffset_AB = DEFAULT_VAC_OFFSET;
    VacOffset_BC = DEFAULT_VAC_OFFSET;
    IacOffset_A = DEFAULT_IAC_OFFSET;
    IacOffset_B = DEFAULT_IAC_OFFSET;

    user_regulator_info("Offset measurement reset - starting new measurement");
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
