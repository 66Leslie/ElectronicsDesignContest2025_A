// Created by 86195 on 25-7-14.
// OLED 128*64 长宽比2比1 
// 6*8字体下 每一行最多显示21个字符
// 6*8字体下 最多显示16行

#ifndef USER_REGULATOR_H
#define USER_REGULATOR_H

#include "main.h"
#include "stdio.h"
#include "math.h"
#include "sogi_qsg.h"      // 新增：基于老师算法的高效锁相模块
#include "debug_config.h"  // 新增：调试配置文件

// ============================================================================
// 开环模式调制比控制变量
// ============================================================================
#define SINE_TABLE_SIZE 400   // 10kHz/25Hz = 400个采样点，使用180度相位差生成第二路PWM
#define ADC_AC_BUFFER_SIZE 800  // AC环路DMA缓冲区大小 (400个采样点 × 2个通道)
#define PWM_PERIOD_TIM1 htim1.Init.Period     // TIM1的ARR
#define PWM_PERIOD_TIM8 htim8.Init.Period     // TIM8的ARR
#define V_DC 30.0f            // 直流母线电压 30V
#define V_MeasureGain (0.00080586f)      // 线电压测量增益 (AB线电压, BC线电压) ideal = 1/( 1 / 20e3 * 300 / 3.3 * 4096) = 0.05371//y = 68.011x - 0.1784
#define I_MeasureGain (0.00080586f)     // 相电流测量增益 (A相电流, B相电流) ideal = 1/( 1 * 2匝  * 0.625V/A *  / 3.3 * 4096) = 0.00064
#define VacOffset     2048.0f       // 线电压偏置 (AB, BC线电压)
#define IacOffset     2048.0f       // 相电流偏置 (A, B相电流)
// ============================================================================
// 控制模式枚举 - 支持三种控制模式
// ============================================================================
typedef enum {
    CONTROL_MODE_MANUAL = 0,    // 手动调制比模式
    CONTROL_MODE_VOLTAGE,       // 恒压输出模式 (CV)
    CONTROL_MODE_CURRENT,       // 恒流输出模式 (CC)
    CONTROL_MODE_PLL_DEBUG,     // 开环锁相调试模式 (新增)
    CONTROL_MODE_COUNT
} Control_Mode_t;

// ============================================================================
// 独立环PI参数定义 - 电压环和电流环独立控制
// ============================================================================
// --- 独立电压环 (50Hz更新) - 直接输出调制比 ---
#define PI_KP_VOLTAGE 0.04f      // 电压环比例增益 (调制比输出，适当增大)
#define PI_KI_VOLTAGE 0.02f      // 电压环积分增益 (调制比输出，适当增大)
#define PI_V_OUT_MAX  0.9f       // 电压环输出最大值 (调制比)
#define PI_V_OUT_MIN  0.0f       // 电压环输出最小值 (调制比)

// --- 独立电流环 (50Hz更新) - 直接输出调制比 ---
#define PI_KP_CURRENT 0.1f       // 电流环比例增益 (调制比输出)
#define PI_KI_CURRENT 0.02f      // 电流环积分增益 (调制比输出)
#define PI_I_OUT_MAX  0.9f       // 电流环输出最大值 (调制比)
#define PI_I_OUT_MIN  0.0f       // 电流环输出最小值

// --- 默认参考值 ---
#define V_REF_DEFAULT 5.0f     // 默认参考电压 (RMS)
#define I_REF_DEFAULT 1.0f      // 默认参考电流 (RMS)


// ============================================================================
// PI控制器结构体
// ============================================================================
typedef struct {
    float kp;              // 比例增益
    float ki;              // 积分增益
    float output;          // 当前输出值 (用于增量式PI控制器)
    float output_max;      // 输出最大值
    float output_min;      // 输出最小值
    float last_error;      // 上次误差值
} PI_Controller_t;

// ============================================================================
// 数据处理变量和测量增益系数 (参考老师代码)
// ============================================================================
#define DC_FILTER_SIZE 16
#define AC_SAMPLE_SIZE 400     // AC采样点数（每通道，对应50Hz周期）20khz(OC3 4250)

// 保护阈值定义 (参考老师代码)
#define IL_Peak_MAX         5.0f        // 电流峰值保护阈值 (A)
#define Vdc_MAX             50.0f       // 直流电压保护阈值 (V)
#define DCVol_Coef          0.025f      // 直流电压系数 (参考老师代码)

// ============================================================================
// 高效锁相模块实例声明
// ============================================================================
extern SogiQsg_t g_sogi_qsg;  // 全局SOGI-QSG实例，基于老师的高效锁相算法

// ============================================================================
// 用户调节器模块函数声明
// ============================================================================
void user_regulator_init(void);
void user_regulator_main(void);

// ============================================================================
// 回调函数接口
// ============================================================================
void user_regulator_tim1_callback(void);
void user_regulator_tim8_callback(void);  // 新增：TIM8三相PWM回调函数
void user_regulator_adc_callback(const ADC_HandleTypeDef* hadc);

// ============================================================================
// 内部函数声明
// ============================================================================
void Generate_Sine_Table(void);
void key_proc(void);
void Update_Disp(void);
void Process_AC_Sample(void);
void Process_AC_Data(void);
// void Process_ref_Signal(void);  // 已废弃 - 功能移至ADC回调中
//void Process_DC_Data(void);
float Calculate_RMS_Real(const uint16_t* data, uint16_t length, float gain, float offset);
void Update_DC_Filter(float voltage, float current);

// ============================================================================
// 显示页面函数声明
// ============================================================================
// 控制模式显示函数声明 (针对128×64 OLED优化 - 三相逆变器)
void Display_Manual_Mode_Page(void);  // 手动模式显示（开环）
void Display_CV_Mode_Page(void);      // 恒压模式显示（三相）
void Display_CC_Mode_Page(void);      // 恒流模式显示（三相）


// ============================================================================
// PI控制器函数声明
// ============================================================================
void PI_Controller_Init(PI_Controller_t* pi, float kp, float ki, float output_min, float output_max);
void PI_Controller_Reset(PI_Controller_t* pi);
// 增量式PI控制器更新函数声明
float PI_Controller_Update_Incremental(PI_Controller_t* pi, float reference, float feedback);

// ============================================================================
// 双环控制相关函数声明
// ============================================================================
void Dual_Loop_Control_Init(void);
void Dual_Loop_Control_Reset(void);
void Set_Control_Mode(Control_Mode_t mode);
Control_Mode_t Get_Control_Mode(void);
void Set_Voltage_Reference(float voltage_ref);
void Set_Current_Reference(float current_ref);
const char* Get_Control_Mode_Name(Control_Mode_t mode);

// ============================================================================
// 高级滤波器函数声明
// ============================================================================
void Init_Advanced_Filters(void);

// ============================================================================
// 显示页面枚举
// ============================================================================
typedef enum {
    PAGE_MAIN = 0,      // 主页面：基本信息
    PAGE_CONTROL,       // 控制页面：控制参数
    PAGE_COUNT          // 页面总数
} Display_Page_t;

// ============================================================================
// 同步模式枚举
// ============================================================================
typedef enum {
    SYNC_MODE_FREE = 0, // 自由运行模式（开环锁相）
    SYNC_MODE_PLL,      // PLL同步模式（闭环锁相）
    SYNC_MODE_COUNT     // 模式总数
} Sync_Mode_t;

// ============================================================================
// 参考信号选择枚举
// ============================================================================
typedef enum {
    REF_SIGNAL_EXTERNAL = 0,    // 外部参考信号 (ADC3_IN1)
    REF_SIGNAL_INTERNAL,        // 内部参考信号 (软件生成)
    REF_SIGNAL_COUNT            // 信号源总数
} Reference_Signal_t;

// ============================================================================
// 简化的接口函数声明（移除PLL）
// ============================================================================
float Get_Reference_Frequency(void);
float Get_Reference_Amplitude(void);
uint8_t Is_PLL_Locked(void);
void Set_Sync_Mode(uint8_t mode);

// ============================================================================
// 参考信号选择函数声明
// ============================================================================
void Set_Reference_Signal(Reference_Signal_t signal_type);
Reference_Signal_t Get_Reference_Signal(void);
const char* Get_Reference_Signal_Name(Reference_Signal_t signal_type);

// ============================================================================
// 系统状态机枚举 (参考老师代码)
// ============================================================================
typedef enum {
    PowerUp_Check_State = 0,    // 上电检查状态 (参考老师代码)
    Wait_State,                 // 等待状态
    Check_State,                // 检查状态
    Running_State,              // 运行状态
    Stop_State,                 // 停止状态
    Permanent_Fault_State,      // 永久故障状态
    STATE_COUNT                 // 状态总数
} System_State_t;

// 兼容性定义 (保持现有代码工作)
#define STATE_INIT          PowerUp_Check_State
#define STATE_OPEN_LOOP     Wait_State
#define STATE_CLOSED_LOOP   Running_State
#define STATE_FAULT         Permanent_Fault_State

// ============================================================================
// 故障标志结构体 (参考老师代码)
// ============================================================================
typedef union {
    struct {
        uint16_t IL_OverCurrent_SW : 1;     // 软件过流保护
        uint16_t DCVoltage_OV : 1;          // 直流过压保护
        uint16_t GridVoltage_State : 1;     // 电网电压状态
        uint16_t Reserved : 13;             // 保留位
    } bit;
    struct {
        uint16_t Fault1;                    // 故障字1
        uint16_t Fault2;                    // 故障字2 (预留)
    } Word;
} SYS_FAULT_FLAG;

// ============================================================================
// 状态机相关函数声明
// ============================================================================
void State_Machine_Init(void);
void State_Machine_Update(void);
System_State_t Get_Current_State(void);
void Set_System_State(System_State_t new_state);
const char* Get_State_Name(System_State_t state);
void Set_Start_CMD(uint16_t cmd);  // 设置启动命令 (供按键控制)

// ============================================================================
// 系统控制函数声明 (参考老师代码)
// ============================================================================
void USER_Regulator_Start(void);    // 启动系统 (参考老师代码)
void USER_Regulator_Stop(void);     // 停止系统
void PWM_Enable(void);               // 使能PWM (参考老师代码)
void PWM_Disable(void);              // 禁用PWM (参考老师代码)

// ============================================================================
// 三相逆变器控制函数声明
// ============================================================================
void Three_Phase_PWM_Enable(void);   // 使能三相PWM (TIM8)
void Three_Phase_PWM_Disable(void);  // 禁用三相PWM (TIM8)

// ============================================================================
// 信号质量检测和阈值锁相模块
// ============================================================================
#define SIGNAL_BUFFER_SIZE      400     // 缓冲区大小：2个50Hz周期 (10kHz采样率)
#define MIN_SIGNAL_AMPLITUDE    500     // 最小信号幅度 (ADC计数)
#define MAX_SIGNAL_AMPLITUDE    3500    // 最大信号幅度 (ADC计数)
#define FREQUENCY_TOLERANCE     2.0f    // 频率容差 (Hz)
#define SMOOTHNESS_THRESHOLD    100     // 平滑度阈值 (ADC计数的标准差)

// 频率关系定义
#define REFERENCE_FREQ          50.0f   // 参考信号频率 (Hz)

// 信号质量状态
typedef enum {
    SIGNAL_UNKNOWN,     // 未知状态
    SIGNAL_COLLECTING,  // 正在收集数据
    SIGNAL_ANALYZING,   // 正在分析信号质量
    SIGNAL_GOOD,        // 信号质量良好
    SIGNAL_BAD          // 信号质量差
} Signal_Quality_t;

// 同步状态机
typedef enum {
    SYNC_IDLE,          // 空闲，等待信号质量确认
    SYNC_READY,         // 信号质量良好，准备同步
    SYNC_CORRECTING,    // 正在进行相位校正
    SYNC_LOCKED         // 已锁定，正常运行
} Sync_State_t;

// 信号分析结果
typedef struct {
    float frequency;        // 检测到的频率
    float amplitude;        // 信号幅度
    float dc_offset;        // 直流偏置
    float smoothness;       // 平滑度指标
    uint16_t zero_crossings; // 过零点数量
    Signal_Quality_t quality; // 信号质量评估
} Signal_Analysis_t;


// ============================================================================
// 信号质量检测函数声明
// ============================================================================
void Signal_Quality_Init(void);
void Signal_Quality_Update(uint16_t adc_value);

// ============================================================================
// DAC输出相关函数声明
// ============================================================================
void Init_DAC_Output_Buffer(void);
void Update_DAC_Output(void);

#endif //USER_REGULATOR_H
