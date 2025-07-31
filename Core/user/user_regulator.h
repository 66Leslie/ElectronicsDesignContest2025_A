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
// 三相PWM控制变量 (逆变器+整流器同步输出)
// ============================================================================
#define SINE_TABLE_SIZE 400   // 10kHz/25Hz = 400个采样点，三相SPWM生成
#define PWM_PERIOD_TIM8 8499     // TIM8的ARR (三相PWM)
#define PWM_PERIOD_TIM1 8499     // TIM1的ARR (三相PWM，与TIM8同步)
#define V_DC_NOMINAL 30.0f       // 标称直流母线电压 30V (用于归一化基准)
#define V_DC_MIN 25.0f           // 最小直流母线电压
#define V_DC_MAX 65.0f           // 最大直流母线电压
// 修正的测量增益系数 - 基于硬件设计计算 (修正数值避免无穷大)
#define MeasureGain 0.000805664f     // ADC转换增益: 3.3V / 4096 = 0.000805664f
// 偏置测量相关定义 - 基于1.65V偏置
#define IacOffset_A     2048.0f    //adc_ac_buf[0] 48 56.17 8.17 2039.83
#define VacOffset_AB    2048.0f   //adc_ac_buf[1]
#define IacOffset_B     2048.0f    //adc_ac_buf[2]12.25 48 -12.25 35.75
#define VacOffset_BC    2048.0f   //adc_ac_buf[3]



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
// 归一化电压环PI参数定义 - 适应不同直流母线电压
// ============================================================================
// --- 归一化电压环 (50Hz更新) - 输出归一化调制比 ---
// 归一化参数：基于30V标称电压整定，自动适应25V-65V范围
#define PI_KP_V_NORM 0.9f        // 归一化电压环比例增益 (基于标称电压)
#define PI_KI_V_NORM 0.3f       // 归一化电压环积分增益 (基于标称电压)
#define PI_V_OUT_MAX  1.1547f    // 电压环输出最大值 (SVPWM扩展调制比: 2/√3)
#define PI_V_OUT_MIN  0.0f       // 电压环输出最小值 (归一化调制比)

// --- αβ坐标系电流环控制参数 (20kHz更新) - 调整参数以改善响应 ---
#define PI_KP_CURRENT_ALPHA 0.5f    // α轴电流环比例增益 (增大以提高响应速度)
#define PI_KI_CURRENT_ALPHA 0.1f    // α轴电流环积分增益 (增大以减少稳态误差)
#define PI_KP_CURRENT_BETA  0.5f    // β轴电流环比例增益 (增大以提高响应速度)
#define PI_KI_CURRENT_BETA  0.1f    // β轴电流环积分增益 (增大以减少稳态误差)
#define PI_I_OUT_MAX  0.9f          // 电流环输出最大值 (调制比)
#define PI_I_OUT_MIN  -0.9f         // 电流环输出最小值 (αβ坐标系可以为负)
// --- 默认参考值 ---
#define V_REF_DEFAULT 5.0f     // 默认参考电压 (RMS)
#define I_REF_DEFAULT 0.5f      // 默认参考电流 (RMS)


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
// 归一化电压PI控制器结构体 - 适应不同直流母线电压
// ============================================================================
typedef struct {
    float kp_norm;         // 归一化比例增益 (基于标称电压)
    float ki_norm;         // 归一化积分增益 (基于标称电压)
    float output;          // 当前输出值 (归一化调制比 0-1)
    float output_max;      // 输出最大值 (归一化调制比)
    float output_min;      // 输出最小值 (归一化调制比)
    float last_error_norm; // 上次归一化误差值
    float v_dc_actual;     // 当前实际直流母线电压
    float v_dc_nominal;    // 标称直流母线电压 (归一化基准)
} Voltage_PI_Norm_t;

// ============================================================================
// αβ坐标系电流控制器结构体
// ============================================================================
typedef struct {
    // 三相电流指令值 (abc坐标系)
    float Ia_CMD;
    float Ib_CMD;
    float Ic_CMD;

    // αβ坐标系电压指令值
    float Valpha_CMD;
    float Vbeta_CMD;

    // αβ坐标系误差值
    float Error_alpha;
    float Error_alpha_Pre;
    float Error_beta;
    float Error_beta_Pre;

    // αβ坐标系PI控制器输出
    float PI_Out_alpha;
    float PI_Out_Beta;

    // 前馈补偿
    float feedforward_a;
    float feedforward_b;
    float feedforward_c;
} Current_Controller_AlphaBeta_t;

// ============================================================================
// 三相调制信号结构体
// ============================================================================
typedef struct {
    float Ma;  // A相调制比
    float Mb;  // B相调制比
    float Mc;  // C相调制比
} Modulation_t;

// ============================================================================
// 数据处理变量和测量增益系数 (参考老师代码)
// ============================================================================
#define AC_SAMPLE_SIZE 400     // AC采样点数（每通道，对应50Hz周期）20kHz(update event)
// ============================================================================
// 高效锁相模块实例声明
// ============================================================================
extern SogiQsg_t g_sogi_qsg;  // 全局SOGI-QSG实例，基于老师的高效锁相算法

// ============================================================================
// 控制输出变量声明 (简化变量名)
// ============================================================================
extern volatile float i_ref_peak;          // 电流峰值指令 (由外环输出)
extern volatile float i_ref_inst;          // 瞬时电流参考值 (20kHz)
extern volatile float mod_output;          // 最终调制比输出
extern volatile float i_fdbk_inst;         // 瞬时电流反馈值

// αβ坐标系电流控制器实例
extern Current_Controller_AlphaBeta_t CurrConReg;
extern Modulation_t Modulation;

// ============================================================================
// PI控制器实例声明
// ============================================================================
extern PI_Controller_t voltage_pi;                 // 传统电压PI控制器 (兼容性保留)
extern PI_Controller_t current_pi;                 // 电流PI控制器
extern Voltage_PI_Norm_t voltage_pi_norm;          // 归一化电压PI控制器 (新增)

// ============================================================================
// 控制模式和参考值声明 (简化变量名)
// ============================================================================
extern volatile Control_Mode_t ctrl_mode;          // 当前控制模式
extern volatile float v_ref;                       // 电压参考值 (RMS)
extern volatile float i_ref;                       // 电流参考值 (RMS)

// ============================================================================
// 用户调节器模块函数声明
// ============================================================================
void user_regulator_init(void);
void user_regulator_main(void);

// ============================================================================
// 回调函数接口 (TIM8回调已移除，逻辑整合至ADC回调)
// ============================================================================
void user_regulator_adc_callback(const ADC_HandleTypeDef* hadc);

// ============================================================================
// 内部函数声明
// ============================================================================
void key_proc(void);
void Update_Disp(void);

// ============================================================================
// 显示页面函数声明
// ============================================================================
// 控制模式显示函数声明 (针对128×64 OLED优化 - 三相逆变器)
void Display_Manual_Mode_Page(void);  // 手动模式显示（开环调制比）
void Display_CV_Mode_Page(void);      // 恒压模式显示（三相）
void Display_CC_Mode_Page(void);      // 恒流模式显示（三相）
void Display_Freq_Mode_Page(void);    // 频率调节页面显示（开环频率调节）

// ============================================================================
// PI控制器函数声明
// ============================================================================
void PI_Controller_Init(PI_Controller_t* pi, float kp, float ki, float output_min, float output_max);
void PI_Controller_Reset(PI_Controller_t* pi);
// 增量式PI控制器更新函数声明
float PI_Controller_Update_Incremental(PI_Controller_t* pi, float reference, float feedback);

// ============================================================================
// 归一化电压PI控制器函数声明
// ============================================================================
/**
 * 归一化电压PI控制器使用说明：
 *
 * 1. 初始化：使用基于30V标称电压整定的参数
 *    Voltage_PI_Norm_Init(&voltage_pi_norm, PI_KP_V_NORM, PI_KI_V_NORM,
 *                        PI_V_OUT_MIN, PI_V_OUT_MAX, V_DC_NOMINAL);
 *
 * 2. 运行时调用：自动适应25V-65V直流母线电压范围
 *    float mod_ratio = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref, v_feedback, v_dc_actual);
 *
 * 3. 优势：
 *    - PI参数具有电压无关性，在30V整定的参数可直接用于60V
 *    - 自动补偿直流母线电压变化
 *    - 保持相同的动态响应特性
 */
void Voltage_PI_Norm_Init(Voltage_PI_Norm_t* pi, float kp_norm, float ki_norm,
                         float output_min, float output_max, float v_dc_nominal);
void Voltage_PI_Norm_Reset(Voltage_PI_Norm_t* pi);
float Voltage_PI_Norm_Update(Voltage_PI_Norm_t* pi, float v_ref, float v_feedback, float v_dc_actual);


// ============================================================================
// αβ坐标系电流控制器函数声明
// ============================================================================
void Current_Controller_AlphaBeta_Init(void);
void Current_Controller_AlphaBeta_Reset(void);
void Current_Controller_AlphaBeta_Update(float Ia_CMD, float current_A, float current_B);
float _fsat(float value, float max_val, float min_val);



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
    PAGE_MANUAL = 0,    // 手动模式页面：开环调制比控制
    PAGE_CV,            // 恒压模式页面：CV控制
    PAGE_CC,            // 恒流模式页面：CC控制
    PAGE_FREQ,          // 频率调节页面：开环频率调节
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
// 偏置测量相关函数声明
// ============================================================================
void Measure_ADC_Offsets(void);     // 测量ADC偏置函数
uint8_t Is_Offset_Measurement_Complete(void);  // 检查偏置测量是否完成
void Reset_Offset_Measurement(void); // 重置偏置测量，重新开始测量
void Perform_Initial_Offset_Measurement(void); // 在初始化阶段完成偏置测量

// ============================================================================
// 状态机相关函数声明
// ============================================================================
void State_Machine_Init(void);
void State_Machine_PLL(void);
void Set_System_State(System_State_t new_state);
void Set_Start_CMD(uint16_t cmd);  // 设置启动命令 (供按键控制)

// ============================================================================
// 系统控制函数声明 (参考老师代码)
// ============================================================================
void USER_Regulator_Start(void);    // 启动系统 (参考老师代码)
void USER_Regulator_Stop(void);     // 停止系统
void PWM_Enable(void);               // 使能PWM (同时启动逆变器和整流器)
void PWM_Disable(void);              // 禁用PWM (同时停止逆变器和整流器)
void Test_Rectifier_Sync(void);     // 测试整流器与逆变器同步性
void Start_TIM8_For_Offset_Measurement(void);  // 启动TIM8用于偏置测量（保持shutdown低电平关断输出）

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

#endif //USER_REGULATOR_H
