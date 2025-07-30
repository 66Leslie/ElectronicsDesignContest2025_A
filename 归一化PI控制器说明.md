# 归一化电压PI控制器实现说明

## 概述

为了解决直流母线电压变化（30V-60V）导致的PI参数泛化性问题，实现了归一化电压PI控制器。该控制器能够自动适应不同的直流母线电压，保持一致的控制性能。

## 主要特性

### 1. 电压无关性
- PI参数基于标称电压（30V）整定
- 自动适应25V-65V直流母线电压范围
- 在30V整定的参数可直接用于60V系统

### 2. 自动补偿
- 误差归一化：将电压误差归一化到标称直流母线电压
- 调制比补偿：根据实际直流母线电压自动调整输出调制比
- 动态响应一致：不同电压下保持相同的动态特性

## 核心算法

### 归一化公式

```c
// 1. 误差归一化
float error_norm = (v_ref - v_feedback) / v_dc_nominal;

// 2. PI控制器计算
float delta_output = kp_norm * (error_norm - last_error_norm) + ki_norm * error_norm;

// 3. 直流母线电压补偿
float voltage_compensation = v_dc_nominal / v_dc_actual;
output *= voltage_compensation;
```

### 补偿原理

当直流母线电压为60V时（2倍标称值）：
- 相同的调制比产生2倍的输出电压
- 因此需要0.5倍的调制比来产生相同的输出电压
- 补偿系数 = 30V / 60V = 0.5

## 代码结构

### 结构体定义

```c
typedef struct {
    float kp_norm;         // 归一化比例增益
    float ki_norm;         // 归一化积分增益
    float output;          // 当前输出值
    float output_max;      // 输出最大值
    float output_min;      // 输出最小值
    float last_error_norm; // 上次归一化误差值
    float v_dc_actual;     // 当前实际直流母线电压
    float v_dc_nominal;    // 标称直流母线电压
} Voltage_PI_Norm_t;
```

### 关键参数

```c
#define PI_KP_V_NORM 0.9f        // 归一化比例增益
#define PI_KI_V_NORM 0.36f       // 归一化积分增益
#define V_DC_NOMINAL 30.0f       // 标称直流母线电压
#define V_DC_MIN 25.0f           // 最小直流母线电压
#define V_DC_MAX 65.0f           // 最大直流母线电压
```

## 使用方法

### 1. 初始化

```c
Voltage_PI_Norm_Init(&voltage_pi_norm, PI_KP_V_NORM, PI_KI_V_NORM, 
                    PI_V_OUT_MIN, PI_V_OUT_MAX, V_DC_NOMINAL);
```

### 2. 运行时调用

```c
// 在50Hz慢速环中调用
float mod_ratio = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref, 
                                        v_feedback, v_dc_actual);
```

### 3. 直流母线电压测量

```c
// TODO: 实际应用中应通过ADC测量
float v_dc_actual = ADC_Read_DC_Bus_Voltage();
```

## 变量名简化

为了提高代码可读性，对冗长的变量名进行了简化：

| 原变量名 | 新变量名 | 说明 |
|---------|---------|------|
| `voltage_reference` | `v_ref` | 电压参考值 |
| `current_reference` | `i_ref` | 电流参考值 |
| `control_mode` | `ctrl_mode` | 控制模式 |
| `pi_modulation_output` | `mod_output` | 调制比输出 |
| `current_reference_peak` | `i_ref_peak` | 电流峰值指令 |

## 测试验证

提供了测试函数用于验证归一化效果：

```c
void Test_Voltage_PI_Normalization(void);
```

该函数测试相同误差下不同直流母线电压的调制比输出，验证补偿算法的正确性。

## 实际应用建议

1. **直流母线电压测量**：建议添加ADC通道实时测量直流母线电压
2. **参数微调**：可根据实际系统特性微调归一化参数
3. **安全限制**：确保直流母线电压在安全范围内（25V-65V）
4. **性能监控**：监控不同电压下的控制性能，必要时调整参数

## 技术优势

1. **参数泛化性强**：一套参数适用于不同电压等级
2. **维护成本低**：减少了参数调试工作量
3. **系统稳定性好**：自动补偿提高了系统鲁棒性
4. **代码可读性高**：简化的变量名便于理解和维护

## 注意事项

1. 直流母线电压测量精度会影响补偿效果
2. 极端电压条件下可能需要额外的保护措施
3. 建议在实际系统中进行充分测试验证
