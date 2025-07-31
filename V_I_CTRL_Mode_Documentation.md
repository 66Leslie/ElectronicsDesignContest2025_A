# V_I_CTRL模式功能说明

## 概述
新增的V_I_CTRL模式实现了电压电流分离控制功能，允许逆变器和整流器使用不同的控制算法：
- **TIM8 CH1/CH2/CH3（逆变器）**：恒压控制
- **TIM8 CH4 + TIM1 CH1/CH2（整流器）**：恒流控制

## 功能特点

### 1. 分离控制算法
- **逆变器恒压控制**：
  - 使用归一化电压PI控制器（voltage_pi_norm）
  - 基于ac_voltage_rms_AB反馈
  - 采用SVPWM调制，支持1.1547倍调制比
  - 50Hz慢速环更新

- **整流器恒流控制**：
  - 使用αβ坐标系电流控制器
  - 基于current_A_calibrated和current_B_calibrated反馈
  - 20kHz快速环更新
  - 支持三相电流控制

### 2. 独立PWM输出
- **逆变器PWM**（TIM8 CH1-3）：根据电压控制算法计算占空比
- **整流器PWM**（TIM8 CH4 + TIM1 CH1-2）：根据电流控制算法计算占空比
- 两套PWM输出完全独立，可以有不同的占空比

### 3. 用户界面
- **显示页面**：PAGE_V_I
- **显示内容**：
  - 逆变器电压设定值和实际值
  - 整流器电流设定值和实际值
  - 两套调制比输出
  - 实时电压电流测量值

### 4. 按键控制
- **KEY1**：增加电压参考值（逆变器恒压控制）
- **KEY2**：增加电流参考值（整流器恒流控制）
- **KEY4**：减少电流参考值（整流器恒流控制）
- **KEY3**：PWM开启/关闭
- **KEY5**：页面切换

## 页面切换顺序
Manual -> CV -> CC -> **V_I** -> Freq -> Manual

## 控制逻辑

### 50Hz慢速环（外环）
```c
case CONTROL_MODE_V_I_CTRL:
    // 电压控制器更新（用于逆变器）
    float v_dc_actual_vi = V_DC_NOMINAL;
    mod_output = Voltage_PI_Norm_Update(&voltage_pi_norm, v_ref,
                                       ac_voltage_rms_AB, v_dc_actual_vi);
    mod_output = _fsat(mod_output, PI_V_OUT_MAX, PI_V_OUT_MIN);
    break;
```

### 20kHz快速环（内环）
```c
case CONTROL_MODE_V_I_CTRL:
    // 1. 逆变器恒压控制（SVPWM）
    // 使用mod_output计算三相调制信号
    
    // 2. 整流器恒流控制（αβ坐标系）
    // 使用电流控制器计算三相调制信号
    
    // 3. 分别设置两套PWM输出
    break;
```

## 技术实现

### 代码修改文件
1. **Core/user/user_regulator.h**
   - 添加CONTROL_MODE_V_I_CTRL枚举
   - 添加PAGE_V_I显示页面枚举
   - 添加Display_V_I_Mode_Page函数声明

2. **Core/user/user_regulator.c**
   - 50Hz慢速环添加V_I_CTRL控制逻辑
   - 20kHz快速环添加分离PWM控制逻辑
   - 添加V_I模式显示页面函数
   - 更新按键处理逻辑
   - 更新页面切换逻辑

### 关键变量
- `v_ref`：电压参考值（用于逆变器恒压控制）
- `i_ref`：电流参考值（用于整流器恒流控制）
- `mod_output`：电压控制器输出的调制比
- `Modulation.Ma/Mb/Mc`：电流控制器输出的三相调制信号

## 使用方法

1. **进入V_I模式**：
   - 按KEY5切换到V_I页面
   - 系统自动切换到CONTROL_MODE_V_I_CTRL模式

2. **参数调节**：
   - KEY1：增加电压参考值（步长1.0V）
   - KEY2：增加电流参考值（步长0.1A）
   - KEY4：减少电流参考值（步长0.1A）

3. **启动控制**：
   - KEY3：开启/关闭PWM输出

4. **监控状态**：
   - 观察OLED显示的设定值和实际值
   - 监控两套调制比输出

## 安全特性

1. **参数限制**：
   - 电压参考值：5.0V - 35.0V
   - 电流参考值：0.1A - 5.0A

2. **输出限制**：
   - 电压调制比：0.0 - 1.1547（SVPWM扩展范围）
   - 电流调制比：-0.9 - 0.9

3. **模式切换保护**：
   - 切换模式时自动复位控制器状态
   - 防止积分饱和

## 应用场景
- 双向变流器系统
- 需要同时进行电压和电流控制的应用
- 逆变器和整流器独立控制的场合
- 电力电子实验和测试平台
