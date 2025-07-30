/*
 * data_process.c
 *
 *  Created on: 2025年7月17日
 *      Author: Augment Agent
 *  Description: 数据处理和滤波算法实现
 */

#include "data_process.h"

// ============================================================================
// 冒泡排序算法实现
// ============================================================================

/*
 * 函数名：void Bubble_Sort_Uint16(uint16_t dat[], uint16_t Num)
 * 作用：16位无符号整数冒泡排序
 * 输入：一长度为Num，首地址为dat[]的16位无符号整型数组
 * 输出：结果仍存放在该数组里
 * 返回值：无
 * 备注：通过调用该函数，可以使某个大小为Num的整型数组自动从小到大排序。
 */
void Bubble_Sort_Uint16(uint16_t dat[], uint16_t Num)
{
    uint16_t i, j, buff;
    for(i = 0; i < Num - 1; i++)
    {
        for(j = 0; j < Num - 1 - i; j++)
        {
            if(dat[j] > dat[j + 1])
            {
                buff = dat[j + 1];
                dat[j + 1] = dat[j];
                dat[j] = buff;
            }
        }
    }
}

/*
 * 函数名：void Bubble_Sort_Uint32(uint32_t dat[], uint16_t Num)
 * 作用：32位无符号整数冒泡排序
 */
void Bubble_Sort_Uint32(uint32_t dat[], uint16_t Num)
{
    uint16_t i, j;
    uint32_t buff;
    for(i = 0; i < Num - 1; i++)
    {
        for(j = 0; j < Num - 1 - i; j++)
        {
            if(dat[j] > dat[j + 1])
            {
                buff = dat[j + 1];
                dat[j + 1] = dat[j];
                dat[j] = buff;
            }
        }
    }
}

/*
 * 函数名：void Bubble_Sort_Int32(int32_t dat[], uint16_t Num)
 * 作用：32位有符号整数冒泡排序
 */
void Bubble_Sort_Int32(int32_t dat[], uint16_t Num)
{
    uint16_t i, j;
    int32_t buff;
    for(i = 0; i < Num - 1; i++)
    {
        for(j = 0; j < Num - 1 - i; j++)
        {
            if(dat[j] > dat[j + 1])
            {
                buff = dat[j + 1];
                dat[j + 1] = dat[j];
                dat[j] = buff;
            }
        }
    }
}

/*
 * 函数名：void Bubble_Sort_Float(float dat[], uint16_t Num)
 * 作用：浮点数冒泡排序
 * 输入：一长度为Num，首地址为dat[]的浮点数组
 * 输出：结果仍存放在该数组里
 * 返回值：无
 * 备注：通过调用该函数，可以使某个大小为Num的浮点数组自动从小到大排序。
 */
void Bubble_Sort_Float(float dat[], uint16_t Num)
{
    uint16_t i, j;
    float buff;
    for(i = 0; i < Num - 1; i++)
    {
        for(j = 0; j < Num - 1 - i; j++)
        {
            if(dat[j] > dat[j + 1])
            {
                buff = dat[j + 1];
                dat[j + 1] = dat[j];
                dat[j] = buff;
            }
        }
    }
}

// ============================================================================
// 中位值滤波算法实现
// ============================================================================

/*
 * 函数名：uint16_t Median_Filter_Uint16(uint16_t dat[], uint16_t Num, uint8_t N_avg)
 * 作用：16位无符号整数中位值平均滤波法
 * 输入：一长度为Num，首地址为dat[]的整型数组，N_avg为取平均的点数
 * 返回值：uint16_t
 * 备注：当对某信号多次采样时，可调用该函数实现中位值平均滤波法，可将一些毛刺、脉冲干扰滤除
 */
uint16_t Median_Filter_Uint16(uint16_t dat[], uint16_t Num, uint8_t N_avg)
{
    uint16_t i;
    uint32_t result = 0;
    
    // 对数据进行排序
    Bubble_Sort_Uint16(dat, Num);
    
    // 取中间N_avg个数据的平均值
    for(i = (Num - N_avg) / 2; i < (Num + N_avg) / 2; i++)
        result += dat[i];
    
    result = result / N_avg;
    return (uint16_t)result;
}

/*
 * 函数名：uint32_t Median_Filter_Uint32(uint32_t dat[], uint16_t Num, uint8_t N_avg)
 * 作用：32位无符号整数中位值平均滤波法
 */
uint32_t Median_Filter_Uint32(uint32_t dat[], uint16_t Num, uint8_t N_avg)
{
    uint16_t i;
    uint64_t result = 0;
    
    // 对数据进行排序
    Bubble_Sort_Uint32(dat, Num);
    
    // 取中间N_avg个数据的平均值
    for(i = (Num - N_avg) / 2; i < (Num + N_avg) / 2; i++)
        result += dat[i];
    
    result = result / N_avg;
    return (uint32_t)result;
}

/*
 * 函数名：int32_t Median_Filter_Int32(int32_t dat[], uint16_t Num, uint8_t N_avg)
 * 作用：32位有符号整数中位值平均滤波法
 */
int32_t Median_Filter_Int32(int32_t dat[], uint16_t Num, uint8_t N_avg)
{
    uint16_t i;
    int64_t result = 0;
    
    // 对数据进行排序
    Bubble_Sort_Int32(dat, Num);
    
    // 取中间N_avg个数据的平均值
    for(i = (Num - N_avg) / 2; i < (Num + N_avg) / 2; i++)
        result += dat[i];
    
    result = result / N_avg;
    return (int32_t)result;
}

/*
 * 函数名：float Median_Filter_Float(float dat[], uint16_t Num, uint8_t N_avg)
 * 作用：浮点数中位值平均滤波法
 * 输入：一长度为Num，首地址为dat[]的浮点数组，N_avg为取平均的点数
 * 返回值：float
 * 备注：当对某信号多次采样时，可调用该函数实现中位值平均滤波法，可将一些毛刺、脉冲干扰滤除
 */
float Median_Filter_Float(float dat[], uint16_t Num, uint8_t N_avg)
{
    uint16_t i;
    float result = 0.0f;
    
    // 对数据进行排序
    Bubble_Sort_Float(dat, Num);
    
    // 取中间N_avg个数据的平均值
    for(i = (Num - N_avg) / 2; i < (Num + N_avg) / 2; i++)
        result += dat[i];
    
    result = result / N_avg;
    return result;
}

// ============================================================================
// 滑动平均滤波算法实现
// ============================================================================

/*
 * 函数名：float Moving_Average_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index)
 * 作用：滑动平均滤波器
 * 输入：new_value - 新的采样值，buffer - 滤波缓冲区，size - 缓冲区大小，index - 当前索引指针
 * 返回值：float - 滤波后的值
 * 备注：实现滑动窗口平均滤波，适用于连续数据流的实时滤波
 */
float Moving_Average_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index)
{
    uint16_t i;
    float sum = 0.0f;

    // 将新值存入缓冲区
    buffer[*index] = new_value;
    *index = (*index + 1) % size;

    // 计算缓冲区所有值的平均值
    for(i = 0; i < size; i++)
    {
        sum += buffer[i];
    }

    return sum / size;
}

/*
 * 函数名：uint16_t Moving_Average_Filter_Uint16(uint16_t new_value, uint16_t buffer[], uint16_t size, uint16_t *index)
 * 作用：16位无符号整数滑动平均滤波器
 */
uint16_t Moving_Average_Filter_Uint16(uint16_t new_value, uint16_t buffer[], uint16_t size, uint16_t *index)
{
    uint16_t i;
    uint32_t sum = 0;

    // 将新值存入缓冲区
    buffer[*index] = new_value;
    *index = (*index + 1) % size;

    // 计算缓冲区所有值的平均值
    for(i = 0; i < size; i++)
    {
        sum += buffer[i];
    }

    return (uint16_t)(sum / size);
}

// ============================================================================
// 一阶低通滤波算法实现
// ============================================================================

/*
 * 函数名：float Alpha_Filter(float new_value, float *last_output, float alpha)
 * 作用：一阶低通滤波器（α滤波器）
 * 输入：new_value - 新的采样值，last_output - 上次输出值指针，alpha - 滤波系数(0-1)
 * 返回值：float - 滤波后的值
 * 备注：alpha越小滤波效果越强，但响应越慢；alpha越大响应越快，但滤波效果越弱
 */
float Alpha_Filter(float new_value, float *last_output, float alpha)
{
    *last_output = alpha * new_value + (1.0f - alpha) * (*last_output);
    return *last_output;
}

/*
 * 函数名：uint16_t Alpha_Filter_Uint16(uint16_t new_value, uint16_t *last_output, float alpha)
 * 作用：16位无符号整数一阶低通滤波器
 */
uint16_t Alpha_Filter_Uint16(uint16_t new_value, uint16_t *last_output, float alpha)
{
    float temp = alpha * (float)new_value + (1.0f - alpha) * (float)(*last_output);
    *last_output = (uint16_t)temp;
    return *last_output;
}

// ============================================================================
// 限幅滤波算法实现
// ============================================================================

/*
 * 函数名：float Limit_Filter(float new_value, float *last_value, float max_change)
 * 作用：限幅滤波器（程序判断滤波法）
 * 输入：new_value - 新的采样值，last_value - 上次有效值指针，max_change - 最大允许变化量
 * 返回值：float - 滤波后的值
 * 备注：当新采样值与上次有效值的差值超过最大允许变化量时，认为是干扰，使用上次有效值
 */
float Limit_Filter(float new_value, float *last_value, float max_change)
{
    // 如果是第一次调用（last_value为0且new_value不为0），直接接受新值
    if (*last_value == 0.0f && new_value != 0.0f) {
        *last_value = new_value;
        return new_value;
    }

    if (fabsf(new_value - *last_value) <= max_change) {
        *last_value = new_value;  // 更新有效值
        return new_value;
    } else {
        // 如果变化太大，逐步调整而不是完全拒绝
        if (new_value > *last_value) {
            *last_value += max_change;
        } else {
            *last_value -= max_change;
        }
        return *last_value;
    }
}

/*
 * 函数名：uint16_t Limit_Filter_Uint16(uint16_t new_value, uint16_t *last_value, uint16_t max_change)
 * 作用：16位无符号整数限幅滤波器
 */
uint16_t Limit_Filter_Uint16(uint16_t new_value, uint16_t *last_value, uint16_t max_change)
{
    uint16_t change = (new_value > *last_value) ? (new_value - *last_value) : (*last_value - new_value);

    if (change <= max_change) {
        *last_value = new_value;  // 更新有效值
        return new_value;
    } else {
        return *last_value;  // 返回上次有效值
    }
}

// ============================================================================
// 复合滤波算法实现
// ============================================================================

/*
 * 函数名：float Composite_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index,
 *                                float *alpha_last, float alpha, float *limit_last, float max_change)
 * 作用：复合滤波器（限幅 + 滑动平均 + 一阶低通）
 * 输入：new_value - 新采样值，buffer - 滑动平均缓冲区，size - 缓冲区大小，index - 索引指针
 *       alpha_last - α滤波器上次输出，alpha - α系数，limit_last - 限幅滤波上次值，max_change - 最大变化量
 * 返回值：float - 滤波后的值
 * 备注：先进行限幅滤波，再进行滑动平均，最后进行一阶低通滤波，综合多种滤波优点
 */
float Composite_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index,
                      float *alpha_last, float alpha, float *limit_last, float max_change)
{
    // 第一步：限幅滤波
    float step1 = Limit_Filter(new_value, limit_last, max_change);

    // 第二步：滑动平均滤波
    float step2 = Moving_Average_Filter(step1, buffer, size, index);

    // 第三步：一阶低通滤波
    float step3 = Alpha_Filter(step2, alpha_last, alpha);

    return step3;
}

// ============================================================================
// 滤波器结构体操作函数实现
// ============================================================================

/*
 * 函数名：void MovingAvgFilter_Init(MovingAvgFilter_t *filter, float *buffer, uint16_t size)
 * 作用：初始化滑动平均滤波器
 */
void MovingAvgFilter_Init(MovingAvgFilter_t *filter, float *buffer, uint16_t size)
{
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->initialized = 1;

    // 清零缓冲区
    for(uint16_t i = 0; i < size; i++) {
        buffer[i] = 0.0f;
    }
}

/*
 * 函数名：float MovingAvgFilter_Update(MovingAvgFilter_t *filter, float new_value)
 * 作用：更新滑动平均滤波器
 */
float MovingAvgFilter_Update(MovingAvgFilter_t *filter, float new_value)
{
    if (!filter->initialized) return new_value;

    return Moving_Average_Filter(new_value, filter->buffer, filter->size, &filter->index);
}

/*
 * 函数名：void AlphaFilter_Init(AlphaFilter_t *filter, float alpha, float initial_value)
 * 作用：初始化一阶低通滤波器
 */
void AlphaFilter_Init(AlphaFilter_t *filter, float alpha, float initial_value)
{
    filter->alpha = alpha;
    filter->last_output = initial_value;
    filter->initialized = 1;
}

/*
 * 函数名：float AlphaFilter_Update(AlphaFilter_t *filter, float new_value)
 * 作用：更新一阶低通滤波器
 */
float AlphaFilter_Update(AlphaFilter_t *filter, float new_value)
{
    if (!filter->initialized) return new_value;

    return Alpha_Filter(new_value, &filter->last_output, filter->alpha);
}

/*
 * 函数名：void LimitFilter_Init(LimitFilter_t *filter, float max_change, float initial_value)
 * 作用：初始化限幅滤波器
 */
void LimitFilter_Init(LimitFilter_t *filter, float max_change, float initial_value)
{
    filter->max_change = max_change;
    filter->last_value = initial_value;
    filter->initialized = 1;
}

/*
 * 函数名：float LimitFilter_Update(LimitFilter_t *filter, float new_value)
 * 作用：更新限幅滤波器
 */
float LimitFilter_Update(LimitFilter_t *filter, float new_value)
{
    if (!filter->initialized) return new_value;

    return Limit_Filter(new_value, &filter->last_value, filter->max_change);
}

/*
 * 函数名：void CompositeFilter_Init(CompositeFilter_t *filter, float *buffer, uint16_t size,
 *                                  float alpha, float max_change, float initial_value)
 * 作用：初始化复合滤波器
 */
void CompositeFilter_Init(CompositeFilter_t *filter, float *buffer, uint16_t size,
                         float alpha, float max_change, float initial_value)
{
    // 初始化各个子滤波器
    MovingAvgFilter_Init(&filter->moving_avg, buffer, size);
    AlphaFilter_Init(&filter->alpha_filter, alpha, initial_value);
    LimitFilter_Init(&filter->limit_filter, max_change, initial_value);

    filter->initialized = 1;
}

/*
 * 函数名：float CompositeFilter_Update(CompositeFilter_t *filter, float new_value)
 * 作用：更新复合滤波器
 */
float CompositeFilter_Update(CompositeFilter_t *filter, float new_value)
{
    if (!filter->initialized) return new_value;

    // 第一步：限幅滤波
    float step1 = LimitFilter_Update(&filter->limit_filter, new_value);

    // 第二步：滑动平均滤波
    float step2 = MovingAvgFilter_Update(&filter->moving_avg, step1);

    // 第三步：一阶低通滤波
    float step3 = AlphaFilter_Update(&filter->alpha_filter, step2);

    return step3;
}

// ============================================================================
// SOGI滤波器算法实现
// ============================================================================

/*
 * 函数名：void SOGIFilter_Init(SOGIFilter_t *filter, float target_freq, float sampling_freq, float damping_factor)
 * 作用：初始化SOGI滤波器
 * 输入：filter - SOGI滤波器结构体指针，target_freq - 目标频率(Hz)，sampling_freq - 采样频率(Hz)，damping_factor - 阻尼系数
 * 返回值：无
 * 备注：SOGI（Second Order Generalized Integrator）是一种二阶广义积分器，特别适用于提取特定频率的正弦信号
 */
void SOGIFilter_Init(SOGIFilter_t *filter, float target_freq, float sampling_freq, float damping_factor)
{
    // 计算角频率
    filter->omega = 2.0f * M_PI * target_freq;

    // 计算采样周期
    filter->ts = 1.0f / sampling_freq;

    // 计算增益系数
    filter->k = damping_factor;

    // 初始化状态变量
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->last_input = 0.0f;

    filter->initialized = 1;
}

/*
 * 函数名：float SOGIFilter_Update(SOGIFilter_t *filter, float input)
 * 作用：更新SOGI滤波器
 * 输入：filter - SOGI滤波器结构体指针，input - 输入信号
 * 返回值：float - 滤波后的信号（同相分量）
 * 备注：使用双线性变换实现的离散化SOGI滤波器，输出为提取的基波分量
 */
float SOGIFilter_Update(SOGIFilter_t *filter, float input)
{
    if (!filter->initialized) return input;

    // SOGI离散化实现（使用双线性变换）
    // 连续时间SOGI方程：
    // dx1/dt = k*omega*(input - x1) - omega*x2
    // dx2/dt = omega*x1
    // y1 = x1 (同相分量)
    // y2 = x2 (正交分量)

    float omega_ts = filter->omega * filter->ts;
    float k_omega_ts = filter->k * omega_ts;

    // 使用欧拉法进行数值积分
    float dx1 = k_omega_ts * (input - filter->x1) - omega_ts * filter->x2;
    float dx2 = omega_ts * filter->x1;

    // 更新状态变量
    filter->x1 += dx1;
    filter->x2 += dx2;

    // 输出同相分量（滤波后的信号）
    filter->y1 = filter->x1;
    filter->y2 = filter->x2;  // 正交分量（可用于相位检测等）

    return filter->y1;
}

/*
 * 函数名：void SOGIFilter_Reset(SOGIFilter_t *filter)
 * 作用：重置SOGI滤波器状态
 */
void SOGIFilter_Reset(SOGIFilter_t *filter)
{
    if (!filter->initialized) return;

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->last_input = 0.0f;
}

/*
 * 函数名：void SOGICompositeFilter_Init(SOGICompositeFilter_t *filter, float target_freq, float sampling_freq,
 *                                      float damping_factor, float max_change, float alpha, float initial_value)
 * 作用：初始化SOGI复合滤波器（限幅 + SOGI + 一阶低通）
 * 输入：filter - 复合滤波器结构体指针，target_freq - 目标频率，sampling_freq - 采样频率
 *       damping_factor - 阻尼系数，max_change - 限幅最大变化量，alpha - 一阶低通滤波系数，initial_value - 初始值
 * 返回值：无
 * 备注：先进行限幅滤波去除异常值，再使用SOGI提取基波分量，最后进行一阶低通滤波，适用于含噪声的50Hz正弦信号
 */
void SOGICompositeFilter_Init(SOGICompositeFilter_t *filter, float target_freq, float sampling_freq,
                             float damping_factor, float max_change, float alpha, float initial_value)
{
    // 初始化限幅滤波器
    LimitFilter_Init(&filter->limit_filter, max_change, initial_value);

    // 初始化SOGI滤波器
    SOGIFilter_Init(&filter->sogi_filter, target_freq, sampling_freq, damping_factor);

    // 初始化一阶低通滤波器参数
    filter->alpha = alpha;
    filter->last_output = initial_value;

    filter->initialized = 1;
}

/*
 * 函数名：float SOGICompositeFilter_Update(SOGICompositeFilter_t *filter, float new_value)
 * 作用：更新SOGI复合滤波器
 * 输入：filter - 复合滤波器结构体指针，new_value - 新的输入值
 * 返回值：float - 滤波后的值
 * 备注：对于20kHz采样的50Hz信号，先限幅再SOGI滤波，最后一阶低通滤波，有效提取基波分量并进一步平滑
 */
float SOGICompositeFilter_Update(SOGICompositeFilter_t *filter, float new_value)
{
    if (!filter->initialized) return new_value;

    // 第一步：限幅滤波，去除异常的跳变值
    float step1 = LimitFilter_Update(&filter->limit_filter, new_value);

    // 第二步：SOGI滤波，提取50Hz基波分量
    float step2 = SOGIFilter_Update(&filter->sogi_filter, step1);

    // 第三步：一阶低通滤波
    float step3 = Alpha_Filter(step2, &filter->last_output, filter->alpha);

    return step3;
}

/*
 * 函数名：void SOGICompositeFilter_Reset(SOGICompositeFilter_t *filter)
 * 作用：重置SOGI复合滤波器状态
 */
void SOGICompositeFilter_Reset(SOGICompositeFilter_t *filter)
{
    if (!filter->initialized) return;

    // 重置限幅滤波器（保持配置参数）
    filter->limit_filter.last_value = 0.0f;

    // 重置SOGI滤波器
    SOGIFilter_Reset(&filter->sogi_filter);

    // 重置一阶低通滤波器状态
    filter->last_output = 0.0f;
}
