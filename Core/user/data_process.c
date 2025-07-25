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
