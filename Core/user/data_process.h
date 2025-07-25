/*
 * data_process.h
 *
 *  Created on: 2025年7月17日
 *      Author: Augment Agent
 *  Description: 数据处理和滤波算法头文件
 */

#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include "main.h"
#include "math.h"

// ============================================================================
// 滤波算法参数定义
// ============================================================================
#define MEDIAN_FILTER_SIZE 5        // 中位值滤波器窗口大小
#define MEDIAN_AVG_SIZE 3           // 中位值平均滤波中取平均的点数
#define MOVING_AVG_SIZE 8           // 滑动平均滤波器窗口大小
#define ALPHA_FILTER_COEFF 0.1f     // 一阶低通滤波器系数 (0-1, 越小滤波效果越强)
#define LIMIT_FILTER_MAX_CHANGE 5.0f // 限幅滤波器最大允许变化量

// ============================================================================
// 冒泡排序算法函数声明
// ============================================================================
void Bubble_Sort_Uint16(uint16_t dat[], uint16_t Num);
void Bubble_Sort_Uint32(uint32_t dat[], uint16_t Num);
void Bubble_Sort_Int32(int32_t dat[], uint16_t Num);
void Bubble_Sort_Float(float dat[], uint16_t Num);

// ============================================================================
// 中位值滤波算法函数声明
// ============================================================================
uint16_t Median_Filter_Uint16(uint16_t dat[], uint16_t Num, uint8_t N_avg);
uint32_t Median_Filter_Uint32(uint32_t dat[], uint16_t Num, uint8_t N_avg);
int32_t Median_Filter_Int32(int32_t dat[], uint16_t Num, uint8_t N_avg);
float Median_Filter_Float(float dat[], uint16_t Num, uint8_t N_avg);

// ============================================================================
// 滑动平均滤波算法函数声明
// ============================================================================
float Moving_Average_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index);
uint16_t Moving_Average_Filter_Uint16(uint16_t new_value, uint16_t buffer[], uint16_t size, uint16_t *index);

// ============================================================================
// 一阶低通滤波算法函数声明
// ============================================================================
float Alpha_Filter(float new_value, float *last_output, float alpha);
uint16_t Alpha_Filter_Uint16(uint16_t new_value, uint16_t *last_output, float alpha);

// ============================================================================
// 限幅滤波算法函数声明
// ============================================================================
float Limit_Filter(float new_value, float *last_value, float max_change);
uint16_t Limit_Filter_Uint16(uint16_t new_value, uint16_t *last_value, uint16_t max_change);

// ============================================================================
// 复合滤波算法函数声明
// ============================================================================
float Composite_Filter(float new_value, float buffer[], uint16_t size, uint16_t *index, 
                      float *alpha_last, float alpha, float *limit_last, float max_change);

// ============================================================================
// 滤波器状态结构体定义
// ============================================================================

// 滑动平均滤波器状态结构体
typedef struct {
    float *buffer;          // 滤波缓冲区指针
    uint16_t size;          // 缓冲区大小
    uint16_t index;         // 当前索引
    uint8_t initialized;    // 初始化标志
} MovingAvgFilter_t;

// 一阶低通滤波器状态结构体
typedef struct {
    float last_output;      // 上次输出值
    float alpha;            // 滤波系数
    uint8_t initialized;    // 初始化标志
} AlphaFilter_t;

// 限幅滤波器状态结构体
typedef struct {
    float last_value;       // 上次有效值
    float max_change;       // 最大允许变化量
    uint8_t initialized;    // 初始化标志
} LimitFilter_t;

// 复合滤波器状态结构体
typedef struct {
    MovingAvgFilter_t moving_avg;   // 滑动平均滤波器
    AlphaFilter_t alpha_filter;     // 一阶低通滤波器
    LimitFilter_t limit_filter;     // 限幅滤波器
    uint8_t initialized;            // 初始化标志
} CompositeFilter_t;

// ============================================================================
// 滤波器初始化和操作函数声明
// ============================================================================
void MovingAvgFilter_Init(MovingAvgFilter_t *filter, float *buffer, uint16_t size);
float MovingAvgFilter_Update(MovingAvgFilter_t *filter, float new_value);

void AlphaFilter_Init(AlphaFilter_t *filter, float alpha, float initial_value);
float AlphaFilter_Update(AlphaFilter_t *filter, float new_value);

void LimitFilter_Init(LimitFilter_t *filter, float max_change, float initial_value);
float LimitFilter_Update(LimitFilter_t *filter, float new_value);

void CompositeFilter_Init(CompositeFilter_t *filter, float *buffer, uint16_t size, 
                         float alpha, float max_change, float initial_value);
float CompositeFilter_Update(CompositeFilter_t *filter, float new_value);

#endif // DATA_PROCESS_H
