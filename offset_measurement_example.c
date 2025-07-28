/**
 * @file offset_measurement_example.c
 * @brief ADC偏置测量功能使用示例
 * @author AI Assistant
 * @date 2025-07-28
 */

#include "user_regulator.h"

/**
 * @brief 示例：如何在应用中使用动态测量的偏置值
 */
void example_usage_of_measured_offsets(void)
{
    // 等待偏置测量完成
    while (!Is_Offset_Measurement_Complete()) {
        // 可以在这里执行其他初始化任务
        HAL_Delay(10);
    }
    
    // 现在可以安全地使用测量得到的偏置值
    user_regulator_info("Using measured offsets: VacOffset=%.1f, IacOffset=%.1f", 
                       VacOffset, IacOffset);
    
    // 示例：处理ADC数据时使用动态偏置
    uint16_t adc_raw_voltage = 2100;  // 假设的ADC原始值
    uint16_t adc_raw_current = 2050;  // 假设的ADC原始值
    
    // 使用动态测量的偏置进行数据处理
    float voltage_value = ((int16_t)adc_raw_voltage - VacOffset) * V_MeasureGain;
    float current_value = ((int16_t)adc_raw_current - IacOffset) * I_MeasureGain;
    
    user_regulator_debug("Processed values: Voltage=%.3fV, Current=%.3fA", 
                        voltage_value, current_value);
}

/**
 * @brief 示例：检查偏置值的合理性
 * @return 1: 偏置值合理, 0: 偏置值异常
 */
uint8_t check_offset_validity(void)
{
    // 检查偏置值是否在合理范围内 (12位ADC: 0-4095)
    const float MIN_OFFSET = 1500.0f;  // 最小合理偏置
    const float MAX_OFFSET = 2500.0f;  // 最大合理偏置
    
    if (VacOffset < MIN_OFFSET || VacOffset > MAX_OFFSET) {
        user_regulator_error("Voltage offset out of range: %.1f", VacOffset);
        return 0;
    }
    
    if (IacOffset < MIN_OFFSET || IacOffset > MAX_OFFSET) {
        user_regulator_error("Current offset out of range: %.1f", IacOffset);
        return 0;
    }
    
    user_regulator_info("Offset values are valid");
    return 1;
}

/**
 * @brief 示例：在特定条件下触发重新测量
 */
void conditional_remeasure_offsets(void)
{
    // 示例条件：检测到偏置值可能不准确
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每10秒检查一次
    if (current_time - last_check_time > 10000) {
        last_check_time = current_time;
        
        // 如果偏置值看起来不合理，且PWM关闭，则重新测量
        if (!check_offset_validity() && !pwm_enabled) {
            user_regulator_info("Triggering offset remeasurement due to invalid values");
            Reset_Offset_Measurement();
            // 注意：这里需要确保状态机会重新进入测量状态
        }
    }
}

/**
 * @brief 示例：保存偏置值到非易失性存储 (伪代码)
 */
void save_offsets_to_flash(void)
{
    if (Is_Offset_Measurement_Complete()) {
        // 这里是伪代码，实际实现需要根据具体的Flash操作函数
        /*
        typedef struct {
            float voltage_offset;
            float current_offset;
            uint32_t checksum;
        } offset_data_t;
        
        offset_data_t offset_data;
        offset_data.voltage_offset = VacOffset;
        offset_data.current_offset = IacOffset;
        offset_data.checksum = calculate_checksum(&offset_data);
        
        // 写入Flash
        flash_write(OFFSET_STORAGE_ADDRESS, &offset_data, sizeof(offset_data));
        */
        
        user_regulator_info("Offsets saved to flash (placeholder)");
    }
}

/**
 * @brief 示例：从非易失性存储加载偏置值 (伪代码)
 */
void load_offsets_from_flash(void)
{
    // 这里是伪代码，实际实现需要根据具体的Flash操作函数
    /*
    offset_data_t offset_data;
    
    // 从Flash读取
    if (flash_read(OFFSET_STORAGE_ADDRESS, &offset_data, sizeof(offset_data)) == SUCCESS) {
        // 验证校验和
        if (verify_checksum(&offset_data)) {
            VacOffset = offset_data.voltage_offset;
            IacOffset = offset_data.current_offset;
            
            // 标记为已完成测量，跳过自动测量
            offset_measurement_complete = 1;
            
            user_regulator_info("Offsets loaded from flash: V=%.1f, I=%.1f", 
                               VacOffset, IacOffset);
            return;
        }
    }
    */
    
    user_regulator_info("No valid offsets in flash, will measure automatically");
}

/**
 * @brief 示例：获取偏置测量的统计信息
 */
void get_offset_statistics(void)
{
    if (Is_Offset_Measurement_Complete()) {
        // 计算偏置值与理想值的偏差
        float voltage_deviation = VacOffset - DEFAULT_VAC_OFFSET;
        float current_deviation = IacOffset - DEFAULT_IAC_OFFSET;
        
        user_regulator_info("Offset Statistics:");
        user_regulator_info("  Voltage: %.1f (deviation: %+.1f)", VacOffset, voltage_deviation);
        user_regulator_info("  Current: %.1f (deviation: %+.1f)", IacOffset, current_deviation);
        
        // 评估偏置质量
        float max_deviation = fmaxf(fabsf(voltage_deviation), fabsf(current_deviation));
        if (max_deviation < 10.0f) {
            user_regulator_info("  Quality: Excellent (deviation < 10 LSB)");
        } else if (max_deviation < 50.0f) {
            user_regulator_info("  Quality: Good (deviation < 50 LSB)");
        } else {
            user_regulator_info("  Quality: Poor (deviation >= 50 LSB)");
        }
    }
}

/**
 * @brief 示例：在主循环中的使用方式
 */
void main_loop_example(void)
{
    // 系统初始化
    user_regulator_init();
    
    // 等待偏置测量完成
    while (!Is_Offset_Measurement_Complete()) {
        user_regulator_main();  // 继续执行主循环
        HAL_Delay(1);
    }
    
    // 检查偏置值有效性
    if (!check_offset_validity()) {
        user_regulator_error("Invalid offset values detected!");
        // 可以选择使用默认值或重新测量
    }
    
    // 显示测量统计信息
    get_offset_statistics();
    
    // 主循环
    while (1) {
        user_regulator_main();
        
        // 定期检查是否需要重新测量
        conditional_remeasure_offsets();
        
        HAL_Delay(1);
    }
}
