//
// Created by AI Assistant
// 调试配置文件 - 统一管理所有调试输出
//

#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

#include "stdio.h"  // 需要printf函数

// ============================================================================
// 调试输出总开关
// ============================================================================
#define GLOBAL_DEBUG_ENABLE 1  // 设置为1启用调试输出以诊断问题

// ============================================================================
// 各模块调试开关 - 可以单独控制每个模块的调试输出
// ============================================================================
#if GLOBAL_DEBUG_ENABLE

#define USER_MAIN_DEBUG 1           // 主程序调试
#define USER_REGULATOR_DEBUG 1      // 用户调节器调试
#define THREE_PHASE_PWM_DEBUG 1     // 三相PWM调试
#define PLL_DEBUG 1                 // PLL调试

#else

// 全局调试关闭时，所有模块调试都关闭
#define USER_MAIN_DEBUG 0
#define USER_REGULATOR_DEBUG 0
#define THREE_PHASE_PWM_DEBUG 0
#define PLL_DEBUG 0

#endif

// ============================================================================
// 调试级别控制
// ============================================================================
#define DEBUG_LEVEL_ERROR   1   // 错误信息
#define DEBUG_LEVEL_INFO    2   // 一般信息
#define DEBUG_LEVEL_DEBUG   3   // 详细调试信息

// 设置当前调试级别（只输出小于等于此级别的信息）
#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_INFO

// ============================================================================
// 调试输出宏定义 - 格式化调试信息
// ============================================================================

// 主程序调试宏
#ifdef USER_MAIN_DEBUG

#define user_main_printf(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define user_main_info(format, ...) printf("[\tmain]info:" format "\r\n", ##__VA_ARGS__)
#define user_main_debug(format, ...) printf("[\tmain]debug:" format "\r\n", ##__VA_ARGS__)
#define user_main_error(format, ...) printf("[\tmain]error:" format "\r\n", ##__VA_ARGS__)

#else

#define user_main_printf(format, ...)
#define user_main_info(format, ...)
#define user_main_debug(format, ...)
#define user_main_error(format, ...)

#endif

// 用户调节器模块调试宏
#ifdef USER_REGULATOR_DEBUG

#define user_regulator_printf(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define user_regulator_info(format, ...) printf("[\tregulator]info:" format "\r\n", ##__VA_ARGS__)
#define user_regulator_debug(format, ...) printf("[\tregulator]debug:" format "\r\n", ##__VA_ARGS__)
#define user_regulator_error(format, ...) printf("[\tregulator]error:" format "\r\n", ##__VA_ARGS__)

#else

#define user_regulator_printf(format, ...)
#define user_regulator_info(format, ...)
#define user_regulator_debug(format, ...)
#define user_regulator_error(format, ...)

#endif

// 三相PWM调试宏
#ifdef THREE_PHASE_PWM_DEBUG

#define three_phase_printf(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define three_phase_info(format, ...) printf("[\t3phase]info:" format "\r\n", ##__VA_ARGS__)
#define three_phase_debug(format, ...) printf("[\t3phase]debug:" format "\r\n", ##__VA_ARGS__)
#define three_phase_error(format, ...) printf("[\t3phase]error:" format "\r\n", ##__VA_ARGS__)

#else

#define three_phase_printf(format, ...)
#define three_phase_info(format, ...)
#define three_phase_debug(format, ...)
#define three_phase_error(format, ...)

#endif

// PLL调试宏
#ifdef PLL_DEBUG

#define pll_printf(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define pll_info(format, ...) printf("[\tpll]info:" format "\r\n", ##__VA_ARGS__)
#define pll_debug(format, ...) printf("[\tpll]debug:" format "\r\n", ##__VA_ARGS__)
#define pll_error(format, ...) printf("[\tpll]error:" format "\r\n", ##__VA_ARGS__)

#else

#define pll_printf(format, ...)
#define pll_info(format, ...)
#define pll_debug(format, ...)
#define pll_error(format, ...)

#endif

// ============================================================================
// 兼容性定义 - 保持现有代码工作
// ============================================================================
#define DEBUG_PRINTF_ENABLE USER_REGULATOR_DEBUG
#define DEBUG_PLL_ONLY 0        // 显示所有调试信息
#define PID_DEBUG_PRINTF USER_REGULATOR_DEBUG

// ============================================================================
// 使用示例和说明
// ============================================================================
/*
 * 使用示例：
 *
 * 1. 控制调试输出：
 *    #define THREE_PHASE_PWM_DEBUG 1  // 启用三相PWM调试
 *    #define THREE_PHASE_PWM_DEBUG 0  // 禁用三相PWM调试
 *
 * 2. 在代码中使用：
 *    three_phase_info("三相PWM已启动");
 *    three_phase_debug("调制比: %.1f%%", ratio * 100.0f);
 *    three_phase_error("PWM初始化失败");
 *
 * 3. 输出格式：
 *    [	3phase]info:三相PWM已启动
 *    [	3phase]debug:调制比: 50.0%
 *    [	3phase]error:PWM初始化失败
 *
 * 4. 快速禁用所有调试：
 *    #define GLOBAL_DEBUG_ENABLE 0
 */

#endif // DEBUG_CONFIG_H
