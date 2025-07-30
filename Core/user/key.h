//
// Created by 86195 on 25-7-14.
//

#ifndef KEY_H
#define KEY_H

#include "main.h"

/* 按键相关宏定义 */
#define KEY_DEBOUNCE_TIME    150    // 按键防抖时间(ms)
#define KEY_NUM              5      // 按键总数

/* 按键枚举定义 */
typedef enum {
    KEY1 = 0,    // PE3  - 参数增加 (+)
    KEY2,        // PB9  - 参数减少 (-)
    KEY3,        // PF10 - PWM开启/关闭
    KEY4,        // PE1  - 暂时没有用处
    KEY5,        // PE5  - 模式切换(开环/CV/CC)
    KEY_BOOT     // PB8  - BOOT按键(特殊处理)
} key_num_t;

/* 按键状态枚举 */
typedef enum {
    KEY_NONE = 0,       // 无按键
    KEY_PRESS,          // 按键按下
    KEY_RELEASE,        // 按键释放
    KEY_LONG_PRESS,     // 长按(预留)
    KEY_COMBO_PRESS     // 组合按键按下
} key_state_t;

/* 按键扫描结果结构体 */
typedef struct {
    key_num_t key_num;      // 按键编号
    key_state_t key_state;  // 按键状态
    uint8_t combo_keys;     // 组合按键标志位 (bit4=KEY4, bit5=KEY5)
} key_result_t;

/* 函数声明 */
void key_init(void);                        // 按键初始化
key_result_t key_scan(void);                // 按键扫描
uint8_t key_read_raw(key_num_t key);        // 读取按键原始状态
uint8_t key_check_combo(void);              // 检测组合按键

#endif //KEY_H
