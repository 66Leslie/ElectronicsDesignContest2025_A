//
// Created by 86195 on 25-7-14.
//

#include "key.h"

/* 按键GPIO配置表 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t active_level;  // 有效电平：0-低电平有效，1-高电平有效
} key_gpio_t;

/* 按键GPIO映射表 */
static const key_gpio_t key_gpio_table[KEY_NUM] = {
    {KEY1_GPIO_Port, KEY1_Pin, 0},      // KEY1: PE3,  低电平有效
    {KEY2_GPIO_Port, KEY2_Pin, 0},      // KEY2: PB9,  低电平有效
    {KEY3_GPIO_Port, KEY3_Pin, 0},      // KEY3: PF10, 低电平有效
    {KEY4_GPIO_Port, KEY4_Pin, 0},      // KEY4: PE1,  低电平有效
    {KEY5_GPIO_Port, KEY5_Pin, 0},      // KEY5: PE5,  低电平有效
};

/* 按键状态变量 */
static struct {
    uint8_t prev_state;         // 上次状态
    uint8_t current_state;      // 当前状态
    uint32_t change_time;       // 状态改变时间
    uint8_t processed;          // 是否已处理
} key_states[KEY_NUM];

/* 组合按键检测变量 */
static struct {
    uint8_t combo_detected;     // 组合按键检测标志
    uint32_t combo_start_time;  // 组合按键开始时间
    uint8_t combo_processed;    // 组合按键是否已处理
} combo_state;

/**
 * @brief 按键初始化
 */
void key_init(void)
{
    // 初始化按键状态
    for (int i = 0; i < KEY_NUM; i++) {
        key_states[i].prev_state = 0;
        key_states[i].current_state = 0;
        key_states[i].change_time = 0;
        key_states[i].processed = 0;
    }

    // 初始化组合按键状态
    combo_state.combo_detected = 0;
    combo_state.combo_start_time = 0;
    combo_state.combo_processed = 0;
}

/**
 * @brief 读取按键原始状态
 * @param key: 按键编号
 * @return: 1-按下，0-释放
 */
uint8_t key_read_raw(key_num_t key)
{
    if (key >= KEY_NUM) return 0;

    uint8_t gpio_state = HAL_GPIO_ReadPin(key_gpio_table[key].port, key_gpio_table[key].pin);

    // 根据有效电平返回逻辑状态
    if (key_gpio_table[key].active_level == 0) {
        return !gpio_state;  // 低电平有效，取反
    } else {
        return gpio_state;   // 高电平有效，直接返回
    }
}

/**
 * @brief 检测组合按键 (KEY4 + KEY5)
 * @return: 组合按键状态位 (bit4=KEY4, bit5=KEY5)
 */
uint8_t key_check_combo(void)
{
    uint8_t combo_keys = 0;

    // 检查KEY4和KEY5是否同时按下
    if (key_read_raw(KEY4)) {
        combo_keys |= (1 << 4);  // 设置bit4
    }
    if (key_read_raw(KEY5)) {
        combo_keys |= (1 << 5);  // 设置bit5
    }

    return combo_keys;
}

/**
 * @brief 按键扫描函数
 * @return: 按键扫描结果
 */
key_result_t key_scan(void)
{
    key_result_t result = {KEY1, KEY_NONE, 0};  // 默认返回无按键
    uint32_t current_time = HAL_GetTick();

    // 首先检测组合按键 (KEY4 + KEY5)
    uint8_t combo_keys = key_check_combo();
    if ((combo_keys & 0x30) == 0x30) {  // KEY4和KEY5都按下 (bit4和bit5都为1)
        if (!combo_state.combo_detected) {
            combo_state.combo_detected = 1;
            combo_state.combo_start_time = current_time;
            combo_state.combo_processed = 0;
        }

        // 防抖处理
        if (!combo_state.combo_processed &&
            (current_time - combo_state.combo_start_time) >= KEY_DEBOUNCE_TIME) {
            combo_state.combo_processed = 1;
            result.key_state = KEY_COMBO_PRESS;
            result.combo_keys = combo_keys;
            return result;
        }
    } else {
        combo_state.combo_detected = 0;
        combo_state.combo_processed = 0;
    }

    // 扫描单个按键
    for (int i = 0; i < KEY_NUM; i++) {
        uint8_t current_raw = key_read_raw((key_num_t)i);

        // 检测状态变化
        if (current_raw != key_states[i].prev_state) {
            key_states[i].change_time = current_time;
            key_states[i].prev_state = current_raw;
            key_states[i].processed = 0;
        }

        // 防抖处理
        if (!key_states[i].processed &&
            (current_time - key_states[i].change_time) >= KEY_DEBOUNCE_TIME) {

            key_states[i].processed = 1;

            // 检测按键按下事件
            if (current_raw && !key_states[i].current_state) {
                key_states[i].current_state = 1;
                result.key_num = (key_num_t)i;
                result.key_state = KEY_PRESS;
                result.combo_keys = 0;
                return result;  // 立即返回第一个检测到的按键
            }
            // 检测按键释放事件
            else if (!current_raw && key_states[i].current_state) {
                key_states[i].current_state = 0;
                result.key_num = (key_num_t)i;
                result.key_state = KEY_RELEASE;
                result.combo_keys = 0;
                return result;  // 立即返回第一个检测到的按键
            }
        }
    }

    return result;  // 无按键事件
}