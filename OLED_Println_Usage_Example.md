# OLED行显示功能使用说明

## 概述

新增的OLED行显示功能让你可以像使用串口打印 `printf` 一样，轻松地在OLED上一行一行地显示信息，无需手动计算Y坐标。

## 新增函数

### 1. `OLED_SetLine(uint8_t Line)`
设置下一次打印的起始行号。

**参数：**
- `Line`: 行号，范围：0-3 (对于16像素字体) 或 0-7 (对于8像素字体)

### 2. `OLED_ClearLine(uint8_t Line, uint8_t FontSize)`
清空指定行。

**参数：**
- `Line`: 要清空的行号
- `FontSize`: 用于计算行高的字体 (OLED_8X16 或 OLED_6X8)

### 3. `OLED_Println(uint8_t FontSize, const char *format, ...)`
在当前行打印格式化字符串，并自动换行。

**参数：**
- `FontSize`: 字体大小 (OLED_8X16 或 OLED_6X8)
- `format`: 格式化字符串
- `...`: 可变参数

**特性：**
- 打印后，行号会自动+1
- 如果超出屏幕范围，会自动清屏并从第0行开始
- 支持printf格式化字符串
- 自动清空当前行，防止字符重叠

## 使用示例

### 基本使用

```c
void Display_Example(void)
{
    // 设置从第0行开始打印
    OLED_SetLine(0);
    
    // 使用大字体打印
    OLED_Println(OLED_8X16, "Temperature: %.1f°C", 25.6f);
    OLED_Println(OLED_8X16, "Humidity: %.1f%%", 60.2f);
    
    // 使用小字体打印
    OLED_Println(OLED_6X8, "Status: Running");
    OLED_Println(OLED_6X8, "Time: %02d:%02d:%02d", 12, 34, 56);
    OLED_Println(OLED_6X8, "Press K1 to continue");
    
    OLED_Update(); // 更新显示
}
```

### 实际应用：手动模式页面

**修改前（需要手动计算Y坐标）：**
```c
void Display_Manual_Mode_Page_Old(void)
{
    OLED_Printf(0, 0, OLED_8X16, "Mod: %.1f%%", modulation_ratio * 100.0f);
    OLED_Printf(0, 16, OLED_8X16, "AB: %.1fV", ac_voltage_rms);
    OLED_Printf(0, 32, OLED_6X8, "A: %.2fA", ac_current_rms);
    OLED_Printf(0, 40, OLED_6X8, "BC: %.1fV", dc_voltage);
    OLED_Printf(0, 48, OLED_6X8, "B: %.2fA", dc_current);
    OLED_Printf(0, 56, OLED_6X8, "K1:+ K2:- K3:PWM K5:Page");
}
```

**修改后（使用新的行显示功能）：**
```c
void Display_Manual_Mode_Page(void)
{
    // 设置从第0行开始
    OLED_SetLine(0);
    
    // 大字体显示主要信息
    OLED_Println(OLED_8X16, "Mod: %.1f%%", modulation_ratio * 100.0f);
    OLED_Println(OLED_8X16, "AB: %.1fV", ac_voltage_rms);
    
    // 小字体显示详细信息
    OLED_Println(OLED_6X8, "A: %.2fA", ac_current_rms);
    OLED_Println(OLED_6X8, "BC: %.1fV", dc_voltage);
    OLED_Println(OLED_6X8, "B: %.2fA", dc_current);
    OLED_Println(OLED_6X8, "K1:+ K2:- K3:PWM K5:Page");
}
```

### 动态信息显示

```c
void Display_Dynamic_Info(void)
{
    static uint32_t counter = 0;
    
    OLED_SetLine(0);
    
    OLED_Println(OLED_8X16, "System Monitor");
    OLED_Println(OLED_6X8, "Counter: %lu", counter++);
    OLED_Println(OLED_6X8, "CPU: %.1f%%", get_cpu_usage());
    OLED_Println(OLED_6X8, "Memory: %d KB", get_free_memory());
    OLED_Println(OLED_6X8, "Uptime: %02d:%02d:%02d", 
                 get_uptime_hours(), get_uptime_minutes(), get_uptime_seconds());
    
    // 当行数超过屏幕容量时，会自动清屏并从头开始
    OLED_Println(OLED_6X8, "Auto scroll demo");
    OLED_Println(OLED_6X8, "Line 7");
    OLED_Println(OLED_6X8, "Line 8 - will clear screen");
    
    OLED_Update();
}
```

## 注意事项

1. **字体混用**: 如果在同一个显示函数中混用不同字体，建议在切换字体后使用 `OLED_SetLine()` 手动校准行号。

2. **屏幕容量**:
   - OLED_8X16字体：最多4行 (64像素 ÷ 16像素/行 = 4行)
   - OLED_6X8字体：最多8行 (64像素 ÷ 8像素/行 = 8行)

3. **自动清屏**: 当打印的行数超过屏幕容量时，函数会自动调用 `OLED_Clear()` 并重置行号为0。

4. **缓冲区大小**: 格式化字符串的缓冲区大小为128字节，足够显示一行内容。

## 优势

- **简化代码**: 无需手动计算Y坐标
- **自动换行**: 行号自动递增
- **防止重叠**: 自动清空当前行
- **格式化支持**: 完全支持printf格式化字符串
- **自动滚屏**: 超出屏幕时自动清屏重新开始
- **易于维护**: 代码更加简洁和易读

这个功能让OLED显示代码变得像串口打印一样简单易用！
