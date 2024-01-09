/*
 * @Author: wuyao wuyaosantu@qq.com
 * @Date: 2023-11-18 20:24:43
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-05 19:50:21
 * @FilePath: /TlsDeploy/inc/colorprint.h
 * @Description: 打印彩色字体
 */
// colorprint.h

#ifndef COLORPRINT_H
#define COLORPRINT_H

#include <iostream>
#include <string>

// 定义颜色的枚举类型
enum class TextColor {
    Default = 0,
    Black = 30,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan,
    LightGray,
    DarkGray = 90,
    LightRed,
    LightGreen,
    LightYellow,
    LightBlue,
    LightMagenta,
    LightCyan,
    White
};

// 声明打印带颜色的文本函数

template<typename T>
void PrintColorText(const T& value, TextColor color)
{
    std::cout << "\033[" << static_cast<int>(color) << "m" << value << "\033[0m" << std::endl;
}
template<>
void PrintColorText<bool>(const bool& value, TextColor color);

#endif // COLORPRINT_H
