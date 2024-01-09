/*
 * @Author: wuyao wuyaosantu@qq.com
 * @Date: 2023-11-18 20:26:49
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-05 19:43:04
 * @FilePath: /TlsDeploy/inc/colorprint.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "colorprint.h"



template<>
void PrintColorText<bool>(const bool& value, TextColor color)
{
    std::string text = value ? "true" : "false";
    std::cout << "\033[" << static_cast<int>(color) << "m" << text << "\033[0m" << std::endl;
}
