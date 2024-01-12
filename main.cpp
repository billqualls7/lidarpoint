/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:51:32
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-12 10:20:53
 * @FilePath: /lidarpoint/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include "colorprint.h"
#include "preprocess.h"


int main(int, char**){
    // PrintColorText("Hello, from lidarpoint!\n", TextColor::Green);
    // std::cout << "Hello, from lidarpoint!\n";
    // std::string filename  = "/home/rqh/wycode/data/SemanticKITTI/dataset/sequences/01/velodyne/";
    // filename += "000111.bin";
    Preprocess preprocess_;
    std::string filename = "../data/000000.pcd";
    PrintColorText(filename,TextColor::Default);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    auto res  = preprocess_.ReadPointCloud_pcd(filename);
    PrintColorText(res, TextColor::LightGreen);
    // preprocess_._3Dto2D();
}

