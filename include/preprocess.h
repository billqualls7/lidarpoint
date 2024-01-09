/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 19:10:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-09 13:51:56
 * @FilePath: /lidarpoint/include/preprocess.h
 * @Description: 预处理
 */
#ifndef PREPROCESS_H_
#define PREPROCESS_H_

//=============include===================================================
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include "colorprint.h"
#include <algorithm>
#include <functional>
#include <vector>
//=============include===================================================




// #define M_PI = 3.14159265358979

class Preprocess
{
private:
    /* data */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in;

    int proj_H = 64 ;
    int proj_W = 1024 ;
    float proj_fov_up = 3.0 ;
    float proj_fov_down  = -25.0;

    std::vector<double> x_2d;
    std::vector<double> y_2d;
    // pcl::PointCloud<pcl::PointXYZI>& _cloud_in;

public:
    Preprocess(/* args */){
        x_2d.resize(proj_W);
        y_2d.resize(proj_H);
    }
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in;

    // Preprocess(/* args */);
    // ~Preprocess()= default;



    
    bool ReadPointCloud_pcd(const std::string& filename);
    void _3Dto2D();
};



// Preprocess::~Preprocess()
// {
// }









#endif // PREPROCESS_H_

