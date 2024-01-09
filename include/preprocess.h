/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 19:10:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-05 21:23:35
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
//=============include===================================================




#define pi = 3.14159265358979

class Preprocess
{
private:
    /* data */

    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_in;

public:
    Preprocess(/* args */);
    ~Preprocess()= default;

    int H = 64 ;
    int W = 1024 ;
    float fov_up = 3.0 ;
    float fov_down = -25.0;

    
    bool ReadPointCloud_Bin(const std::string& filename);
};



// Preprocess::~Preprocess()
// {
// }









#endif // PREPROCESS_H_

