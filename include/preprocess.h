/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 19:10:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-12 21:04:15
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
#include <Eigen/Dense>
#include <Eigen/Core>
#include <omp.h>

//=============include===================================================




// #define M_PI = 3.14159265358979
    // proj_x = v = cols = length of the Image = 1024
    // proj_y = u = rows = Number of Lasers of lidar = 64 
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
    std::vector<std::vector<std::vector<float>>> spherical_img_; 
    // pcl::PointCloud<pcl::PointXYZI>& _cloud_in;

    void Calculate( Eigen::MatrixXf xyzi,
                    const double& fov_rad,
                    const double& fov_down_rad,
                    Eigen::VectorXi* pixel_v, Eigen::VectorXi* pixel_u, Eigen::VectorXf* range);
    
    void PCL2Eigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, 
                        Eigen::MatrixXf* xyzi);
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
    void GetProjection( const pcl::PointXYZI& point,
                                const double& fov_rad,
                                const double& fov_down_rad,
                                int* pixel_v, int* pixel_u,
                                double* range
                                    );


};



// Preprocess::~Preprocess()
// {
// }









#endif // PREPROCESS_H_

