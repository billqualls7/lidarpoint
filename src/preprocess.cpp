/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:58:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-11 16:45:05
 * @FilePath: /lidarpoint/src/preprocess.cpp
 * @Description: 
 */
#include "preprocess.h"
    // proj_x = v = cols = length of the Image = 1024
    // proj_y = u = rows = Number of Lasers of lidar = 64 

template<typename PointInT>
double CalculateRangeXY(const PointInT pointIn) {

	return sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y);
}

template<typename PointInT>
double CalculateRangeZXY(const PointInT pointIn) {

	// return sqrt(
	// 		pointIn.x * pointIn.x + pointIn.y * pointIn.y
	// 				+ (pointIn.z-2.1) * (pointIn.z-2.1));
    return sqrt(
        pointIn.x * pointIn.x + pointIn.y * pointIn.y
                + (pointIn.z) * (pointIn.z));
}




bool Preprocess::ReadPointCloud_pcd(const std::string& filename)
{   
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (pcl::io::loadPCDFile(filename, *cloud_in) == -1)
        {
        std::cerr << "Failed to read file: " << filename << std::endl;
        return false;
        }
    // _cloud_in = *cloud_in;
    
    // std::cout << "Loaded " << cloud_in->width * cloud_in->height << " data points from file " << filename << std::endl;
    // 可视化
    // pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    // viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(cloud_in, "intensity");

    // viewer.addPointCloud<pcl::PointXYZI>(cloud_in, color_handler, "cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    // while (!viewer.wasStopped())
    // {
    //     viewer.spinOnce();
    // }


    return true;


}

void Preprocess::GetProjection( const pcl::PointXYZI& point,
                                const double& fov_rad,
                                const double& fov_down_rad,
                                int* pixel_v, int* pixel_u,
                                double* range
                                    )

{

    *range = CalculateRangeZXY(point);
    auto yaw = -std::atan2(point.y, point.x);
    auto pitch = std::asin(point.z / *range);

    // proj_x = v = cols = length of the Image = 1024
    // proj_y = u = rows = Number of Lasers of lidar = 64 

    double proj_x = (0.5 * (yaw / M_PI + 1.0))*proj_W;  // in [0.0, W]   v
    double proj_y = (1.0 - (pitch + fabs(fov_down_rad)) / fov_rad)*proj_H; // in [0.0, H]  u

    proj_x = std::floor(proj_x);
    proj_x = std::min(static_cast<double>(proj_W - 1), proj_x);
    proj_x = std::max(0.0, proj_x);
    *pixel_v = int(proj_x);

    proj_y = std::floor(proj_y);
    proj_y = std::min(static_cast<double>(proj_H - 1), proj_y);
    proj_y = std::max(0.0, proj_y);
    *pixel_u = int(proj_y);
}


/**
 * @description: 球形投影 输入xyz 
 * @return {*}
 */

void Preprocess::_3Dto2D()
{
    double fov_up = proj_fov_up / 180.0 * M_PI;      // field of view up in rad
    double fov_down = proj_fov_down / 180.0 * M_PI;  // field of view down in rad
    double fov = fabs(fov_down) + fabs(fov_up);  // get field of view total in rad

    spherical_img_.resize(proj_H, std::vector<std::vector<double>>((cloud_in->width * cloud_in->height),std::vector<double>(5, 0.0)));
    double count = 0;
    for (auto point : *cloud_in)
    {
        int pixel_v = 0;
        int pixel_u = 0;
        double range = 0.0;
        GetProjection(point, fov, fov_down, &pixel_v, &pixel_u, &range);

        spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{
            point.x, point.y, point.z, range, point.intensity};
        
        count++;
    }
    auto unsort_proj = spherical_img_;
    PrintColorText(count,TextColor::White);
    std::sort(std::begin(spherical_img_), std::end(spherical_img_), [](std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b){
    return a[0][3] > b[0][3];
});


    cv::Mat sp_img(proj_H, proj_W, CV_64FC1);
    for (int i = 0; i < sp_img.rows; ++i) {
        for (int j = 0; j < sp_img.cols; ++j) {
            sp_img.at<double>(i, j) = spherical_img_.at(i).at(j).at(4);  // Intensity value
            // sp_img.at<double>(i, j) = spherical_img_.at(i).at(j).at(3);  // range value
        }
    }
    cv::imshow("Intensity Image", sp_img);
    cv::waitKey(0);
    // cv::Mat sp_img(spherical_img_.at(0).size(), spherical_img_.size(), CV_64FC1);
    // for (int i = 0; i < sp_img.rows; ++i) {
    //     for (int j = 0; j < sp_img.cols; ++j) {
    //         sp_img.at<double>(i, j) = spherical_img_.at(j).at(i).at(4); // Intensity value
    //     }
    // }

    



}



