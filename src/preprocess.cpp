/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:58:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-09 14:38:23
 * @FilePath: /lidarpoint/src/preprocess.cpp
 * @Description: 
 */
#include "preprocess.h"


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
    // viewer.addPointCloud<pcl::PointXYZI>(cloud_in, "cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    //  while (!viewer.wasStopped())
    // {
    //     viewer.spinOnce();
    // }


    return true;


}


void Preprocess::_3Dto2D()
{

    double fov_up = proj_fov_up / 180.0 * M_PI;      // field of view up in rad
    double fov_down = proj_fov_down / 180.0 * M_PI;  // field of view down in rad
    double fov = fabs(fov_down) + fabs(fov_up);  // get field of view total in rad

    auto cloud_size = cloud_in->size();
    std::vector<double> scan_x(cloud_size);
    std::vector<double> scan_y(cloud_size);
    std::vector<double> scan_z(cloud_size);
    std::vector<double> depth(cloud_size);
    std::vector<double> yaw(cloud_size);
    std::vector<double> pitch(cloud_size);
    std::vector<double> proj_x(cloud_size);
    std::vector<double> proj_y(cloud_size);

    PrintColorText(cloud_in->size(),TextColor::Blue);



    for (int i =0; i < cloud_size; ++i){
        depth[i] = CalculateRangeZXY(cloud_in->points[i]);
        scan_x[i] = cloud_in->points[i].x;
        scan_y[i] = cloud_in->points[i].y;
        scan_z[i] = cloud_in->points[i].z;
        yaw[i] = -std::atan2(scan_y[i], scan_x[i]);
        pitch[i] = std::asin(scan_z[i] / depth[i]);
        proj_x[i] = (0.5 * (yaw[i] / M_PI + 1.0))*proj_W;  // in [0.0, W]
        proj_y[i] = (1.0 - (pitch[i] + fabs(fov_down)) / fov)*proj_H; // in [0.0, H]
        

        proj_x[i] = std::floor(proj_x[i]);
        proj_x[i] = std::min(static_cast<double>(proj_W - 1), proj_x[i]);
        proj_x[i] = std::max(0.0, proj_x[i]);
        int32_t proj_x_int = static_cast<int32_t>(proj_x[i]); // in [0, W-1]

        proj_y[i] = std::floor(proj_y[i]);
        proj_y[i] = std::min(static_cast<double>(proj_H - 1), proj_y[i]);
        proj_y[i] = std::max(0.0, proj_y[i]);
        int32_t proj_y_int = static_cast<int32_t>(proj_y[i]);

        x_2d.push_back(proj_x_int);
        y_2d.push_back(proj_y_int);
        


    }

    auto unproj_range = depth;
    // order in decreasing depth
    // 根据深度信息进行从大到小的排序
    std::vector<int> indices(cloud_size);
    for (int i = 0; i < cloud_size; ++i) {
        indices[i] = i;
    }

    // 使用自定义比较函数，按照深度信息从大到小排序
    std::sort(indices.begin(), indices.end(), [&](int i, int j) {
        return depth[i] > depth[j];
    });


    // 根据排序结果重排其他存储变量
    std::vector<double> sorted_depth(cloud_size);
    std::vector<double> sorted_scan_x(cloud_size);
    std::vector<double> sorted_scan_y(cloud_size);
    std::vector<double> sorted_scan_z(cloud_size);
    std::vector<double> sorted_yaw(cloud_size);
    std::vector<double> sorted_pitch(cloud_size);
    std::vector<double> sorted_proj_x(cloud_size);
    std::vector<double> sorted_proj_y(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        int idx = indices[i];
        sorted_depth[i] = depth[idx];
        sorted_scan_x[i] = scan_x[idx];
        sorted_scan_y[i] = scan_y[idx];
        sorted_scan_z[i] = scan_z[idx];
        sorted_yaw[i] = yaw[idx];
        sorted_pitch[i] = pitch[idx];
        sorted_proj_x[i] = proj_x[idx];
        sorted_proj_y[i] = proj_y[idx];
    }


    cv::Mat depth_image(proj_H, proj_W, CV_64FC1, cv::Scalar(0));
    for (int i = 0; i < cloud_size; ++i) {
        int32_t proj_x_int = x_2d[i];
        int32_t proj_y_int = y_2d[i];
        double proj_depth = sorted_depth[i];

        if (proj_depth > 0 && proj_x_int >= 0 && proj_x_int < proj_W && proj_y_int >= 0 && proj_y_int < proj_H) {
            depth_image.at<double>(proj_y_int, proj_x_int) = proj_depth;
        }
    }

    // 可以将深度图像保存为图片文件
    cv::imwrite("depth_image.png", depth_image);



    

}

