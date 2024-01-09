/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:51:32
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-09 14:59:23
 * @FilePath: /lidarpoint/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

constexpr int proj_W = 1024;
constexpr int proj_H = 512;

// 计算点到原点的距离
double CalculateRange(const pcl::PointXYZ& point) {
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

// 将三维点转换为球坐标
void PointToSpherical(const pcl::PointXYZ& point, double& theta, double& phi) {
    theta = std::atan2(point.y, point.x);
    phi = std::acos(point.z / CalculateRange(point));
}

// 将球坐标转换为二维图像坐标
void SphericalToImage(const double& theta, const double& phi, int& u, int& v) {
    u = static_cast<int>((theta / (2 * M_PI) + 0.5) * proj_W);
    v = static_cast<int>((phi / M_PI + 0.5) * proj_H);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud.pcd>" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud_in) == -1) {
        std::cerr << "Failed to load point cloud file " << argv[1] << std::endl;
        return 1;
    }

    // 创建二维图像
    cv::Mat image(proj_H, proj_W, CV_8UC3, cv::Scalar(0, 0, 0));

    for (const auto& point : cloud_in->points) {
        double theta, phi;
        PointToSpherical(point, theta, phi);

        int u, v;
        SphericalToImage(theta, phi, u, v);

        if (u >= 0 && u < proj_W && v >= 0 && v < proj_H) {
            // 设置点的颜色为白色
            image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255);
        }
    }

    cv::imshow("Spherical Projection", image);
    cv::waitKey(0);

    return 0;
}
