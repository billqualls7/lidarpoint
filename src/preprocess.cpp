/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:58:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-05 21:29:57
 * @FilePath: /lidarpoint/src/preprocess.cpp
 * @Description: 
 */
#include "preprocess.h"


template<typename PointInT>
float CalculateRangeXY(const PointInT pointIn) {

	return sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y);
}

template<typename PointInT>
float CalculateRangeZXY(const PointInT pointIn) {

	// return sqrt(
	// 		pointIn.x * pointIn.x + pointIn.y * pointIn.y
	// 				+ (pointIn.z-2.1) * (pointIn.z-2.1));
    return sqrt(
        pointIn.x * pointIn.x + pointIn.y * pointIn.y
                + (pointIn.z) * (pointIn.z));
}


Preprocess::Preprocess(/* args */)
{
    // _cloud_in.reset();

}


bool Preprocess::ReadPointCloud_Bin(const std::string& filename)
{   
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile(filename, *cloud_in) == -1)
        {
        std::cerr << "Failed to read file: " << filename << std::endl;
        return false;
        }
    _cloud_in = cloud_in;
    std::cout << "Loaded " << _cloud_in->width * _cloud_in->height << " data points from file " << filename << std::endl;

    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZI>(_cloud_in, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

     while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }


    return true;


}


