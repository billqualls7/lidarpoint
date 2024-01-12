/*
 * @Author: wuyao sss
 * @Date: 2024-01-05 18:58:33
 * @LastEditors: wuyao sss
 * @LastEditTime: 2024-01-12 22:14:22
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


static double gtm() {
    struct timeval tm;
    gettimeofday(&tm, 0);
    // 返回秒加上微秒的小数部分
    double re = tm.tv_sec + (double) tm.tv_usec / 1000000.0;
    return re;
}


void sort_vec(const Eigen::MatrixXf& mtrx, Eigen::MatrixXf& sorted_mtrx, Eigen::VectorXi& ind)
{
    ind = Eigen::VectorXi::LinSpaced(mtrx.rows(), 0, mtrx.rows() - 1);

    auto rule = [&mtrx](int i, int j) -> bool {
        return mtrx(i, mtrx.cols() - 1) > mtrx(j, mtrx.cols() - 1); // 比较最后一列的元素大小
    };

    std::sort(ind.data(), ind.data() + ind.size(), rule);

    sorted_mtrx.resize(mtrx.rows(), mtrx.cols());
    for (int i = 0; i < mtrx.rows(); i++) {
        sorted_mtrx.row(i) = mtrx.row(ind(i));
    }
}





bool Preprocess::ReadPointCloud_pcd(const std::string& filename)
{   
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    // Eigen::MatrixXf xyz, intensity;
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // spherical_img_.resize(proj_H, std::vector<std::vector<double>>((cloud_in->width * cloud_in->height),std::vector<double>(5, 0.0)));
   
    if (pcl::io::loadPCDFile(filename, *cloud_in) == -1)
    {
    std::cerr << "Failed to read file: " << filename << std::endl;
    return false;
    }

    

    double fov_up = proj_fov_up / 180.0 * M_PI;      // field of view up in rad
    double fov_down = proj_fov_down / 180.0 * M_PI;  // field of view down in rad
    double fov = fabs(fov_down) + fabs(fov_up);  // get field of view total in rad

    // std::vector<std::vector<Eigen::VectorXf>> spherical_img_;

    auto tm0 = gtm();
    const auto cloud_in_Size = cloud_in->size();
    const pcl::PointXYZI* data_ptr = cloud_in->points.data();
    Eigen::MatrixXf pointCloudMatrix = Eigen::Map<Eigen::MatrixXf>(cloud_in->getMatrixXfMap().data(), cloud_in->size(), 4 );    // spherical_img_.resize(proj_H, std::vector<std::vector<float>>((cloud_in_Size),std::vector<float>(5, 0.0)));
    // std::cout << "Loaded " << cloud_in->width * cloud_in->height << " data points from file " << filename << std::endl;

    Eigen::VectorXf range(cloud_in_Size, 1);

    // Eigen::MatrixXf xyzir(cloud_in_Size, 5);
    Eigen::MatrixXf sorted_xyzir;
    Eigen::VectorXi ind;

    Eigen::Map<Eigen::MatrixXf> xyzir(pointCloudMatrix.data(), pointCloudMatrix.rows(), pointCloudMatrix.cols() + 1);
    xyzir.col(pointCloudMatrix.cols()-1) << range;

    // Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm(range.size());
    // perm.setIdentity();
    // std::sort(perm.indices().data(), perm.indices().data() + perm.indices().size(),
    //         [&](int i, int j) { return range(i) > range(j); });
    // xyzir = xyzir * perm;

    // PCL2Eigen(cloud_in, &xyzi);

    // Eigen::VectorXf range = xyz.rowwise().norm();

    Eigen::VectorXi pixel_v;
    Eigen::VectorXi pixel_u;
    Calculate(pointCloudMatrix, fov, fov_down, &pixel_v, &pixel_u, &range);
    // xyzir << pointCloudMatrix, range;

    // std::cout << "pointCloudMatrix matrix has " << pointCloudMatrix.rows() << " rows and " << pointCloudMatrix.cols() << " columns" << std::endl;
    // std::cout << "range matrix has " << range.rows() << " rows and " << range.cols() << " columns" << std::endl;
    // std::cout << "Combined matrix has " << xyzir.rows() << " rows and " << xyzir.cols() << " columns" << std::endl;
    // std::cout << "pixel_v matrix has " << pixel_v.rows() << " rows and " << pixel_v.cols() << " columns" << std::endl;



    // sort_vec(xyzir, sorted_xyzir, ind);
    auto tm1 = gtm();
    PrintColorText((tm1-tm0), TextColor::LightYellow);
    
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


void Preprocess::Calculate( Eigen::MatrixXf xyzi,
                            const double& fov_rad,
                            const double& fov_down_rad,
                            Eigen::VectorXi* pixel_v, 
                            Eigen::VectorXi* pixel_u,
                            Eigen::VectorXf* range)
{
    PrintColorText("Calculate", TextColor::LightGreen);
    auto tm0 = gtm();
    const int num_points = xyzi.rows();
    const float k_yaw = 0.5 * proj_W / M_PI;
    const float b_yaw = 0.5 * proj_W;
    const float k_pitch = proj_H / fov_rad;
    const float b_pitch = proj_H / fov_rad * (1.0 + fabs(fov_down_rad));


    range->resize(num_points);
    pixel_v->resize(num_points);
    pixel_u->resize(num_points);

    auto tm1 = gtm();
    PrintColorText((tm1-tm0), TextColor::LightRed);
    // *range = xyzi.leftCols(3).rowwise().norm();
    const Eigen::Map<const Eigen::ArrayXf> x(xyzi.col(0).data(), xyzi.rows());
    const Eigen::Map<const Eigen::ArrayXf> y(xyzi.col(1).data(), xyzi.rows());
    const Eigen::Map<const Eigen::ArrayXf> z(xyzi.col(2).data(), xyzi.rows());
    const Eigen::Map<const Eigen::ArrayXf> i(xyzi.col(3).data(), xyzi.rows());
    *range = (x.square() + y.square() + z.square()).sqrt();
    
    auto tm2 = gtm();
    PrintColorText((tm2-tm1), TextColor::LightGreen);
    const Eigen::ArrayXf yaw = -(y.array() / x.array()).atan();
    const Eigen::ArrayXf pitch = -((z.array() / range->array()).asin());
    // Eigen::ArrayXf proj_x = (0.5 * (yaw / M_PI + 1.0)) * proj_W;
    // Eigen::ArrayXf proj_y = (1.0 - (pitch + fabs(fov_down_rad)) / fov_rad) * proj_H;
    Eigen::ArrayXf proj_x = k_yaw * yaw + b_yaw;
    Eigen::ArrayXf proj_y = k_pitch * yaw + b_pitch;
    auto tm3 = gtm();
    PrintColorText((tm3-tm2), TextColor::White);
    
    proj_x = proj_x.floor().min(static_cast<float>(proj_W - 1)).max(0.0);
    proj_y = proj_y.floor().min(static_cast<float>(proj_H - 1)).max(0.0);
    // std::cout << "proj_x matrix has " << proj_x.rows() << " rows and " << proj_x.cols() << " columns" << std::endl;
    auto tm4 = gtm();
    PrintColorText((tm4-tm3), TextColor::LightGreen);

    *pixel_v = proj_x.cast<int>();
    *pixel_u = proj_y.cast<int>();
    auto tm5 = gtm();
    PrintColorText((tm5-tm4), TextColor::LightGreen);
    PrintColorText((tm5-tm0), TextColor::LightBlue);

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

// void Preprocess::_3Dto2D()
// {
//     double fov_up = proj_fov_up / 180.0 * M_PI;      // field of view up in rad
//     double fov_down = proj_fov_down / 180.0 * M_PI;  // field of view down in rad
//     double fov = fabs(fov_down) + fabs(fov_up);  // get field of view total in rad

//     // spherical_img_.resize(proj_H, std::vector<std::vector<double>>((cloud_in->width * cloud_in->height),std::vector<double>(5, 0.0)));
//     // double count = 0;



//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
//     // std::vector<std::vector<int>> point_index_map(spherical_img_.size(), std::vector<int>(spherical_img_[0].size(), -1));
//     int index = 0;

//     auto tm0 = gtm();
//     // for (auto point : *cloud_in)
//     // {   
//     //     // auto tm0 = gtm();
//     //     int pixel_v = 0;
//     //     int pixel_u = 0;
//     //     double range = 0.0;
//     //     GetProjection(point, fov, fov_down, &pixel_v, &pixel_u, &range);

//     //     spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{
//     //         point.x, point.y, point.z, range, point.intensity};

//     //     point_index_map[pixel_u][pixel_v] = index;
//     //     index++;
//     //     // count++;

//     // }


//     #pragma omp parallel for
// for (size_t i = 0; i < cloud_in->size(); ++i) {
//     auto point = (*cloud_in)[i];

//     int pixel_v = 0;
//     int pixel_u = 0;
//     double range = 0.0;
//     GetProjection(point, fov, fov_down, &pixel_v, &pixel_u, &range);

//     #pragma omp critical
//     {
//         spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{
//             point.x, point.y, point.z, range, point.intensity};

//         point_index_map[pixel_u][pixel_v] = i;
//     }
// }




//     auto unsort_proj = spherical_img_;
    
//     auto tm1 = gtm();
//     PrintColorText((tm1-tm0), TextColor::LightGreen);

//     std::sort(std::begin(spherical_img_), std::end(spherical_img_), [](std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b){
//     return a[0][3] > b[0][3];
// });





//     for (int u = 0; u < spherical_img_.size(); ++u) {
//     for (int v = 0; v < spherical_img_[u].size(); ++v) {
//         double range = spherical_img_[u][v][3];
//         if (range < 1e-6) continue;

//         // 获取点的索引
//         int point_index = point_index_map[u][v];

//         // 如果索引为 -1，则表示该位置没有点的信息
//         if (point_index == -1) continue;

//         // 从哈希表中获取点的坐标和强度信息
//         pcl::PointXYZI p = cloud_in->at(point_index);

//         pcl::PointXYZI new_point;
//         new_point.x = p.x;
//         new_point.y = p.y;
//         new_point.z = p.z;
//         new_point.intensity = p.intensity;

//         cloud_out->push_back(new_point);
//         }
//     }   

//     int64_t tm2 = gtm();

//     // PrintColorText((tm2-tm1), TextColor::LightGreen);
//     // pcl::visualization::PCLVisualizer viewer("PCD Viewer");
//     // viewer.setBackgroundColor(0.0, 0.0, 0.0);
//     // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(cloud_out, "intensity");

//     // viewer.addPointCloud<pcl::PointXYZI>(cloud_out, color_handler, "cloud");
//     // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

//     // while (!viewer.wasStopped())
//     // {
//     //     viewer.spinOnce();
//     // }


//     cv::Mat sp_img(proj_H, proj_W, CV_64FC1);
//     for (int i = 0; i < sp_img.rows; ++i) {
//         for (int j = 0; j < sp_img.cols; ++j) {
//             sp_img.at<double>(i, j) = spherical_img_.at(i).at(j).at(4);  // Intensity value
//             // sp_img.at<double>(i, j) = spherical_img_.at(i).at(j).at(3);  // range value
//         }
//     }
//     cv::imshow("Intensity Image", sp_img);
//     cv::waitKey(0);
//     // cv::Mat sp_img(spherical_img_.at(0).size(), spherical_img_.size(), CV_64FC1);
//     // for (int i = 0; i < sp_img.rows; ++i) {
//     //     for (int j = 0; j < sp_img.cols; ++j) {
//     //         sp_img.at<double>(i, j) = spherical_img_.at(j).at(i).at(4); // Intensity value
//     //     }
//     // }

    



// }







// void Preprocess::PCL2Eigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, 
//                             Eigen::MatrixXf* xyz, Eigen::MatrixXf* intensity)
// {
//     auto tm0 = gtm();
//     Eigen::MatrixXf matrix_xyz(points_in->size(), 3);
//     Eigen::MatrixXf matrix_i(points_in->size(), 1);

//     for(int i = 0; i<points_in->size(); ++i)
//     {
//         matrix_xyz(i, 0) = points_in->points[i].x;
//         matrix_xyz(i, 1) = points_in->points[i].y;
//         matrix_xyz(i, 2) = points_in->points[i].z;
//         matrix_i(i, 0) = points_in->points[i].intensity;
//     }

//     *xyz = matrix_xyz;
//     *intensity = matrix_i;
//     auto tm1 = gtm();
//     PrintColorText((tm1-tm0), TextColor::LightGreen);

// }



// void Preprocess::PCL2Eigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, 
//                             Eigen::MatrixXf* xyz, Eigen::MatrixXf* intensity)
// {
//     auto tm0 = gtm();
    
//     // 获取点云数据的指针和大小
//     const pcl::PointXYZI* data_ptr = points_in->points.data();
//     const std::size_t cloud_size = points_in->size();

//     // 使用 Eigen 库的 Map 功能，直接将点云数据映射到 Eigen::MatrixXf 类型的矩阵中
//     Eigen::Map<Eigen::MatrixXf> matrix_xyz(reinterpret_cast<float*>(xyz->data()), cloud_size, 3);
//     Eigen::Map<Eigen::MatrixXf> matrix_i(reinterpret_cast<float*>(intensity->data()), cloud_size, 1);
    
//     // 将点云数据复制到 Eigen 矩阵中
//     for (std::size_t i = 0; i < cloud_size; ++i)
//     {
//         matrix_xyz.row(i) << data_ptr[i].x, data_ptr[i].y, data_ptr[i].z;
//         matrix_i(i, 0) = data_ptr[i].intensity;
//     }

//     auto tm1 = gtm();
//     PrintColorText((tm1-tm0), TextColor::LightGreen);
// }






void Preprocess::PCL2Eigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, 
                            Eigen::MatrixXf* xyzi)
{
    auto tm0 = gtm();
    
    // 获取点云数据的指针和大小
    const pcl::PointXYZI* data_ptr = points_in->points.data();
    const std::size_t cloud_size = points_in->size();

    // 使用 Eigen 库的 Map 功能，直接将点云数据映射到 Eigen::MatrixXf 类型的矩阵中
    Eigen::Map<Eigen::MatrixXf> matrix_xyz(reinterpret_cast<float*>(xyzi->data()), cloud_size, 4);
    
    // 将点云数据复制到 Eigen 矩阵中
    for (std::size_t i = 0; i < cloud_size; ++i)
    {
        matrix_xyz.row(i) << data_ptr[i].x, data_ptr[i].y, data_ptr[i].z,data_ptr[i].intensity;
    }

    auto tm1 = gtm();
    PrintColorText((tm1-tm0), TextColor::LightGreen);
}









// void Preprocess::PCL2Eigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, 
//                             std::vector<Eigen::Vector4d>* point_cloud)
// {
//     auto tm0 = gtm();
    
//     // 获取点云数据的指针和大小
//     const pcl::PointXYZI* data_ptr = points_in->points.data();
//     const std::size_t cloud_size = points_in->size();

//     // 清空输出的点云数据容器
//     point_cloud->clear();
//     // 设置输出的点云数据容器大小
//     point_cloud->resize(cloud_size);

//     // 将PCL的PointXYZI点云数据转换为Eigen::Vector4d
//     for (std::size_t i = 0; i < cloud_size; ++i)
//     {
//         (*point_cloud)[i] << data_ptr[i].x, data_ptr[i].y, data_ptr[i].z, data_ptr[i].intensity;
//     }

//     auto tm1 = gtm();
//     PrintColorText((tm1-tm0), TextColor::LightGreen);
// }