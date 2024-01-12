#include <iostream>
#include <Eigen/Dense>
#include <vector> 
int main() {
    Eigen::MatrixXd matrix(4, 3);
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9,
              2, 4, 6;
    
    // 获取矩阵的行数和列数
    int m = matrix.rows();
    int n = matrix.cols();
    
    // 定义一个以最后一列元素为关键字的排序函数
    auto sort_func = [&matrix, n](int i, int j) {
        return matrix(i, n-1) > matrix(j, n-1);
    };
    
    // 对矩阵的行进行排序
    std::vector<int> indices(m);
    for (int i = 0; i < m; ++i) {
        indices[i] = i;
    }
    std::sort(indices.begin(), indices.end(), sort_func);
    
    // 根据排序结果重新排列矩阵
    Eigen::MatrixXd sorted_matrix(m, n);
    for (int i = 0; i < m; ++i) {
        sorted_matrix.row(i) = matrix.row(indices[i]);
    }
    
    std::cout << sorted_matrix << std::endl;
    
    return 0;
}
