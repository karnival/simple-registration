#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

int main(void) {
    Eigen::MatrixXd p_set(4,3);
    p_set << 1, 2, 3,
             3, 2, 3,
             1, 5, 3,
             3, 5, 3;

    Eigen::MatrixXd p_dash_set(4,3);
    p_dash_set << 0.28, 2.13, 3.00,
                  1.70, 3.54, 3.00,
                 -1.83, 4.26, 3.00,
                 -0.41, 5.66, 3.01;

    auto p = p_set.colwise().mean();
    auto p_dash = p_dash_set.colwise().mean();

    std::cout << "p is " << p << std::endl;
    std::cout << "p' is " << p_dash << std::endl;

    Eigen::Matrix3d H; 
    H.setZero();

    for(int i = 0; i < p_set.rows(); i++) {
        H += (p_set.row(i) - p).transpose() * (p_dash_set.row(i) - p_dash);
    }

    std::cout << "H is " << H << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Lambda is " << svd.singularValues() << std::endl;
    std::cout << "U is " << svd.matrixU() << std::endl;
    std::cout << "V is " << svd.matrixV() << std::endl;

    auto X = svd.matrixV()*(svd.matrixU()).transpose();

    std::cout << "X is " << X << std::endl;
    std::cout << "X's det is " << X.determinant() << std::endl;

    return 0;
}
