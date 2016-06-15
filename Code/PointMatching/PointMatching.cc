#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

int main(void) {
    Eigen::MatrixXd p_set(3,4);
    p_set << 1, 3, 1, 3,
             2, 2, 5, 5,
             3, 3, 3, 3;

    Eigen::MatrixXd p_dash_set(3,4);
    p_dash_set << 0.28, 1.70,-1.83,-0.41,
                  2.13, 3.54, 4.26, 5.66,
                  3.00, 3.00, 3.00, 3.01;
    
    auto p = (p_set.rowwise().mean());
    auto p_dash = (p_dash_set.rowwise().mean());

    std::cout << "p is " << p << std::endl;
    std::cout << "p' is " << p_dash << std::endl;

    Eigen::Matrix3d H; 
    H.setZero();

    for(int i = 0; i < p_set.cols(); i++) {
        H += (p_set.col(i) - p) * (p_dash_set.col(i) - p_dash).transpose();
    }

    std::cout << "H is " << H << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Lambda is " << svd.singularValues() << std::endl;
    std::cout << "U is " << svd.matrixU() << std::endl;
    std::cout << "V is " << svd.matrixV() << std::endl;

    auto X = svd.matrixV()*(svd.matrixU()).transpose();

    std::cout << "X is " << X << std::endl;
    std::cout << "X's det is " << X.determinant() << std::endl;

    auto T = p_dash - X*p;

    std::cout << "T is " << T << std::endl;

    return 0;
}
