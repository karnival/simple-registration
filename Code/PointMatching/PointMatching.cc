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

    return 0;
}
