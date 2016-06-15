#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset) {
    auto average = pointset.rowwise().mean();
    return average;
}

Eigen::MatrixXd residuals_from_average(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point) {
    return pointset.colwise() - point;
}

void estimate_rigid_transform(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    auto p = find_pointset_average(pointset);
    auto p_dash = find_pointset_average(pointset_dash);

    auto q = residuals_from_average(pointset, p);
    auto q_dash = residuals_from_average(pointset_dash, p_dash);

    auto H = q * q_dash.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto proposed_rotation = svd.matrixV()*(svd.matrixU()).transpose();

    auto translation = p_dash - proposed_rotation*p;
}

int main(void) {
    Eigen::MatrixXd pointset(3,4);
    pointset << 1, 3, 1, 3,
             2, 2, 5, 5,
             3, 3, 3, 3;

    Eigen::MatrixXd pointset_dash(3,4);
    pointset_dash << 0.28, 1.70,-1.83,-0.41,
                  2.13, 3.54, 4.26, 5.66,
                  3.00, 3.00, 3.00, 3.01;
    
    estimate_rigid_transform(pointset, pointset_dash);
    return 0;
}
