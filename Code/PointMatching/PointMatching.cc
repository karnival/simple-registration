#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset) {
    auto average = pointset.rowwise().mean();
    return average;
}

Eigen::MatrixXd residuals_from_average(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point) {
    return pointset.colwise() - point;
}

Eigen::Matrix4d estimate_rigid_transform(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    auto p = find_pointset_average(pointset);
    auto p_dash = find_pointset_average(pointset_dash);

    auto q = residuals_from_average(pointset, p);
    auto q_dash = residuals_from_average(pointset_dash, p_dash);

    auto H = q * q_dash.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto proposed_rotation = svd.matrixV()*(svd.matrixU()).transpose();

    auto translation = p_dash - proposed_rotation*p;

    Eigen::Matrix4d final_transform;
    final_transform.block(0,0,3,3) << proposed_rotation;
    final_transform.block(0,3,3,1) << translation;
    final_transform.block(3,0,1,3) << 0, 0, 0;
    final_transform.block(3,3,1,1) << 1;

    return final_transform;
}
