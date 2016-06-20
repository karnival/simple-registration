/* Small utility functions useful for point-based registration */

bool isApproxEqual(double a, double b, double eps) {
    // Check whether a and b are equal, to within tolerance eps.
    auto diff = std::abs(a - b);
    return diff < eps;
}

bool isApproxEqual(double a, double b) {
    // Epsilon not specified, default to 0.001;
    return isApproxEqual(a, b, 0.001);
}

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset) {
    auto average = pointset.rowwise().mean();
    return average;
}

Eigen::MatrixXd residuals_from_point(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point) {
    return pointset.colwise() - point;
}

Eigen::VectorXd distances_between_pointsets(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    return (pointset - pointset_dash).colwise().norm();
}

double root_mean_square(const Eigen::VectorXd& v) {
    return sqrt((v.cwiseProduct(v)).mean());
}

Eigen::Matrix4d compose_final_transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    // Compose 3x3 rotation and 3d vector translation to get a joint rotation+translation.
    // Final transform is a 4x4 rigid transform matrix, with rotation part in the top-left 3x3, and translation in the top-right 3x1.
    Eigen::Matrix4d final_transform;
    final_transform.block(0,0,3,3) << rotation;
    final_transform.block(0,3,3,1) << translation;
    final_transform.block(3,0,1,3) << 0, 0, 0;
    final_transform.block(3,3,1,1) << 1;

    return final_transform;
}

