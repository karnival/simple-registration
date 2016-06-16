/* Point-based registration implemented via the method of "Least-Squares Fitting of Two 3-D Point Sets", Arun et al, 1987 */

#include <cstdlib>
#include <iostream>
#include <exception>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>

class PointMatchingException : public std::exception {
    virtual const char* what() const throw() {
        return "Exception occurred in PointMatching.";
    }
} PointMatchingException;

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset) {
    auto average = pointset.rowwise().mean();
    return average;
}

Eigen::MatrixXd residuals_from_average(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point) {
    return pointset.colwise() - point;
}

Eigen::Matrix4d create_final_transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    // Final transform is a 4x4 rigid transform matrix, with rotation part in the top-left 3x3, and translation in the top-right 3x1.
    Eigen::Matrix4d final_transform;
    final_transform.block(0,0,3,3) << rotation;
    final_transform.block(0,3,3,1) << translation;
    final_transform.block(3,0,1,3) << 0, 0, 0;
    final_transform.block(3,3,1,1) << 1;

    return final_transform;
}

Eigen::Matrix4d estimate_rigid_transform(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    if(pointset.cols() < 4 || pointset_dash.cols() < 4) {
        throw(PointMatchingException);
    }

    if(pointset.rows() != 3 || pointset_dash.rows() != 3) {
        throw(PointMatchingException);
    }

    auto p = find_pointset_average(pointset);
    auto p_dash = find_pointset_average(pointset_dash);

    auto q = residuals_from_average(pointset, p);
    auto q_dash = residuals_from_average(pointset_dash, p_dash);

    auto H = q * q_dash.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto proposed_rotation = svd.matrixV()*(svd.matrixU()).transpose();

    auto translation = p_dash - proposed_rotation*p;

    auto final_transform = create_final_transform(proposed_rotation, translation);

    return final_transform;
}

Eigen::VectorXd distances_between_pointsets(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    return (pointset - pointset_dash).colwise().norm();
}

double root_mean_square(const Eigen::VectorXd& v) {
    return sqrt((v.cwiseProduct(v)).mean());
}

double fiducial_registration_error(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash, const Eigen::Matrix4d& transform) {
    // Need to add a one on the end of each vector, for the translation part of the transform.
    Eigen::MatrixXd pointset_augmented(4,pointset.cols());
    pointset_augmented.block(0,0,3,pointset.cols()) << pointset;
    pointset_augmented.block(3,0,1,pointset.cols()) << Eigen::MatrixXd::Constant(1, pointset.cols(), 1);

    auto proposed_pointset = transform * pointset_augmented;

    // Now remove the unnecessary bottom row of the transformed pointset.
    auto proposed_pointset_reduced = proposed_pointset.block(0,0,3,pointset.cols());

    auto error_per_vector = distances_between_pointsets(proposed_pointset_reduced, pointset_dash);
    auto fre = root_mean_square(error_per_vector);

    return fre;
}
