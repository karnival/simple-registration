/* Point-based registration implemented via the method of "Least-Squares Fitting of Two 3-D Point Sets", Arun et al, 1987 */

#include <cstdlib>
#include <iostream>
#include <exception>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>

bool isApproxEqual(double a, double b, double eps) {
    auto diff = std::abs(a - b);
    return diff < eps;
}

class PointMatchingException : public std::exception {
    virtual const char* what() const throw() {
        return "Exception occurred in PointMatching.";
    }
} PointMatchingEx;

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
        std::cerr << "Not enough points provided -- there should be at least four points in the point cloud." << std::endl;
        throw(PointMatchingEx);
    }

    if(pointset.rows() != 3 || pointset_dash.rows() != 3) {
        std::cerr << "Points must be 3D." << std::endl;
        throw(PointMatchingEx);
    }

    if(pointset.cols() != pointset_dash.cols()) {
        std::cerr << "Pointsets must have the same number of points." << std::endl;
        throw(PointMatchingEx);
    }

    auto p = find_pointset_average(pointset);
    auto p_dash = find_pointset_average(pointset_dash);

    auto q = residuals_from_average(pointset, p);
    auto q_dash = residuals_from_average(pointset_dash, p_dash);

    auto H = q * q_dash.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d rotation;

    auto proposed_rotation = svd.matrixV()*(svd.matrixU()).transpose();
    if(isApproxEqual(proposed_rotation.determinant(), 1, 0.001)) {
        rotation = proposed_rotation;
    } else if(isApproxEqual(proposed_rotation.determinant(), -1, 0.001)) {
        // Determinant of -1 can mean we've calculated a reflection (and so can compute a rotation) or we have insurmountable noise problems.
        auto lambda = svd.singularValues();
        std::cout << "Lambda was set to " << std::endl << lambda << std::endl;
        std::cout << "Is approx zero? " << isApproxEqual(lambda(2), 0, 0.001) << std::endl;
        if(isApproxEqual(lambda(2), 0, 0.001)) { // This is a reflection.
            auto V_new = svd.matrixV();
            std::cout << "Try to set block." << std::endl;
            V_new.block(0,2,V_new.rows(),1) = -1 * V_new.block(0,2,V_new.rows(),1);
            rotation = V_new*(svd.matrixU()).transpose();
        } else {
            std::cerr << "Could not find a rotation. Overlapping points in point cloud, or very noisy data?" << std::endl;
            throw(PointMatchingEx);
        }
    } else {
        std::cerr << "Could not find a rotation. Overlapping points in point cloud, or very noisy data?" << std::endl;
        throw(PointMatchingEx);
    }

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
