/* Point-based registration implemented via the method of "Least-Squares Fitting of Two 3-D Point Sets", Arun et al, 1987 */

#include <cstdlib>
#include <iostream>
#include <exception>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "Exceptions.cc"
#include "Util.cc"

Eigen::Matrix3d find_rotation(const Eigen::MatrixXd& H) {
    // Estimate rotation matrix given H, the matrix product of residuals in both pointsets.

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d rotation;

    auto proposed_rotation = svd.matrixV()*(svd.matrixU()).transpose();
    if(isApproxEqual(proposed_rotation.determinant(), 1)) {
        rotation = proposed_rotation;
    } else if(isApproxEqual(proposed_rotation.determinant(), -1)) {
        // Determinant of -1 can mean we've calculated a reflection (and so can compute a rotation) or we have insurmountable noise problems.
        auto lambda = svd.singularValues();
        if(isApproxEqual(lambda(2), 0) && !isApproxEqual(lambda(1), 0) && !isApproxEqual(lambda(0), 0)) { // This is a reflection.
            auto V_new = svd.matrixV();
            V_new.block(0,2,V_new.rows(),1) = -1 * V_new.block(0,2,V_new.rows(),1);
            rotation = V_new*(svd.matrixU()).transpose();
        } else {
            std::cerr << "Could not find a rotation. Colinear point cloud seems likely, or perhaps very noisy data?" << std::endl;
            throw(PointMatchingEx);
        }
    } else {
        std::cerr << "Could not find a rotation from SVD. Very noisy or otherwise invalid data?" << std::endl;
        throw(PointMatchingEx);
    }

    return rotation;
}

Eigen::Matrix4d estimate_rigid_transform(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    // Find a rigid transform that maps pointset to pointset_dash, with least error.

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

    auto p_average = find_pointset_average(pointset);
    auto p_dash_average = find_pointset_average(pointset_dash);

    auto q = residuals_from_point(pointset, p_average);
    auto q_dash = residuals_from_point(pointset_dash, p_dash_average);

    auto H = q * q_dash.transpose();

    auto rotation = find_rotation(H);

    auto translation = p_dash_average - rotation*p_average;

    auto final_transform = compose_final_transform(rotation, translation);

    return final_transform;
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
