#ifndef SURFACEBASEDREGISTRATION_INCLUDED
#define SURFACEBASEDREGISTRATION_INCLUDED

#include <Eigen/Dense>

Eigen::ArrayXi find_closest_points(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2);

Eigen::MatrixXd reorder_points(const Eigen::MatrixXd& surface, const Eigen::ArrayXi& lookup_table);

Eigen::Matrix4d register_surfaces(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2, const Eigen::Matrix4d& transform_init);

Eigen::Matrix4d register_surfaces(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2);
#endif
