/* Small utility functions useful for point-based registration */
#ifndef UTIL_INCLUDED 
#define UTIL_INCLUDED

#include <Eigen/Dense>

bool isApproxEqual(double a, double b, double eps);

bool isApproxEqual(double a, double b);

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset);

Eigen::MatrixXd residuals_from_point(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point);

Eigen::VectorXd distances_between_pointsets(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash);

double root_mean_square(const Eigen::VectorXd& v);

Eigen::Matrix4d compose_final_transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

Eigen::MatrixXd apply_transform(const Eigen::MatrixXd& pointset, const Eigen::Matrix4d& transform);

Eigen::MatrixXd load_pointcloud_from_file(std::string filename);

Eigen::Matrix4d load_transform_from_file(std::string filename);

void write_matrix_to_file(const Eigen::MatrixXd& matrix, std::string filename);
#endif
