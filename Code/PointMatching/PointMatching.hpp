/* Point-based registration implemented via the method of "Least-Squares Fitting of Two 3-D Point Sets", Arun et al, 1987 */
#ifndef POINTMATCHING_INCLUDED
#define POINTMATCHING_INCLUDED

Eigen::Matrix3d find_rotation(const Eigen::MatrixXd& H);

Eigen::Matrix4d estimate_rigid_transform(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash);

double fiducial_registration_error(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash, const Eigen::Matrix4d& transform);
#endif
