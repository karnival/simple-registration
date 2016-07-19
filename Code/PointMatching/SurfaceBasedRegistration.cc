#include <PointMatching.cc>

Eigen::ArrayXi find_closest_points(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2) {
    Eigen::ArrayXi lookup_table(surface1.cols());
    for(int i = 0; i < surface1.cols(); i++) {
        lookup_table(i) = i;
    }

    // For each point in the floating surface, find the closest point in the reference surface, then update lookup_table accordingly.
    // TODO: ensure that there is a one-to-one correspondence between points after updating lookup_table.
    for(int j = 0; j < surface1.cols(); j++) {
        auto v1 = surface1.col(j);
        auto distance_old = (surface2.col(lookup_table[j]) - surface1.col(j)).norm();

        for(int k = 0; k < surface2.cols(); k++) {
            auto v2 = surface2.col(k);
            auto distance_new = (v2 - v1).norm();
            if(distance_new < distance_old) {
                lookup_table[j] = k;
                distance_old = (surface2.col(lookup_table[j]) - surface1.col(j)).norm();
            }
        }
    }

    return lookup_table;
}

Eigen::MatrixXd reorder_points(const Eigen::MatrixXd& surface, const Eigen::ArrayXi& lookup_table) {
    auto reordered = surface;
    for(int i = 0; i < surface.cols(); i++) {
        reordered.col(i) << surface.col(lookup_table(i));
    }

    return reordered;
}

Eigen::Matrix4d register_surfaces(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2) {
    auto lookup_closest = find_closest_points(surface1, surface2);
    auto closest_points = reorder_points(surface2, lookup_closest);
    Eigen::Matrix4d transform;
    Eigen::Matrix4d transform_old;
    auto transformed_pointcloud = surface2;
    double error = 0;
    double error_new = fiducial_registration_error(surface1, transformed_pointcloud, transform);

     do {
        transform_old = transform;
        error = error_new;

        // Need to match up closest_points information (i.e. reordering) with untransformed surface2.
        transform = estimate_rigid_transform(surface1, closest_points);
        transformed_pointcloud = apply_transform(closest_points, transform);
        lookup_closest = find_closest_points(surface1, transformed_pointcloud);
        closest_points = reorder_points(surface2, lookup_closest);

        std::cout << "error before update is " << error_new << std::endl;
        error_new = fiducial_registration_error(surface1, closest_points, transform);
        std::cout << "error after update is " << error_new << std::endl;
    } while(error_new < error);

    return transform_old;
}
