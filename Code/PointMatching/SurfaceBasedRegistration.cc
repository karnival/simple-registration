#include <PointMatching.cc>

Eigen::ArrayXi find_closest_points(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2) {
    Eigen::ArrayXi lookup_table(surface1.cols());
    bool used[surface1.cols()];

    for(int i = 0; i < surface1.cols(); i++) {
        lookup_table(i) = 0;
        used[i] = false;
    }

    // For each point in the floating surface, find the closest point in the reference surface, then update lookup_table accordingly.
    for(int j = 0; j < surface1.cols(); j++) {
        auto v1 = surface1.col(j);
        double distance_old = 1E10; 

        for(int k = 0; k < surface2.cols(); k++) {
            if(used[k] == false) {
                auto v2 = surface2.col(k);
                auto distance_new = (v2 - v1).norm();
                if(distance_new < distance_old) {
                    used[lookup_table[j]] = false;
                    lookup_table[j] = k;
                    used[k] = true;
                    distance_old = distance_new;
                }
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

        error_new = fiducial_registration_error(surface1, closest_points, transform);
    } while(error_new < error);

    return transform_old;
}
