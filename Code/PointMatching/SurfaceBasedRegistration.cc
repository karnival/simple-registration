#include <PointMatching.cc>

Eigen::Matrix4d register_surfaces(const Eigen::MatrixXd& surface1, const Eigen::MatrixXd& surface2) {
    int lookup_table[surface1.cols()];
    for(int i = 0; i < surface1.cols(); i++) {
        lookup_table[i] = i;
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
    
    // Now reorder the floating surface so corresponding points are in the same locations for floating and reference.
    auto surface3 = surface2;
    for(int i = 0; i < surface1.cols(); i++) {
        surface3.col(i) << surface2.col(lookup_table[i]);
    }

    auto transform = estimate_rigid_transform(surface1, surface3);
}
