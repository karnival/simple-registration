#include <PointMatching.cc>

int main(void) {
    Eigen::MatrixXd surface1(3,4);
    Eigen::MatrixXd surface2(3,4);

    surface1 << 1, 2, 5, 8,
                1, 8, 2, 3,
                2, 6, 2, 8;

    surface2 << 2, 1, 5, 8,
                8, 1, 2, 3,
                6, 2, 2, 8;

    int lookup_table[surface1.cols()];
    for(int i = 0; i < surface1.cols(); i++) {
        lookup_table[i] = i;
    }

    for(int i = 0; i < 100; i++) {
        // For each point in the floating surface, find the closest point in the reference surface, then update lookup_table accordingly.
        for(int j = 0; j < surface1.cols(); j++) {
            auto v1 = surface1.col(j);
            auto distance_old = (surface2.col(lookup_table[j]) - surface1.col(j)).norm();

            for(int k = 0; k < surface2.cols(); k++) {
                auto v2 = surface2.col(k);
                auto distance_new = (v2 - v1).norm();
                if(distance_new < distance_old) {
                    std::cout << "Updated with i = " << i << std::endl;
                    lookup_table[j] = k;
                    distance_old = (surface2.col(lookup_table[j]) - surface1.col(j)).norm();
                }
            }
        }
    }
    
    std::cout << lookup_table[0] << " " << lookup_table[1] << " " << lookup_table[2] << " " << lookup_table [3] << std::endl;

    // Now reorder the floating surface so corresponding points are in the same locations for floating and reference.
    auto surface3 = surface2;
    for(int i = 0; i < surface1.cols(); i++) {
        surface3.col(i) << surface2.col(lookup_table[i]);
    }

    std::cout << "End matrix is " << surface3 << std::endl;
    std::cout << estimate_rigid_transform(surface1, surface3) << std::endl;

}
