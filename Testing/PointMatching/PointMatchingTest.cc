#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <PointMatching.cc>

TEST_CASE( "can estimate transform matrix", "[estimate_rigid_transform]" ) {
    // Example non-pathological pointset.
    Eigen::MatrixXd pointset(3,4);
    pointset << 1, 3, 1, 3,
                2, 2, 5, 5,
                3, 3, 3, 3;

    // Rotation by pi/4 about z-axis, translation by 1 on x-axis.
    Eigen::MatrixXd pointset_dash(3,4);
    pointset_dash << 3.10, 4.54, 5.26, 6.66,
                     0.72,-0.71, 2.84, 1.41,
                     3.01, 3.01, 3.01, 3.01;
    
    Eigen::Matrix4d expected_result;
    expected_result <<  0.70, 0.71, 0.00, 0.99,
                       -0.71, 0.70, 0.00, 0.02,
                        0.00, 0.00, 1.00, 0.01,
                        0.00, 0.00, 0.00, 1.00;
        
    auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash);

    REQUIRE(estimated_transform.isApprox(expected_result, 0.01));
}
