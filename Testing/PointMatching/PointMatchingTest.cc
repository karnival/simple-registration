#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <PointMatching.cc>

TEST_CASE( "can find pointset average", "[find_pointset_average]" ) {
    // Create an example pointset with a known average.
    Eigen::MatrixXd pointset(3,2);
    pointset << 1, 1, 
                2, 1,
                2, 4;

    Eigen::Vector3d expected_average;
    expected_average << 1, 1.5, 3;

    auto computed_average = find_pointset_average(pointset);
    REQUIRE(computed_average.isApprox(expected_average));
}

TEST_CASE( "can find residuals between a pointset and a vector", "residuals_from_average" ) {
    // Create an example pointset and vector with known residuals.
    Eigen::MatrixXd pointset(3,2);
    pointset << 1, 2,
                3, 4,
                5, 6;

    Eigen::Vector3d example_vector;
    example_vector << 1, 3, 2;

    auto expected_residuals = pointset;
    expected_residuals << 0, 1,
                          0, 1,
                          3, 4;

    auto calculated_residuals = residuals_from_average(pointset, example_vector);

    REQUIRE(calculated_residuals.isApprox(expected_residuals));
}

TEST_CASE( "composition of rotation and translation works as expected ", "[create_final_transform]") {
    Eigen::Matrix3d rotation;
    rotation << 0.71, 0.71, 0.00,
               -0.71, 0.71, 0.00,
                0.00, 0.00, 1.00;

    Eigen::Vector3d translation;
    translation << 1, 2, 3;

    Eigen::Matrix4d expected_composed;
    expected_composed << 0.71, 0.71, 0.00, 1.00,
                        -0.71, 0.71, 0.00, 2.00,
                         0.00, 0.00, 1.00, 3.00,
                         0.00, 0.00, 0.00, 1.00;

    auto calculated_composition = create_final_transform(rotation, translation);

    REQUIRE(calculated_composition.isApprox(expected_composed, 0.01));
}

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

TEST_CASE( "can calculate fiducial registration error", "[fiducial_registration_error]" ) {
    Eigen::MatrixXd pointset(3,2);
    pointset << 1, 2,
                2, 3,
                2, 4;

    // Original pointset has been translated by one in each dimension.
    Eigen::MatrixXd pointset_dash(3,2);
    pointset_dash << 2, 3,
                     3, 4,
                     3, 5;

    SECTION( "distance between pointsets is correct" ) {
        Eigen::Vector2d expected_distances;
        expected_distances << 1.73, 1.73;

        auto calculated_distances = distances_between_pointsets(pointset, pointset_dash);

        REQUIRE( calculated_distances.isApprox(expected_distances, 0.01) );
    }

    SECTION( "fiducial registration error is approximately zero for the true transform" ) {
        Eigen::Matrix4d true_transform;
        true_transform << 1, 0, 0, 1,
                          0, 1, 0, 1,
                          0, 0, 1, 1,
                          0, 0, 0, 1;

        auto fre = fiducial_registration_error(pointset, pointset_dash, true_transform);
        REQUIRE( fre == Approx( 0.0 ) );
    }

    SECTION( "fiducial registration error is correct for a false transform" ) {
        Eigen::Matrix4d false_transform;
        false_transform << 1, 0, 0, 2,
                           0, 1, 0, 1,
                           0, 0, 1, 1,
                           0, 0, 0, 1;

        auto fre = fiducial_registration_error(pointset, pointset_dash, false_transform);
        REQUIRE( fre == Approx( 1.0 ) );
    }
}
