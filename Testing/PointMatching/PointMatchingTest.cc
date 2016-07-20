#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <SurfaceBasedRegistration.cc>

TEST_CASE( "can find pointset average", "[find_pointset_average]" ) {
    // Create an example pointset with a known average.
    Eigen::MatrixXd pointset(3,2);
    pointset << 1, 1, 
                2, 1,
                2, 4;

    Eigen::Vector3d expected_average;
    expected_average << 1, 1.5, 3;

    auto computed_average = find_pointset_average(pointset);
    REQUIRE( computed_average.isApprox(expected_average) );
}

TEST_CASE( "can find residuals between a pointset and a vector", "residuals_from_point" ) {
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

    auto calculated_residuals = residuals_from_point(pointset, example_vector);

    REQUIRE( calculated_residuals.isApprox(expected_residuals) );
}

TEST_CASE( "composition of rotation and translation works as expected ", "[compose_final_transform]") {
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

    auto calculated_composition = compose_final_transform(rotation, translation);

    REQUIRE( calculated_composition.isApprox(expected_composed, 0.01) );
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

    REQUIRE( estimated_transform.isApprox(expected_result, 0.01) );
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

TEST_CASE( "appropriate error handling when pointset of wrong dimensions is passed", "[estimate_rigid_transform]" ) {
    SECTION( "appropriate error handling when only one point is passed", "[estimate_rigid_transform]" ) {
        Eigen::MatrixXd pointset(3,1);
        pointset << 1,
                    2,
                    3;
    
        Eigen::MatrixXd pointset_dash(3,1);
        pointset_dash << -1,
                         -2,
                         -3;
    
        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException );
    }

    SECTION( "appropriate error handling when only two points are passed", "[estimate_rigid_transform]" ) {
        Eigen::MatrixXd pointset(3,2);
        pointset << 1, 2,
                    2, 3,
                    3, 4;
    
        Eigen::MatrixXd pointset_dash(3,2);
        pointset_dash << -1, 0,
                         -2,-1,
                         -3,-2;
    
        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException );
    }

    SECTION( "appropriate error handling when too few dimensions are passed", "[estimate_rigid_transform]" ) {
        Eigen::MatrixXd pointset(2,4);
        pointset << 1, 2, 3, 4,
                    2, 3, 4, 5;
    
        Eigen::MatrixXd pointset_dash(2,4);
        pointset_dash << -1, 0, 1, 2,
                         -2,-1, 0, 1;
    
        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException );
    }

    SECTION( "appropriate error handling when too many dimensions are passed", "[estimate_rigid_transform]" ) {
        Eigen::MatrixXd pointset(4,4);
        pointset << 1, 2, 3, 4,
                    2, 3, 4, 5,
                    1, 2, 3, 4,
                    1, 2, 6, 8;
    
        Eigen::MatrixXd pointset_dash(4,4);
        pointset_dash << -1,  0, 1, 2,
                         -2, -1, 0, 1,
                         -1,  0, 1, 2,
                         -1,  0, 2, 1;
    
        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException );
    }

    SECTION( "throws error when pointsets are not the same size" ) {
        Eigen::MatrixXd pointset(3,4);
        pointset << 1, 2, 3, 4,
                    2, 3, 4, 5,
                    1, 2, 3, 4;
    
        Eigen::MatrixXd pointset_dash(3,5);
        pointset_dash << -1,  0, 1, 2, 4,
                         -2, -1, 0, 1, 2,
                         -1,  0, 1, 2, 3;
    
        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException );

    }
}

TEST_CASE( "can handle coplanar/colinear" ) {
    SECTION( "colinear -- should fail with exception" ) {
        Eigen::MatrixXd pointset(3,4);
        pointset << 1, 2, 3, 1,
                    2, 3, 4, 2,
                    1, 2, 3, 1;

        Eigen::MatrixXd pointset_dash(3,4);
        pointset_dash << 1, 2, 3, 1,
                        -2,-3,-4,-2,
                         1, 2, 3, 1;

        REQUIRE_THROWS_AS( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash), PointMatchingException ) ;
    }

    SECTION( "coplanar -- should be able to find a rotation" ) {
        Eigen::MatrixXd pointset(3,4);
        pointset << 1, 2, 5, 1,
                    2, 3, 4, 2,
                    1, 2, 5, 1;

        Eigen::MatrixXd pointset_dash(3,4);
        pointset_dash << 1, 2, 5, 1,
                        -2,-3,-4,-2,
                         1, 2, 5, 1;

        REQUIRE_NOTHROW( auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash) );

        auto estimated_transform = estimate_rigid_transform(pointset, pointset_dash);
        Eigen::Matrix4d expected_transform;
        expected_transform << 0, 0, 1, 0,
                              0,-1, 0, 0,
                              1, 0, 0, 0,
                              0, 0, 0, 1;

        REQUIRE( estimated_transform.isApprox(expected_transform) );
    }
}

TEST_CASE( "can register two surfaces with no transformation, just reordering points", "[register_surfaces]" ) {
    Eigen::MatrixXd surface1(3,5);
    Eigen::MatrixXd surface2(3,5);

    surface1 << 1, 2, 5, 8, 8,
                1, 8, 2, 3, 9,
                2, 6, 2, 8, 9;

    surface2 << 2, 1, 5, 8, 8,
                8, 1, 2, 3, 9,
                6, 2, 2, 8, 9;

    Eigen::Matrix4d expected_result;
    expected_result <<  1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

    auto estimated_transform = register_surfaces(surface1, surface2);

    REQUIRE( estimated_transform.isApprox(expected_result, 0.01) );
}

TEST_CASE ("can match up the closest points", "[find_closest_points]" ) {
    Eigen::MatrixXd surface1(3,10);
    Eigen::MatrixXd surface2(3,10);

    surface1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

    surface2 << 2, 1, 3, 4, 6, 5, 8, 9, 7, 10,
                2, 1, 3, 4, 6, 5, 8, 9, 7, 10,
                2, 1, 3, 4, 6, 5, 8, 9, 7, 10;

    auto lookup_closest = find_closest_points(surface1, surface2);
    auto closest_points = reorder_points(surface2, lookup_closest);

    REQUIRE( closest_points.isApprox(surface1));

}

TEST_CASE( "can register two surfaces with a transformation between them", "[register_surfaces]" ) {
    Eigen::MatrixXd surface1(3,5);
    Eigen::MatrixXd surface2(3,5);

    surface1 << 1, 2, 5, 8, 8,
                1, 8, 2, 3, 9,
                2, 6, 2, 8, 9;

    surface2 << 7.07, 1.41,  4.94,  7.77, 12.0208,
                4.24, 0.00, -2.12, -3.53,  0.7071,
                6.00, 2.00,  2.00,  8.00,  9.0000;

    Eigen::Matrix4d expected_result;
    expected_result <<  0.71,-0.71, 0.00, 0.00,
                        0.71, 0.71, 0.00, 0.00,
                        0.00, 0.00, 1.00, 0.00,
                        0.00, 0.00, 0.00, 1.00;

    auto estimated_transform = register_surfaces(surface1, surface2, expected_result);

    std::cout << "Got " << estimated_transform << std::endl;

    REQUIRE( estimated_transform.isApprox(expected_result, 0.01) );
}

TEST_CASE( "can load a pointcloud from a file", "[load_pointcloud_from_file]" ) {
    SECTION( "successfully load pointcloud" ) {
        auto cloud = load_pointcloud_from_file("../Testing/PointBasedRegistrationData/moving.txt");
        REQUIRE( cloud(0,0) == Approx(192.8328) );
        REQUIRE( cloud(2,4) == Approx(-47.9328) );
    }

    SECTION( "throw exception when file does not exist" ) {
        REQUIRE_THROWS_AS( auto cloud = load_pointcloud_from_file("path does not exist"), PointMatchingException );
    }

    SECTION( "throw exception when file is not valid" ) {
        REQUIRE_THROWS_AS( auto cloud = load_pointcloud_from_file("../Testing/PointBasedRegistrationData/invalid_cloud.txt"), PointMatchingException );
    }
}

TEST_CASE( "point-based registration for test data" ) {
    auto data1 = "../Testing/PointBasedRegistrationData/moving.txt";
    auto data2 = "../Testing/PointBasedRegistrationData/fixed.txt";
    auto transform_file = "../Testing/PointBasedRegistrationData/matrix.4x4";

    auto cloud1 = load_pointcloud_from_file(data1);
    auto cloud2 = load_pointcloud_from_file(data2);

    auto expected_transform = load_transform_from_file(transform_file);
    auto estimated_transform = estimate_rigid_transform(cloud1, cloud2);

    REQUIRE( estimated_transform.isApprox(expected_transform, 0.01) );
}

TEST_CASE( "surface-based registration for test data" ) {
    auto data1 = "../Testing/SurfaceBasedRegistrationData/fran_cut.txt";
    auto data2 = "../Testing/SurfaceBasedRegistrationData/fran_cut_transformed.txt";
    auto transform_file = "../Testing/SurfaceBasedRegistrationData/matrix.4x4";

    auto surface1 = load_pointcloud_from_file(data1);
    auto surface2 = load_pointcloud_from_file(data2);

    auto expected_transform = load_transform_from_file(transform_file);
    auto estimated_transform = register_surfaces(surface1, surface2, expected_transform.inverse());

    REQUIRE( estimated_transform.isApprox(expected_transform.inverse(), 0.01) );
}
