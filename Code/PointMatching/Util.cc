#include <cstdlib>
#include <iostream>
#include <fstream>
#include <exception>
/* Small utility functions useful for point-based registration */

bool isApproxEqual(double a, double b, double eps) {
    // Check whether a and b are equal, to within tolerance eps.
    auto diff = std::abs(a - b);
    return diff < eps;
}

bool isApproxEqual(double a, double b) {
    // Epsilon not specified, default to 0.001;
    return isApproxEqual(a, b, 0.001);
}

Eigen::Vector3d find_pointset_average(const Eigen::MatrixXd& pointset) {
    auto average = pointset.rowwise().mean();
    return average;
}

Eigen::MatrixXd residuals_from_point(const Eigen::MatrixXd& pointset, const Eigen::Vector3d& point) {
    return pointset.colwise() - point;
}

Eigen::VectorXd distances_between_pointsets(const Eigen::MatrixXd& pointset, const Eigen::MatrixXd& pointset_dash) {
    return (pointset - pointset_dash).colwise().norm();
}

double root_mean_square(const Eigen::VectorXd& v) {
    return sqrt((v.cwiseProduct(v)).mean());
}

Eigen::Matrix4d compose_final_transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    // Compose 3x3 rotation and 3d vector translation to get a joint rotation+translation.
    // Final transform is a 4x4 rigid transform matrix, with rotation part in the top-left 3x3, and translation in the top-right 3x1.
    Eigen::Matrix4d final_transform;
    final_transform.block(0,0,3,3) << rotation;
    final_transform.block(0,3,3,1) << translation;
    final_transform.block(3,0,1,3) << 0, 0, 0;
    final_transform.block(3,3,1,1) << 1;

    return final_transform;
}

Eigen::MatrixXd apply_transform(const Eigen::MatrixXd& pointset, const Eigen::Matrix4d& transform) {
    // Need to add a one on the end of each vector, for the translation part of the transform.
    Eigen::MatrixXd pointset_augmented(4,pointset.cols());
    pointset_augmented.block(0,0,3,pointset.cols()) << pointset;
    pointset_augmented.block(3,0,1,pointset.cols()) << Eigen::MatrixXd::Constant(1, pointset.cols(), 1);

    auto proposed_pointset = transform * pointset_augmented;

    // Now remove the unnecessary bottom row of the transformed pointset.
    auto proposed_pointset_reduced = proposed_pointset.block(0,0,3,pointset.cols());

    return proposed_pointset_reduced;
}

Eigen::MatrixXd load_pointcloud_from_file(std::string filename) {
    int max_points = 1E6;
    int line_counter = 0;
    Eigen::MatrixXd points(3,max_points);

    std::ifstream infile;
    infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
        infile.open(filename);
    
        double x, y, z;
    
        while(infile >> x >> y >> z) {
            points.col(line_counter) << x, y, z;
            line_counter++;
        }

    } catch(std::ifstream::failure e) {
        if(!infile.eof()) {
            // Any reason other than EOF is a failure. TODO: give a specific error message for non-existent file.
            std::cerr << "Could not read file " << filename << std::endl;
            throw(PointMatchingEx);
        } else{
            infile.close();

            auto pointcloud = points.block(0,0,3,line_counter);
            return pointcloud;
        }
    }

}

Eigen::Matrix4d load_transform_from_file(std::string filename) {
    int line_counter = 0;
    Eigen::Matrix4d transform;

    std::ifstream infile;
    infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
        infile.open(filename);
    
        double a, b, c, d;
    
        while(infile >> a >> b >> c >> d) {
            transform.row(line_counter) << a, b, c, d;
            line_counter++;
        }

    } catch(std::ifstream::failure e) {
        if(!infile.eof()) {
            // Any reason other than EOF is a failure. TODO: give a specific error message for non-existent file.
            std::cerr << "Could not read file " << filename << std::endl;
            throw(PointMatchingEx);
        } else if(infile.eof() && line_counter == 4) {
            infile.close();
            return transform;
        } else {
            std::cerr << "Could not read file " << filename << std::endl;
            throw(PointMatchingEx);
        }
    }

}

void write_matrix_to_file(const Eigen::MatrixXd& matrix, std::string filename) {
    std::ofstream outfile(filename);
    if(outfile.is_open()) {
        outfile << matrix;
    }
}
