#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <SurfaceBasedRegistration.cc>


Eigen::MatrixXd load_pointcloud_from_file(std::string filename) {
    int max_points = 10240;
    int line_counter = 0;
    Eigen::MatrixXd points(3,max_points);

    std::ifstream infile;
    infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
        infile.open(filename);

        double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    
        while(infile >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3) {
            points.col(line_counter) << x1, y1, z1;
            points.col(line_counter+1) << x2, y2, z2;
            points.col(line_counter+2) << x3, y3, z3;
            line_counter += 3;
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

int main(int argc, char** argv) {
    try {
        std::string appName = boost::filesystem::basename(argv[0]);

        std::string data1;
        std::string data2;
        std::string out;

        namespace opts = boost::program_options;
        opts::options_description desc("Options");
        desc.add_options()
                ("help", "Print help message")
                ("data1", opts::value<std::string> (&data1)->required(), "First point cloud filename.")
                ("data2", opts::value<std::string> (&data2)->required(), "Second point cloud filename.")
                ("out", opts::value<std::string> (&out)->required(), "Output filename.")
        ;

        opts::positional_options_description positionalOptions;

        opts::variables_map vm;

        try {
            opts::store(opts::command_line_parser(argc, argv).options(desc)
                            .positional(positionalOptions).run(),
                        vm);
            if(vm.count("help")) {
                std::cout << "Program description." << std::endl;
                return 0;
            }

            opts::notify(vm);
        } catch(opts::required_option& e) {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 

            return 1;
        } catch(boost::program_options::error& e) { 
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;  

            return 1;                                  
        }                                                                

        if(vm.count("data1")) {
            std::cout << "data1 was " << vm["data1"].as<std::string>() << std::endl;
        }
          
        if(vm.count("data2")) {
            std::cout << "data2 was " << vm["data2"].as<std::string>() << std::endl;
        }
       
        if(vm.count("out")) {
            std::cout << "out was " << vm["out"].as<std::string>() << std::endl;
        }
      
        Eigen::MatrixXd pointcloud1;
        Eigen::MatrixXd pointcloud2;

        auto cloud1 = load_pointcloud_from_file(data1);
        auto cloud2 = load_pointcloud_from_file(data2);

        auto transform = register_surfaces(cloud1, cloud2);
        std::cout << "Estimated transform was " << std::endl << transform << std::endl;
    } catch(std::exception& e) {
        std::cerr << "Unhandled Exception reached the top of main: " 
                  << e.what() << ", application will now exit" << std::endl; 
        return 2;
    }
    return 0;
}