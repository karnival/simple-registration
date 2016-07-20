#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <PointMatching.hpp>
#include <Util.hpp>

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
                ("out", opts::value<std::string> (&out), "Output filename.")
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

        auto transform = estimate_rigid_transform(cloud1, cloud2);

        if(vm.count("out")) {
            write_matrix_to_file(transform, out);
        }

        std::cout << "Estimated transform was " << std::endl << transform << std::endl;
    } catch(std::exception& e) {
        std::cerr << "Unhandled Exception reached the top of main: " 
                  << e.what() << ", application will now exit" << std::endl; 
        return 2;
    }
    return 0;
}
