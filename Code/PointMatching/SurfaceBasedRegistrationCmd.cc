#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <SurfaceBasedRegistration.hpp>

#include <Util.hpp>

int main(int argc, char** argv) {
    try {
        std::string appName = boost::filesystem::basename(argv[0]);

        std::string data1;
        std::string data2;
        std::string out;

        std::string init_file;

        namespace opts = boost::program_options;
        opts::options_description desc("Options");
        desc.add_options()
                ("help", "Print help message")
                ("data1", opts::value<std::string> (&data1)->required(), "First point cloud filename.")
                ("data2", opts::value<std::string> (&data2)->required(), "Second point cloud filename.")
                ("out", opts::value<std::string> (&out), "Output filename.")
                ("init_file", opts::value<std::string> (&init_file), "Filename for transformation initialisation matrix (4x4).")
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
      
        Eigen::Matrix4d init_matrix;
        if(vm.count("init_file")) {
            std::cout << "init_file was " << vm["init_file"].as<std::string>() << std::endl;
            init_matrix = load_transform_from_file(init_file);
            std::cout << "Transform initialised as " << std::endl << init_matrix << std::endl;
        }
      
        Eigen::MatrixXd pointcloud1;
        Eigen::MatrixXd pointcloud2;

        auto cloud1 = load_pointcloud_from_file(data1);
        auto cloud2 = load_pointcloud_from_file(data2);


        Eigen::Matrix4d transform;
        if(vm.count("init_file")) {
            transform = register_surfaces(cloud1, cloud2, init_matrix.inverse());
        } else {
            transform = register_surfaces(cloud1, cloud2);
        }

        if(vm.count("out")) {
            write_matrix_to_file(transform.inverse(), out);
        }

        std::cout << "Estimated transform was " << std::endl << transform.inverse() << std::endl;
    } catch(std::exception& e) {
        std::cerr << "Unhandled Exception reached the top of main: " 
                  << e.what() << ", application will now exit" << std::endl; 
        return 2;
    }
    return 0;
}
