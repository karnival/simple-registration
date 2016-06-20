#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <PointMatching.cc>

int main(int argc, char** argv) {
    try {
        std::string appName = boost::filesystem::basename(argv[0]);

        namespace opts = boost::program_options;
        opts::options_description desc("Options");
        desc.add_options()
                ("help", "Print help message")
                ("data1", "First point cloud")
                ("data2", "Second point cloud")
                ("out", "Output filename")
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
          
       
      
    } catch(std::exception& e) {
        std::cerr << "Unhandled Exception reached the top of main: " 
                  << e.what() << ", application will now exit" << std::endl; 
        return 2;
    }
    return 0;
}
