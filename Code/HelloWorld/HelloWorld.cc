/*============================================================================

  research-computing-with-cpp-demo: CMake based demo code.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

============================================================================*/

#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <itkImage.h>

int main(int argc, char** argv)
{
  // Check Eigen is correctly included.
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  // Check Boost is correctly included.
  std::cout << "The number 10.0 converted to string is " << boost::lexical_cast<std::string>(10.0) << std::endl;

  // Check Boost libraries are correctly included.
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
  ;

  // Check ITK is correctly included.
  typedef itk::Image<int, 2> ImageType;
  ImageType::Pointer myImage = ImageType::New();
  std::cout << "Image has region " << myImage->GetLargestPossibleRegion() << std::endl;

  return EXIT_SUCCESS;
}
