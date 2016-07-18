#/*============================================================================
#
#  research-computing-with-cpp-demo: CMake based demo code.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

add_executable(runner runner.cc)
target_link_libraries(runner)

add_executable(PointMatchingTest PointMatchingTest.cc)
target_link_libraries(PointMatchingTest)

#add_executable(SurfaceMatchingTest SurfaceMatchingTest.cc)
#target_link_libraries(SurfaceMatchingTest)
