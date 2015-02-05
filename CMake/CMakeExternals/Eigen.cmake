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

# Sanity checks
if(DEFINED Eigen_DIR AND NOT EXISTS ${Eigen_DIR})
  message(FATAL_ERROR "Eigen_DIR variable is defined but corresponds to non-existing directory \"${Eigen_ROOT}\".")
endif()

set(version "3.2.2.1")
set(location "${RCCPP_EP_TARBALL_LOCATION}/eigen-eigen-${version}.tar.bz2")

rccppMacroDefineExternalProjectVariables(Eigen ${version} ${location})
set(proj_DEPENDENCIES)

if(NOT DEFINED Eigen_DIR)

  ExternalProject_Add(${proj}
    PREFIX ${proj_CONFIG}
    SOURCE_DIR ${proj_SOURCE}
    BINARY_DIR ${proj_BUILD}
    INSTALL_DIR ${proj_INSTALL}
    URL ${proj_LOCATION}
    URL_MD5 ${proj_CHECKSUM}
    #CONFIGURE_COMMAND ""
    UPDATE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    CMAKE_ARGS
      ${EP_COMMON_ARGS}
      -DCMAKE_INSTALL_PREFIX:PATH=${proj_INSTALL}
      -DEIGEN_LEAVE_TEST_IN_ALL_TARGET=ON
      -DBUILD_SHARED_LIBS:BOOL=${EP_BUILD_SHARED_LIBS}
    DEPENDS ${proj_DEPENDENCIES}
  )

  set(Eigen_DIR ${proj_SOURCE})
  set(Eigen_ROOT ${Eigen_DIR})
  set(Eigen_INCLUDE_DIR ${Eigen_DIR})

  message("SuperBuild loading Eigen from ${Eigen_DIR}")

else(NOT DEFINED Eigen_DIR)

  mitkMacroEmptyExternalProject(${proj} "${proj_DEPENDENCIES}")

endif(NOT DEFINED Eigen_DIR)
