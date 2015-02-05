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
if(DEFINED ITK_DIR AND NOT EXISTS ${ITK_DIR})
  message(FATAL_ERROR "ITK_DIR variable is defined but corresponds to non-existing directory \"${ITK_DIR}\".")
endif()

set(version "4.7.1-2015-01-30-0e40f3f")
set(location "${RCCPP_EP_TARBALL_LOCATION}/InsightToolkit-${version}.tar.gz")

rccppMacroDefineExternalProjectVariables(ITK ${version} ${location})
set(proj_DEPENDENCIES)

if(NOT DEFINED ITK_DIR)

  set(additional_cmake_args )
  if(MINGW)
    set(additional_cmake_args
        -DCMAKE_USE_WIN32_THREADS:BOOL=ON
        -DCMAKE_USE_PTHREADS:BOOL=OFF)
  endif()

  list(APPEND additional_cmake_args
       -DUSE_WRAP_ITK:BOOL=OFF
      )

  # Keep the behaviour of ITK 4.3 which by default turned on ITK Review
  # see MITK bug #17338
  list(APPEND additional_cmake_args
    -DModule_ITKReview:BOOL=ON
  )

  ExternalProject_Add(${proj}
    PREFIX ${proj_CONFIG}
    SOURCE_DIR ${proj_SOURCE}
    BINARY_DIR ${proj_BUILD}
    INSTALL_DIR ${proj_INSTALL}
    URL ${proj_LOCATION}
    URL_MD5 ${proj_CHECKSUM}
    CMAKE_GENERATOR ${GEN}
    CMAKE_ARGS
      ${EP_COMMON_ARGS}
      ${additional_cmake_args}
      -DCMAKE_INSTALL_PREFIX:PATH=${proj_INSTALL}
      -DBUILD_TESTING:BOOL=${EP_BUILD_TESTING}
      -DBUILD_EXAMPLES:BOOL=${EP_BUILD_EXAMPLES}
      -DBUILD_SHARED_LIBS:BOOL=${EP_BUILD_SHARED_LIBS}
    DEPENDS ${proj_DEPENDENCIES}
  )

  set(ITK_DIR ${proj_INSTALL}/lib/cmake/ITK-4.7)
  message("SuperBuild loading ITK from ${ITK_DIR}")

else()

  mitkMacroEmptyExternalProject(${proj} "${proj_DEPENDENCIES}")

endif()
