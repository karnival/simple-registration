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

######################################################################
# Set the minimum CMake version.
######################################################################
if(WIN32)
  cmake_minimum_required(VERSION 3.1)
else()
  cmake_minimum_required(VERSION 2.8.9)
endif()


##################################################################################
# Set some CMake Policies.
# See http://cmake.org/cmake/help/cmake-2-8-docs.html#section_Policies for details
##################################################################################
set(project_policies
  CMP0001 # NEW: CMAKE_BACKWARDS_COMPATIBILITY should no longer be used.
  CMP0002 # NEW: Logical target names must be globally unique.
  CMP0003 # NEW: Libraries linked via full path no longer produce linker search paths.
  CMP0004 # NEW: Libraries linked may NOT have leading or trailing whitespace.
  CMP0005 # NEW: Preprocessor definition values are now escaped automatically.
  CMP0006 # NEW: Installing MACOSX_BUNDLE targets requires a BUNDLE DESTINATION.
  CMP0007 # NEW: List command no longer ignores empty elements.
  CMP0008 # NEW: Libraries linked by full-path must have a valid library file name.
  CMP0009 # NEW: FILE GLOB_RECURSE calls should not follow symlinks by default.
  CMP0010 # NEW: Bad variable reference syntax is an error.
  CMP0011 # NEW: Included scripts do automatic cmake_policy PUSH and POP.
  CMP0012 # NEW: if() recognizes numbers and boolean constants.
  CMP0013 # NEW: Duplicate binary directories are not allowed.
  CMP0014 # NEW: Input directories must have CMakeLists.txt
  )
foreach(policy ${project_policies})
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endforeach()

if (NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
  # CMP0048 # OLD: Leave VERSION variables untouched.
  cmake_policy(SET CMP0048 OLD)
endif ()

if (NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
  # CMP0053 # OLD: Leave references and variable expansion as pre version 3.1.
  cmake_policy(SET CMP0053 OLD)
endif ()


######################################################################
# We have a super-build option to download Eigen, Boost, ITK.
######################################################################
option(BUILD_SUPERBUILD "Build RCCPP and the projects it depends on via SuperBuild.cmake." ON)
mark_as_advanced(BUILD_SUPERBUILD)
if(BUILD_SUPERBUILD)
  project(RCCPP-SUPERBUILD)
else(BUILD_SUPERBUILD)
  project(RCCPP)
endif(BUILD_SUPERBUILD)
set(RCCPP_EP_TARBALL_LOCATION "http://cmic.cs.ucl.ac.uk/platform/dependencies" CACHE STRING "Location of external project tarballs" FORCE)


######################################################################
# We manually set version numbers. See CMP0048.
######################################################################
set(RCCPP_VERSION_MAJOR 0 CACHE STRING "Major version number" FORCE )
set(RCCPP_VERSION_MINOR 0 CACHE STRING "Minor version number" FORCE )
set(RCCPP_VERSION_PATCH 1 CACHE STRING "Patch version number" FORCE )
mark_as_advanced(RCCPP_VERSION_MAJOR)
mark_as_advanced(RCCPP_VERSION_MINOR)
mark_as_advanced(RCCPP_VERSION_PATCH)


######################################################################
# Make sure Git is available.
######################################################################
find_package(Git REQUIRED)
if (WIN32)
  set(GITCOMMAND ${GIT_EXECUTABLE})
endif()


######################################################################
# Setup the path to load CMake macros, and extra CMake files.
######################################################################
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindEigen.cmake ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY)
set(CMAKE_MODULE_PATH
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/CMake
    ${CMAKE_SOURCE_DIR}/CMake/CMakeExternals
    ${CMAKE_MODULE_PATH}
   )


######################################################################
# Add in any functions/macros.
######################################################################


######################################################################
# Now set up flags for this project.
######################################################################
option(BUILD_SHARED_LIBS "Build RCCPP with shared libraries." OFF)
mark_as_advanced(BUILD_SHARED_LIBS)


######################################################################
# Now, if required, do the SuperBuild
# If we are doing SuperBuild
#   We configure up to this point (see the return() statement)
#   and then we call SuperBuild.cmake, which builds all the
#   dependencies as CMake ExternalProjects, and then also builds
#   RCCPP as an ExternalProject. However instead of downloading
#   a tar file, you set the SOURCE_DIR to be THIS project, and force
#   the BUILD_SUPERBUILD flag to be off (to avoid infinite loop).
#
# If we are NOT doing superbuild, then the next statement has no
# effect, and the build goes on the same as before.
######################################################################
if(BUILD_SUPERBUILD)
  include("CMake/SuperBuild.cmake")
  return()
endif(BUILD_SUPERBUILD)


######################################################################
# End of SuperBuild. Print out where the source and binary folders
# are, just to make it really explicit... well, explicit to the user
# that bothers to read these messages! :-)
######################################################################
message("CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
message("CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}")


######################################################################
# Find Eigen.
######################################################################
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIR})
message("Loading Eigen from ${Eigen_INCLUDE_DIR}")


######################################################################
# Find Boost.
######################################################################
set(Boost_NO_SYSTEM_PATHS ON)
if(BUILD_SHARED_LIBS)
  set(Boost_USE_STATIC_LIBS OFF)
else()
  set(Boost_USE_STATIC_LIBS ON)
endif()
find_package( Boost 1.56 COMPONENTS filesystem system date_time regex thread iostreams program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


######################################################################
# Output directories, for when compiling, not installing.
######################################################################
foreach(type LIBRARY RUNTIME ARCHIVE)
  set(output_dir ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_${type}_OUTPUT_DIRECTORY ${output_dir} CACHE INTERNAL "Single output directory for building all libraries.")
  mark_as_advanced(CMAKE_${type}_OUTPUT_DIRECTORY)
endforeach()


######################################################################
# Setup further include folders
######################################################################
include_directories(Code/HelloWorld)
include_directories(Code/PointMatching)
include_directories(Testing/catch)
include_directories(Testing/HelloWorld)
 
######################################################################
# Decide what subdirectories we are building, and go and build them.
######################################################################
add_subdirectory(Code)
add_subdirectory(Testing)

