#/*============================================================================
#
#  NifTK: A software platform for medical image computing.
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
# Set the minimum CMake version.
######################################################################
if(WIN32)
  cmake_minimum_required(VERSION 3.1)
else()
  cmake_minimum_required(VERSION 2.8.9)
endif()


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
# Setting supported build types. Should ONLY be Debug or Release.
######################################################################
if(DEFINED CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Valid options are Debug/Release")
else()
   set(CMAKE_BUILD_TYPE Release CACHE STRING "Setting build type to Release (default). Valid options: Debug/Release.")
endif()

if (NOT (CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "Release"))
  message(FATAL_ERROR "Build type ${CMAKE_BUILD_TYPE} is not supported.")
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
# Make sure Git is available.
######################################################################
find_package(Git REQUIRED)
if (WIN32)
  set(GITCOMMAND ${GIT_EXECUTABLE})
endif()


######################################################################
# Setup the path to load CMake macros, and extra CMake files.
######################################################################
set(CMAKE_MODULE_PATH
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/CMake
    ${CMAKE_SOURCE_DIR}/CMake/CMakeExternals
    ${CMAKE_MODULE_PATH}
   )


######################################################################
# Add in any functions/macros.
######################################################################
include(mitkMacroEmptyExternalProject)


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

return()

######################################################################
# Add additional entries to the CMAKE_MODULE_PATH, now all externals
# are already built.
######################################################################
set(CMAKE_MODULE_PATH
    ${aruco_DIR}/lib/cmake
    ${CMAKE_MODULE_PATH}
   )

######################################################################
# Check each GUI application to set up MACOSX_BUNDLE_NAMES
# as libraries are copied into each bundle.
######################################################################

if(APPLE)

  set(MACOSX_BUNDLE_NAMES)
  include("${CMAKE_CURRENT_SOURCE_DIR}/Code/Gui/MITK/Apps/Apps.cmake")

  foreach(NIFTK_APP ${NIFTK_APPS})

    # extract option_name
    string(REPLACE "^^" "\\;" target_info ${NIFTK_APP})
    set(target_info_list ${target_info})
    list(GET target_info_list 1 option_name)
    list(GET target_info_list 0 app_name)

    if(${option_name} OR NIFTK_BUILD_ALL_APPS)
      set(MACOSX_BUNDLE_NAMES ${MACOSX_BUNDLE_NAMES} ${app_name})
    endif()

  endforeach()

endif()

######################################################################
# Configure CMake files before we need to use them in the FIND_XXX.
#
# So, the variables NIFTK_BOOSTINSTALL etc.
# should be passed in on cmake command line, or set up using ccmake.
######################################################################

configure_file(${CMAKE_SOURCE_DIR}/CMake/FindFFTW.cmake ${CMAKE_BINARY_DIR}/FindFFTW.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindNiftyReg.cmake ${CMAKE_BINARY_DIR}/FindNiftyReg.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindNiftyRec.cmake ${CMAKE_BINARY_DIR}/FindNiftyRec.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindNiftySim.cmake ${CMAKE_BINARY_DIR}/FindNiftySim.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindNiftySeg.cmake ${CMAKE_BINARY_DIR}/FindNiftySeg.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindEigen.cmake ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/Findapriltags.cmake ${CMAKE_BINARY_DIR}/Findapriltags.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/mitkFunctionGetGccVersion.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/mitkFunctionGetGccVersion.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/mitkFunctionGetVersion.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/mitkFunctionGetVersion.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/mitkMacroEmptyExternalProject.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/mitkMacroEmptyExternalProject.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroBuildCLI.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroBuildCLI.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroCreateCommandLineApplication.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroCreateCommandLineApplication.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroCreateCommandLineScript.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroCreateCommandLineScript.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroCreateGuiApplication.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroCreateGuiApplication.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroGenerateCLIScript.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroGenerateCLIScript.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroGetGitBranch.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroGetGitBranch.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroGetGitDateTime.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroGetGitDateTime.cmake COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/niftkMacroInstallCLIScript.cmake ${CMAKE_BINARY_DIR}/CMakeMacros/niftkMacroInstallCLIScript.cmake COPYONLY)
file(GLOB package_depends_files ${CMAKE_SOURCE_DIR}/CMake/PackageDepends/*.cmake)
foreach(file_to_copy ${package_depends_files})
  file(COPY ${file_to_copy} DESTINATION ${CMAKE_BINARY_DIR}/CMakePackageDepends)
endforeach()

if(BUILD_IGI)
  add_definitions(-DBUILD_IGI)
endif()

######################################################################
# Find Mandatory External packages
######################################################################

######################################################################
# Find Boost.
######################################################################
set(Boost_NO_SYSTEM_PATHS ON)
# off is the default, but our optional pcl dependency will try to use static libs if we dont define it explicitly.
set(Boost_USE_STATIC_LIBS OFF)

find_package( Boost 1.54 COMPONENTS filesystem system date_time regex thread REQUIRED)
if(Boost_FOUND)
  message("Found Boost")
  set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  if(WIN32)
    if(NIFTK_WITHIN_SUPERBUILD)
      add_definitions(-DBoost_LIB_DIAGNOSTIC_DEFINITIONS)  # To get debug messages
      add_definitions(-DBOOST_ALL_NO_LIB)      # To stop auto-linking, which seems to be adding "lib" as library prefix in .obj files.
    endif(NIFTK_WITHIN_SUPERBUILD)
  endif(WIN32)
endif(Boost_FOUND)

######################################################################
# Find OpenMP and make sure Microsoft DirectComposition Library (dcomp.dll) is deployed
######################################################################

if (WIN32)
  find_package(OpenMP)

  if(OPENMP_FOUND)
    message("Found OpenMP")

    # the debug versions of the runtime library are considered "non redistributable".
    # we never hand out debug-build installers, so no problem having these packaged along correctly for internal testing.
    set(CMAKE_MSVC_ARCH_DEBUG_PREFIX "")
    set(CMAKE_MSVC_LIBNAME_FRAGMENT "")
    set(CMAKE_MSVC_OPENMP_DLLNAMESUFFIX "")
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
      set(CMAKE_MSVC_ARCH_DEBUG_PREFIX "Debug_NonRedist/")
      set(CMAKE_MSVC_LIBNAME_FRAGMENT "Debug")
      set(CMAKE_MSVC_OPENMP_DLLNAMESUFFIX "d")
    endif()

    if(CMAKE_CL_64)
      if(MSVC_VERSION GREATER 1599)
        # VS 10 and later:
        set(CMAKE_MSVC_ARCH "${CMAKE_MSVC_ARCH_DEBUG_PREFIX}x64")
      else()
        # VS 9 and earlier:
        set(CMAKE_MSVC_ARCH "${CMAKE_MSVC_ARCH_DEBUG_PREFIX}amd64")
      endif()
    else()
      set(CMAKE_MSVC_ARCH "${CMAKE_MSVC_ARCH_DEBUG_PREFIX}x86")
    endif()

    if (MSVC90)
      set (_omp_dll "${MSVC90_REDIST_DIR}/${CMAKE_MSVC_ARCH}/Microsoft.VC90.${CMAKE_MSVC_LIBNAME_FRAGMENT}OPENMP/vcomp90${CMAKE_MSVC_OPENMP_DLLNAMESUFFIX}.dll")
    elseif (MSVC10)
      # for some random reason, MSVC10_REDIST_DIR can be empty sometimes. as in empty-string, not
      # as in undefined-value.
      if (NOT "${MSVC10_REDIST_DIR}" STREQUAL "")
        set (_omp_dll "${MSVC10_REDIST_DIR}/${CMAKE_MSVC_ARCH}/Microsoft.VC100.${CMAKE_MSVC_LIBNAME_FRAGMENT}OPENMP/vcomp100${CMAKE_MSVC_OPENMP_DLLNAMESUFFIX}.dll")
      endif()
    elseif (MSVC11)
      # MSVC11_REDIST_DIR being empty, like above for 2010, has so far never happened for 2011.
      # so this is only precautionary.
      if (NOT "${MSVC11_REDIST_DIR}" STREQUAL "")
        set (_omp_dll "${MSVC11_REDIST_DIR}/${CMAKE_MSVC_ARCH}/Microsoft.VC110.${CMAKE_MSVC_LIBNAME_FRAGMENT}OPENMP/vcomp110${CMAKE_MSVC_OPENMP_DLLNAMESUFFIX}.dll")
      endif()
    elseif (MSVC12)
      if (NOT "${MSVC12_REDIST_DIR}" STREQUAL "")
        set (_omp_dll "${MSVC12_REDIST_DIR}/${CMAKE_MSVC_ARCH}/Microsoft.VC120.${CMAKE_MSVC_LIBNAME_FRAGMENT}OPENMP/vcomp120${CMAKE_MSVC_OPENMP_DLLNAMESUFFIX}.dll")
      endif()
    endif ()

    if ((NOT ("${_omp_dll}" STREQUAL "")) AND (EXISTS "${_omp_dll}"))
      message("Found OpenMP dll at: ${_omp_dll}")
      set (CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS "${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS};${_omp_dll}")
    else()
      message(WARNING "Did not find OpenMP DLLs (even though the compiler supports it)! The built installer might be missing dependencies.")
    endif()

  endif()
endif()

######################################################################
# Find VTK.
######################################################################
find_package(VTK REQUIRED)
if(VTK_FOUND)
  message("Found VTK")
  include(${VTK_USE_FILE})
endif(VTK_FOUND)

######################################################################
# Find GDCM. ITK is built with GDCM, so GDCM is effectively mandatory.
######################################################################
find_package(GDCM PATHS ${GDCM_DIR} REQUIRED)
if(GDCM_FOUND)
  message("Found GDCM")
  list(APPEND NIFTK_INCLUDE_DIRS_BUILD_TREE ${GDCM_INCLUDE_DIRS})
  link_directories(${GDCM_LIBRARY_DIRS})
endif()

# Find OpenCV.
######################################################################
if(BUILD_IGI)
  find_package(OpenCV REQUIRED)
  if(OpenCV_FOUND)
    message("Found OpenCV")
  endif(OpenCV_FOUND)

  # Here we do a find, just so we can read the license file
  find_package(NiftyLink REQUIRED)
  if(NiftyLink_FOUND)
    include(${NiftyLink_USE_FILE})
  endif()

endif(BUILD_IGI)


######################################################################
# Find MITK. Even though MITK can be built without Qt, CTK can't.
######################################################################
set(MITK_USE_EXT 1)
set(MITK_USE_Boost 1)
set(MITK_USE_DCMTK 1)
set(MITK_USE_BLUEBERRY ${QT_FOUND})
set(MITK_USE_QT ${QT_FOUND})
set(MITK_DESIRED_QT_VERSION ${DESIRED_QT_VERSION})
set(MITK_USE_OpenCV ${BUILD_IGI})

if(NIFTK_GENERATE_DOXYGEN_HELP)
  set(BLUEBERRY_USE_QT_HELP ON)
endif(NIFTK_GENERATE_DOXYGEN_HELP)

find_package(MITK REQUIRED)
if(MITK_FOUND)
  message("Found MITK")
  set(CMAKE_MODULE_PATH
    ${MITK_SOURCE_DIR}/CMake
    ${CMAKE_MODULE_PATH}
  )
  link_directories(${MITK_LINK_DIRECTORIES})
endif(MITK_FOUND)

######################################################################
# Find Optional External packages
######################################################################

######################################################################
# Find CGAL.
######################################################################

if (BUILD_MESHING)
  find_package(CGAL REQUIRED COMPONENTS Core ImageIO)
  include("${CGAL_USE_FILE}")
  # UseCGAL.cmake is buggy: despite reading the correct requested modules
  # it only adds the libCGAL.so base library to CGAL_LIBRARIES
  set(CGAL_LIBRARIES 
    CGAL
    CGAL_Core
    CGAL_ImageIO)
endif (BUILD_MESHING)

######################################################################
# Find FFTW.
######################################################################
if(NIFTK_USE_FFTW)
  find_package(FFTW)
  if(FFTW_FOUND)
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${FFTW_INCLUDE_DIR})
    message("Found FFTW")
  else(FFTW_FOUND)
    message("Didn't find FFTW, so Fluid based registration will not be built.")
  endif(FFTW_FOUND)
endif(NIFTK_USE_FFTW)

######################################################################
# Find NIFTYREG - (version will vary according to whether CUDA found)
######################################################################
if(BUILD_NIFTYREG)
  find_package(NiftyReg)
  if(NIFTYREG_FOUND)
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${NIFTYREG_INCLUDE_DIR})
    add_definitions(-DUSE_NIFTYREG)
    set(NIFTYREG_NVCC_FLAGS "${NIFTYREG_NVCC_FLAGS};-DUSE_NIFTYREG")
    message("Found NiftyReg")
  else(NIFTYREG_FOUND)
    message("Didn't find NiftyReg")
  endif(NIFTYREG_FOUND)
endif(BUILD_NIFTYREG)

if(NOT NIFTYREG_FOUND)
  message("No NiftyReg found, so turning off plugin")
  set("NIFTK_Plugins/uk.ac.ucl.cmic.niftyreg" OFF CACHE BOOL "NiftyReg plugin" FORCE)
endif()

######################################################################
# Find NIFTYREC - (version will vary according to whether CUDA found)
# NB: Is dependent on NIFTYREG and at time of writing only has GPU version
######################################################################

if(BUILD_NIFTYREC)
  find_package(NiftyRec)
  if(NIFTYREC_FOUND)
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${NIFTYREC_INCLUDE_DIR})
    add_definitions(-DUSE_NIFTYREC)
    set(NIFTYREC_NVCC_FLAGS "${NIFTYREC_NVCC_FLAGS};-DUSE_NIFTYREC")
    message("Found NiftyRec")
  else(NIFTYREC_FOUND)
    message("Didn't find NiftyRec")
  endif(NIFTYREC_FOUND)
endif(BUILD_NIFTYREC)

######################################################################
# Find NIFTYSIM
######################################################################

if(BUILD_NIFTYSIM)
  find_package(NiftySim)
  if(NIFTYSIM_FOUND)
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${NIFTYSIM_INCLUDE_DIR})
    add_definitions(-DUSE_NIFTYSIM)
    set(NIFTYSIM_NVCC_FLAGS "${NIFTYSIM_NVCC_FLAGS};-DUSE_NIFTYSIM")
    message("Found NiftySim")
  else(NIFTYSIM_FOUND)
    message("Didn't find NiftySim")
  endif(NIFTYSIM_FOUND)
endif(BUILD_NIFTYSIM)

######################################################################
# Find NiftySeg
######################################################################

if(BUILD_NIFTYSEG)
  find_package(NiftySeg)
  if(NIFTYSEG_FOUND)
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE ${NIFTK_INCLUDE_DIRS_BUILD_TREE} ${NIFTYSEG_INCLUDE_DIR})
    add_definitions(-DUSE_NIFTYSEG)
    set(NIFTYSEG_NVCC_FLAGS "${NIFTYSEG_NVCC_FLAGS};-DUSE_NIFTYSEG")
    message("Found NiftySeg")
  else(NIFTYSEG_FOUND)
    message("Didn't find NiftySeg")
  endif(NIFTYSEG_FOUND)
endif(BUILD_NIFTYSEG)

if(NOT NIFTYSEG_FOUND)
  message("No NiftySeg found, so turning off breast segmentation plugin")
  set("NIFTK_Plugins/uk.ac.ucl.cmic.breastsegmentation" OFF CACHE BOOL "Breast segmentation plugin" FORCE)
endif()

######################################################################
# Find RTK
######################################################################

if(BUILD_RTK)
  find_package(RTK)
  if(RTK_FOUND)
    message("Found RTK")
    include(${RTK_USE_FILE})
  endif(RTK_FOUND)
endif(BUILD_RTK)

######################################################################
# Find SlicerExecutionModel
######################################################################
if(BUILD_COMMAND_LINE_PROGRAMS)
  find_package(SlicerExecutionModel REQUIRED)
  if(SlicerExecutionModel_FOUND)
    message("Found SlicerExecutionModel")
    include(${SlicerExecutionModel_USE_FILE})
  else()
    message("Didn't find SlicerExecutionModel")
  endif()
endif()

######################################################################
# Configure prototype section, to get correct include path.
######################################################################
if (BUILD_PROTOTYPE)
  set(PROTOTYPE_DIR ${CMAKE_SOURCE_DIR}/Prototype)

  if (NOT EXISTS ${PROTOTYPE_DIR})
    find_package(Subversion)
    if (Subversion_FOUND)
      message("Found Subversion")
      message("Retrieving NiftyPrototype...")
      execute_process(
        COMMAND "${Subversion_SVN_EXECUTABLE}" checkout "${NIFTK_LOCATION_PROTOTYPE}" Prototype
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    else()
      message("Subversion not found. The Prototype project will not be built.")
    endif()
  endif()

  if (EXISTS ${PROTOTYPE_DIR} AND IS_DIRECTORY ${PROTOTYPE_DIR})
    set(NIFTK_INCLUDE_DIRS_BUILD_TREE
        ${NIFTK_INCLUDE_DIRS_BUILD_TREE}
        ${CMAKE_SOURCE_DIR}/Prototype/CUDA

        # Note that for these Insight Journal extensions, we dont compile the subpackage,
        # but by adding them to the include path, you can use any templated classes easily.
        # You may however have to recompile your ITK using ITK_USE_REVIEW=ON and ITK_USE_OPTIMIZED_REGISTRATION_METHODS=ON
        # depending on which classes you use.

        ${PROTOTYPE_DIR}/InsightJournal/Filters
        ${PROTOTYPE_DIR}/InsightJournal/IJ_181_ITKbinaryThinningImageFilter3D/Source
        ${PROTOTYPE_DIR}/InsightJournal/IJ_120_ITKSkeleton
        ${PROTOTYPE_DIR}/InsightJournal/IJ_644_ITKLogDomainDemonsRegistration/LogDomainDemonsRegistration-0.0.4-Source/Code
        ${PROTOTYPE_DIR}/InsightJournal/IJ_687_ITKSphericalDemons/QuadEdgeMeshFieldSmoothingFilters/Source
        ${PROTOTYPE_DIR}/InsightJournal/IJ_687_ITKSphericalDemons/QuadEdgeMeshRigidRegistration/Source
    )
  endif()
endif()

######################################################################
# Output directories, for when compiling, not installing.
######################################################################

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(SLICER_EXECUTABLE_OUTPUTPATH ${EXECUTABLE_OUTPUT_PATH}/cli-modules)
set(CXX_TEST_PATH ${EXECUTABLE_OUTPUT_PATH})              # Used to control, where to put unit test binaries.
set(BASELINE ${NIFTK_DATA_DIR}/Baseline)
set(INPUT_DATA ${NIFTK_DATA_DIR}/Input)
make_directory(${CMAKE_BINARY_DIR}/Testing/Temporary)
set(TEMP ${CMAKE_BINARY_DIR}/Testing/Temporary)

foreach(type LIBRARY RUNTIME ARCHIVE)
  set(output_dir ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_${type}_OUTPUT_DIRECTORY ${output_dir} CACHE INTERNAL "Single output directory for building all libraries.")
  mark_as_advanced(CMAKE_${type}_OUTPUT_DIRECTORY)
endforeach()

configure_file(${CMAKE_SOURCE_DIR}/Utilities/GenerateCommandLineDoxygen.sh.in
  ${EXECUTABLE_OUTPUT_PATH}/GenerateCommandLineDoxygen @ONLY)

######################################################################
# Compilation specific stuff, like flags etc.
######################################################################

if(NIFTK_CHECK_COVERAGE)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(NIFTK_COVERAGE_FLAGS "-g -fprofile-arcs -ftest-coverage  -O0 -DNDEBUG" )
    set(NIFTK_COVERAGE_C_FLAGS ${NIFTK_COVERAGE_FLAGS} CACHE STRING "C flags for coverage checking")
    set(NIFTK_COVERAGE_CXX_FLAGS ${NIFTK_COVERAGE_FLAGS} CACHE STRING "C++ flags for coverage checking")
  endif()
endif()

if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX /W2")
  set(CMAKE_CXX_WARNING_LEVEL 2)
  if(NIFTK_WITHIN_SUPERBUILD)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_LIB_PREFIX=\"\"")
  endif(NIFTK_WITHIN_SUPERBUILD)
endif(WIN32)

# On Visual Studio 8 MS deprecated C. This removes all 1.276E1265 security warnings
if(WIN32)
  if(NOT BORLAND)
    if(NOT CYGWIN)
      if(NOT MINGW)
        if(NOT ITK_ENABLE_VISUAL_STUDIO_DEPRECATED_C_WARNINGS)
          add_definitions(
            -D_CRT_FAR_MAPPINGS_NO_DEPRECATE
            -D_CRT_IS_WCTYPE_NO_DEPRECATE
            -D_CRT_MANAGED_FP_NO_DEPRECATE
            -D_CRT_NONSTDC_NO_DEPRECATE
            -D_CRT_SECURE_NO_DEPRECATE
            -D_CRT_SECURE_NO_DEPRECATE_GLOBALS
            -D_CRT_SETERRORMODE_BEEP_SLEEP_NO_DEPRECATE
            -D_CRT_TIME_FUNCTIONS_NO_DEPRECATE
            -D_CRT_VCCLRIT_NO_DEPRECATE
            -D_SCL_SECURE_NO_DEPRECATE
            )
        endif(NOT ITK_ENABLE_VISUAL_STUDIO_DEPRECATED_C_WARNINGS)
      endif(NOT MINGW)
    endif(NOT CYGWIN)
  endif(NOT BORLAND)
endif(WIN32)

# This should now always be true as MITK is required.
if(MITK_FOUND)
  set(${PROJECT_NAME}_MODULES_PACKAGE_DEPENDS_DIR "${PROJECT_SOURCE_DIR}/CMake/PackageDepends")
  list(INSERT MODULES_PACKAGE_DEPENDS_DIRS 0 ${${PROJECT_NAME}_MODULES_PACKAGE_DEPENDS_DIR})
  include(mitkCompilerSettings)
endif()

if(WIN32 AND NOT BUILD_SHARED_LIBS)
  add_definitions(-DNIFTK_STATIC)
endif()

#######################################################################
# Set the main install locations.
# These are relative to CMAKE_INSTALL_PREFIX which we MUST NOT touch.
#######################################################################
set(NIFTK_INSTALL_BASE_DIR ".")
set(NIFTK_INSTALL_BIN_DIR "bin")
set(NIFTK_INSTALL_INCLUDE_DIR "include/NifTK")
set(NIFTK_INSTALL_MATLAB_DIR "matlab")
set(NIFTK_INSTALL_DOC_DIR "doc")
set(NIFTK_INSTALL_LICENSES_DIR "licenses")
set(NIFTK_INSTALL_LIB_DIR "bin")

#######################################################################
# Set the main include path, just before generating NifTKConfigure.
#######################################################################
include_directories(
  ${NIFTK_INCLUDE_DIRS_BUILD_TREE}
  )

######################################################################
# Configure files that need variables substituting. Note that
# we 'Configure' them, which copies them to the CMAKE_BINARY_DIR
# while substituting variables, but it is the 'INSTALL' directives
# that place them in the installation directory.
######################################################################

configure_file(${CMAKE_SOURCE_DIR}/INSTALLATION.txt ${CMAKE_BINARY_DIR}/INSTALLATION.txt @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/LICENSE.txt ${CMAKE_BINARY_DIR}/LICENSE.txt @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/README.txt ${CMAKE_BINARY_DIR}/README.txt @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CONTRIBUTORS.txt ${CMAKE_BINARY_DIR}/CONTRIBUTORS.txt @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/Config/SetupDependencies.sh.in ${CMAKE_BINARY_DIR}/SetupDependencies.sh @ONLY )
configure_file(${CMAKE_SOURCE_DIR}/Config/SetupDependencies.csh.in ${CMAKE_BINARY_DIR}/SetupDependencies.csh @ONLY )
configure_file(${CMAKE_SOURCE_DIR}/Config/SetupNifTK.sh.in ${CMAKE_BINARY_DIR}/SetupNifTK.sh @ONLY )
configure_file(${CMAKE_SOURCE_DIR}/Config/SetupNifTK.csh.in ${CMAKE_BINARY_DIR}/SetupNifTK.csh @ONLY )
configure_file(${CMAKE_SOURCE_DIR}/Doc/Doxygen/niftkdoxygen.pl.in ${CMAKE_BINARY_DIR}/niftkdoxygen.pl)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Doxygen/doxygen.config.in ${CMAKE_BINARY_DIR}/doxygen.config)
configure_file(${CMAKE_SOURCE_DIR}/Doc/License.dox.in ${CMAKE_BINARY_DIR}/Doxygen/License.dox)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/ArUco.txt ${CMAKE_BINARY_DIR}/LICENSE_ArUco.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/AprilTags.txt ${CMAKE_BINARY_DIR}/LICENSE_AprilTags.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/Boost.txt ${CMAKE_BINARY_DIR}/LICENSE_Boost.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/CTK.txt ${CMAKE_BINARY_DIR}/LICENSE_CTK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/DCMTK.txt ${CMAKE_BINARY_DIR}/LICENSE_DCMTK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/GDCM.txt ${CMAKE_BINARY_DIR}/LICENSE_GDCM.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/ITK.txt ${CMAKE_BINARY_DIR}/LICENSE_ITK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/RTK.txt ${CMAKE_BINARY_DIR}/LICENSE_RTK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/VTK.txt ${CMAKE_BINARY_DIR}/LICENSE_VTK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/MITK.txt ${CMAKE_BINARY_DIR}/LICENSE_MITK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftyReg.txt ${CMAKE_BINARY_DIR}/LICENSE_NiftyReg.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftySeg.txt ${CMAKE_BINARY_DIR}/LICENSE_NiftySeg.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftySim.txt ${CMAKE_BINARY_DIR}/LICENSE_NiftySim.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/OpenCV.txt ${CMAKE_BINARY_DIR}/LICENSE_OpenCV.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/Qt.txt ${CMAKE_BINARY_DIR}/LICENSE_Qt.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/Eigen.txt ${CMAKE_BINARY_DIR}/LICENSE_Eigen.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/FLANN.txt ${CMAKE_BINARY_DIR}/LICENSE_FLANN.txt)
configure_file(${CMAKE_SOURCE_DIR}/Doc/Licenses/PCL.txt ${CMAKE_BINARY_DIR}/LICENSE_PCL.txt)
if(BUILD_IGI)
  configure_file(${NiftyLink_LICENSE_FILE} ${CMAKE_BINARY_DIR}/ExternalLicenses/NiftyLink.txt @ONLY)
  configure_file(${NiftyLink_OIGT_LICENSE_FILE} ${CMAKE_BINARY_DIR}/ExternalLicenses/OpenIGTLink.txt @ONLY)
  configure_file(${NiftyLink_QSLOG_LICENSE_FILE} ${CMAKE_BINARY_DIR}/ExternalLicenses/QsLog.txt @ONLY)
endif()

#############################################################################################
# These represent the libraries that the apps, libraries and unit tests actually link against
#############################################################################################

set(NIFTK_VTK_LIBS_BUT_WITHOUT_QT vtkIOCore vtkIOLegacy vtkCommonCore vtkRenderingCore vtkRenderingOpenGL vtkFiltersSources vtkFiltersGeometry vtkInteractionStyle vtkRenderingFreeType vtkRenderingFreeTypeOpenGL vtkIOImage vtkIOPLY)
set(NIFTK_VTK_LIBS_WITH_QT vtkViewsQt vtkRenderingQt vtkGUISupportQt vtkGUISupportQtSQL vtkGUISupportQtWebkit)

######################################################################
# Install commands for things like README, licenses etc.
######################################################################

install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/Boost.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/ITK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/RTK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/VTK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/Qt.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/MITK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/CTK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftyReg.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftyRec.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftySeg.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/NiftySim.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/DCMTK.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/OpenCV.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/SlicerExecutionModel.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/ArUco.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/AprilTags.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/Eigen.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/FLANN.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/Doc/Licenses/PCL.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
if(BUILD_IGI)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/ExternalLicenses/NiftyLink.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/ExternalLicenses/OpenIGTLink.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/ExternalLicenses/QsLog.txt DESTINATION ${NIFTK_INSTALL_LICENSES_DIR} COMPONENT documentation)
endif()
install(PROGRAMS ${CMAKE_BINARY_DIR}/LICENSE.txt DESTINATION ${NIFTK_INSTALL_BASE_DIR} COMPONENT applications)
install(PROGRAMS ${CMAKE_BINARY_DIR}/README.txt DESTINATION ${NIFTK_INSTALL_BASE_DIR} COMPONENT applications)
install(PROGRAMS ${CMAKE_BINARY_DIR}/INSTALLATION.txt DESTINATION ${NIFTK_INSTALL_BASE_DIR} COMPONENT applications)
install(PROGRAMS ${CMAKE_BINARY_DIR}/CONTRIBUTORS.txt DESTINATION ${NIFTK_INSTALL_BASE_DIR} COMPONENT applications)
if(NOT WIN32 AND NOT APPLE)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/SetupDependencies.sh DESTINATION ${NIFTK_INSTALL_BIN_DIR} COMPONENT applications)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/SetupDependencies.csh DESTINATION ${NIFTK_INSTALL_BIN_DIR} COMPONENT applications)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/SetupNifTK.sh DESTINATION ${NIFTK_INSTALL_BIN_DIR} COMPONENT applications)
  install(PROGRAMS ${CMAKE_BINARY_DIR}/SetupNifTK.csh DESTINATION ${NIFTK_INSTALL_BIN_DIR} COMPONENT applications)
endif()

##########################################################################
# Install commands for the NiftyReg, NiftySeg, NiftyRec, NiftySim packages
##########################################################################

# NiftyReg
if(BUILD_NIFTYREG AND NIFTYREG_FOUND)
  file(GLOB NIFTY_REG_LIB_FILES
       ${NIFTYREG_ROOT}/lib/*.so
       ${NIFTYREG_ROOT}/lib/*.dylib
       )
  MITK_INSTALL(PROGRAMS ${NIFTY_REG_LIB_FILES})

  file(GLOB NIFTY_REG_BIN_FILES ${NIFTYREG_ROOT}/bin/*)
  MITK_INSTALL(PROGRAMS ${NIFTY_REG_BIN_FILES})

  foreach(prog reg_aladin reg_f3d reg_jacobian reg_resample)
    set(FULL_APP_NAME ${prog})
    NIFTK_GENERATE_CLI_SCRIPT(NAME ${prog})
  endforeach()
endif(BUILD_NIFTYREG AND NIFTYREG_FOUND)

# NiftySeg
if(BUILD_NIFTYSEG)
  file(GLOB NIFTY_SEG_LIB_FILES
       ${NIFTYSEG_ROOT}/lib/*.so
       ${NIFTYSEG_ROOT}/lib/*.dylib
       )
  MITK_INSTALL(PROGRAMS ${NIFTY_SEG_LIB_FILES})

  file(GLOB NIFTY_SEG_BIN_FILES ${NIFTYSEG_ROOT}/bin/*)
  MITK_INSTALL(PROGRAMS ${NIFTY_SEG_BIN_FILES})

  foreach(prog seg_EM seg_LabFusion)
    set(FULL_APP_NAME ${prog})
    NIFTK_GENERATE_CLI_SCRIPT(NAME ${prog})
  endforeach()

  file(GLOB NIFTY_SEG_PRIORS_FILES ${CMAKE_BINARY_DIR}/../CMakeExternals/Install/NIFTYSEG/priors/*)
  install(PROGRAMS ${NIFTY_SEG_PRIORS_FILES} DESTINATION ${NIFTK_INSTALL_BASE_DIR}/priors COMPONENT applications)
endif(BUILD_NIFTYSEG)

# NiftyRec
if(BUILD_NIFTYREC AND NIFTYREC_FOUND)
  file(GLOB NIFTY_REC_LIB_FILES
       ${NIFTYREC_ROOT}/lib/*.so
       ${NIFTYREC_ROOT}/lib/*.dylib
       )
  MITK_INSTALL(PROGRAMS ${NIFTY_REC_LIB_FILES})

  file(GLOB NIFTY_REC_INCL_FILES ${NIFTYREC_ROOT}/include/*)
  MITK_INSTALL(PROGRAMS ${NIFTY_REC_INCL_FILES})
endif(BUILD_NIFTYREC AND NIFTYREC_FOUND)

# NiftySim
if(BUILD_NIFTYSIM AND NIFTYSIM_FOUND)
  file(GLOB NIFTY_SIM_LIB_FILES
       ${NIFTYSIM_ROOT}/lib/*.so
       ${NIFTYSIM_ROOT}/lib/*.dylib
       )
  MITK_INSTALL(PROGRAMS ${NIFTY_SIM_LIB_FILES})

  file(GLOB NIFTY_SIM_BIN_FILES ${NIFTYSIM_ROOT}/bin/*)
  MITK_INSTALL(PROGRAMS ${NIFTY_SIM_BIN_FILES})
endif(BUILD_NIFTYSIM AND NIFTYSIM_FOUND)

# Camino
if(BUILD_CAMINO AND NOT WIN32 AND NOT APPLE )
  file(GLOB CAMINO_BIN_FILES
       ${camino_DIR}/bin/*
      )
  file(GLOB CAMINO_MAN_FILES
       ${camino_DIR}/man/man1/*
      )
  install(FILES ${CAMINO_MAN_FILES} DESTINATION man/man1/ PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
  install(FILES ${CAMINO_BIN_FILES} DESTINATION bin PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif()

######################################################################
# Decide what subdirectories we are building, and go and build them.
######################################################################

add_subdirectory(Doc)
add_subdirectory(Code)

if(BUILD_UTILITIES)
  add_subdirectory(Utilities)
endif(BUILD_UTILITIES)

if(BUILD_TESTING)
  add_subdirectory(Testing)
endif(BUILD_TESTING)

if(BUILD_PROTOTYPE)
  add_subdirectory(Prototype)
endif(BUILD_PROTOTYPE)

######################################################################
# Packaging code.
######################################################################

# 1. Setup defaults, common for all generators.
include(CPackSetup)

# 2. Set variables that may be platform (Windows/Linux/Mac) or Generator (TGZ,DEB,NSIS) specific.
#    When CPack runs, it just uses all the information in the generated files cmake_install.cmake.
#    So, CPack does not read all your configuration information in CMakeLists.txt, and CPack
#    does not re-run any cmake process.  So, it is cmake that reads all the CMakeLists. files
#    and generates all the cmake_install.cmake.  So this command will use cmake to generate
#    and additional file, that we can politely ask cpack to include, in addition to all the
#    cmake_install.cmake files.

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Configuring the Start Menu and Desktop icon strings for NSIS

if (WIN32)
  set(ICONS_SETUP_STRING "")
  set(ICONS_REMOVE_STRING "")
  set(DESKTOP_ICONS_SETUP_STRING "")
  set(DESKTOP_ICONS_REMOVE_STRING "")
  set(DESKTOP_ICONS_REMOVE_STRING "")

  foreach(NIFTK_APP ${NIFTK_APPS})
    # extract option_name
    string(REPLACE "^^" "\\;" target_info ${NIFTK_APP})
    set(target_info_list ${target_info})
    list(GET target_info_list 1 option_name)
    list(GET target_info_list 0 app_name)

    message("Current App Name: ${app_name}")

    if(EXISTS "${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}/${app_name}.exe")
      set(ICONS_SETUP_STRING "${ICONS_SETUP_STRING}CreateShortCut '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\${app_name}.lnk' '$INSTDIR\\\\bin\\\\${app_name}.exe'\n")
      set(ICONS_REMOVE_STRING "${ICONS_REMOVE_STRING}Delete '$SMPROGRAMS\\\\$MUI_TEMP\\\\${app_name}.lnk'\n")

      set(DESKTOP_ICONS_SETUP_STRING "${DESKTOP_ICONS_SETUP_STRING}CreateShortCut '$DESKTOP\\\\${app_name}.lnk' '$INSTDIR\\\\bin\\\\${app_name}.exe'\n")
      set(DESKTOP_ICONS_REMOVE_STRING "${DESKTOP_ICONS_REMOVE_STRING}Delete '$DESKTOP\\\\${app_name}.lnk'\n")
    endif()
  endforeach()
endif()
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

configure_file(${CMAKE_SOURCE_DIR}/CMake/CPackOptions.cmake.in
               ${CMAKE_BINARY_DIR}/NIFTKCPackOptions.cmake)

# 3. Set a variable with the name of this file.		
set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_BINARY_DIR}/NIFTKCPackOptions.cmake")

# 4. Include this optional file.
include(NIFTKCPackOptions)

# 5. Include CPack module once all variables are set.
include(CPack)

# 6. Trac #1796 - Still need to call MITK rules to get Mac Bundles etc.
include(mitkInstallRules)

######################################################################
# If we are under Windows, create two batch files which correctly
# set up the environment for the application and for Visual Studio.
# These are only used to start VS when developing. Not used in
# the final installation package.
######################################################################
if(WIN32)
  if(MITK_FOUND)

    include(mitkFunctionCreateWindowsBatchScript)

    # if we didnt build pcl then substitute some benign empty path instead of potential garbage.
    if(BUILD_PCL)
      set(PCL_BIN_PATH_FOR_BATCH_FILE "${PCL_DIR}/../bin")
    else()
      set(PCL_BIN_PATH_FOR_BATCH_FILE "")
    endif()

    # on windows, the cgal-provided installer has some required 3rd party dlls.
    if(BUILD_MESHING AND CGAL_FOUND)
      # both gmp and mpfr sit in the same directory, so we only need one here.
      get_filename_component(CGALGMP_BIN_PATH_FOR_BATCH_FILE ${GMP_LIBRARIES} PATH)
      # and of course, dont forget cgal's own dlls.
      set(CGALGMP_BIN_PATH_FOR_BATCH_FILE "${CGALGMP_BIN_PATH_FOR_BATCH_FILE};${CGAL_INSTALL_PREFIX}/bin")
    else()
      set(CGALGMP_BIN_PATH_FOR_BATCH_FILE "")
    endif()

    # some other (top secret) fem code needs niftysim at runtime.
    if(NIFTYSIM_FOUND)
      set(NIFTYSIM_BIN_PATH_FOR_BATCH_FILE "${NIFTYSIM_DIR}/bin")
    else()
      set(NIFTYSIM_BIN_PATH_FOR_BATCH_FILE "")
    endif()

    set(VS_SOLUTION_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.sln")
    foreach(VS_BUILD_TYPE debug release)
      mitkFunctionCreateWindowsBatchScript("${CMAKE_SOURCE_DIR}/CMake/StartVS.bat.in"
        ${PROJECT_BINARY_DIR}/StartVS_${VS_BUILD_TYPE}.bat
        ${VS_BUILD_TYPE})

      message( "CreateWindowsBatchScript: Creating ${PROJECT_BINARY_DIR}/StartVS_${VS_BUILD_TYPE}.bat" )
    endforeach()

    foreach(NIFTK_APP ${NIFTK_APPS})
      # extract option_name
      string(REPLACE "^^" "\\;" target_info ${NIFTK_APP})
      set(target_info_list ${target_info})
      list(GET target_info_list 1 option_name)
      list(GET target_info_list 0 app_name)
      message("Current App Name: ${app_name}")

      foreach(VS_BUILD_TYPE debug release)
        message("CreateWindowsBatchScript: Creating ${PROJECT_BINARY_DIR}/bin/Start_${app_name}_${VS_BUILD_TYPE}.bat" )
        mitkFunctionCreateWindowsBatchScript("${CMAKE_SOURCE_DIR}/CMake/StartApp.bat.in"
          ${PROJECT_BINARY_DIR}/bin/Start${app_name}_${VS_BUILD_TYPE}.bat
          ${VS_BUILD_TYPE})
      endforeach()
    endforeach()

  else(MITK_FOUND)
    message( "CreateWindowsBatchScript: MITK not found" )
  endif(MITK_FOUND)
else(WIN32)
  message( "CreateWindowsBatchScript: WIN32 not found" )
endif(WIN32)

######################################################################
# Start: "NifTK Package Config"
######################################################################
set(NIFTK_EXPORTS_FILE "${NIFTK_BINARY_DIR}/NifTKExports.cmake")
file(REMOVE ${NIFTK_EXPORTS_FILE})

set(targets_to_export)
get_property(module_targets GLOBAL PROPERTY MITK_MODULE_TARGETS)
if(module_targets)
  list(APPEND targets_to_export ${module_targets})
endif()

if(MITK_USE_BLUEBERRY)
  get_property(plugin_targets GLOBAL PROPERTY NIFTK_EXPORTED_PLUGINS)
  if(plugin_targets)
    list(APPEND targets_to_export ${plugin_targets})
  endif()
endif()

export(TARGETS ${targets_to_export} APPEND FILE ${NIFTK_EXPORTS_FILE})

set(NIFTK_EXPORTED_TARGET_PROPERTIES )
foreach(target_to_export ${targets_to_export})
  get_target_property(autoload_targets ${target_to_export} MITK_AUTOLOAD_TARGETS)
  if(autoload_targets)
    set(MITK_EXPORTED_TARGET_PROPERTIES "${MITK_EXPORTED_TARGET_PROPERTIES} set_target_properties(${target_to_export} PROPERTIES MITK_AUTOLOAD_TARGETS \"${autoload_targets}\")")
  endif()
  get_target_property(autoload_dir ${target_to_export} MITK_AUTOLOAD_DIRECTORY)
  if(autoload_dir)
    set(MITK_EXPORTED_TARGET_PROPERTIES "${MITK_EXPORTED_TARGET_PROPERTIES} set_target_properties(${target_to_export} PROPERTIES MITK_AUTOLOAD_DIRECTORY \"${autoload_dir}\")")
  endif()
endforeach()

get_property(MITK_ADDITIONAL_LIBRARY_SEARCH_PATHS_CONFIG GLOBAL PROPERTY MITK_ADDITIONAL_LIBRARY_SEARCH_PATHS)

set(VISIBILITY_AVAILABLE 0)
set(visibility_test_flag "")
mitkFunctionCheckCompilerFlags("-fvisibility=hidden" visibility_test_flag)
if(visibility_test_flag)
  # The compiler understands -fvisiblity=hidden (probably gcc >= 4 or Clang)
  set(VISIBILITY_AVAILABLE 1)
endif()

# This generates a C++ header file with values that need compiling into code.
configure_file(${CMAKE_SOURCE_DIR}/Config/NifTKConfigure.h.in ${CMAKE_BINARY_DIR}/NifTKConfigure.h)

# This generates a C++ header file for GUI apps only.
configure_file(${CMAKE_SOURCE_DIR}/Code/Gui/MITK/Apps/NifTKApplication.h ${CMAKE_BINARY_DIR}/NifTKApplication.h)

# The next two generate CMake code to enable external projects to correctly use NifTK.
configure_file(${CMAKE_SOURCE_DIR}/Config/NifTKConfig.cmake.in ${CMAKE_BINARY_DIR}/NifTKConfig.cmake @ONLY)
write_basic_config_version_file(${CMAKE_CURRENT_BINARY_DIR}/NifTKConfigVersion.cmake VERSION "${NIFTK_VERSION_MAJOR}.${NIFTK_VERSION_MINOR}.${NIFTK_VERSION_PATCH}" COMPATIBILITY AnyNewerVersion)

######################################################################
# End: "NifTK Package Config"
######################################################################
