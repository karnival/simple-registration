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

include(ExternalProject)

set(EP_BASE "${CMAKE_BINARY_DIR}" CACHE PATH "Directory where the external projects are configured and built")
mark_as_advanced(EP_BASE)
set_property(DIRECTORY PROPERTY EP_BASE ${EP_BASE})

# For external projects like ITK we always want to turn their testing targets off.
set(EP_BUILD_TESTING OFF)
set(EP_BUILD_EXAMPLES OFF)
set(EP_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})

if(MSVC)
  set(EP_COMMON_C_FLAGS "${CMAKE_C_FLAGS} /bigobj /MP /W0 /Zi")
  set(EP_COMMON_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /MP /W0 /Zi")
  # we want symbols, even for release builds!
  set(EP_COMMON_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /debug")
  set(EP_COMMON_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /debug")
  set(EP_COMMON_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /debug")
  set(CMAKE_CXX_WARNING_LEVEL 0)
else()
  if(${BUILD_SHARED_LIBS})
    set(EP_COMMON_C_FLAGS "${CMAKE_C_FLAGS} -DLINUX_EXTRA")
    set(EP_COMMON_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLINUX_EXTRA")
  else()
    set(EP_COMMON_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -DLINUX_EXTRA")
    set(EP_COMMON_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX_EXTRA")
  endif()
  # These are not relevant for linux but we set them anyway to keep
  # the variable bits below symmetric.
  set(EP_COMMON_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
  set(EP_COMMON_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS}")
  set(EP_COMMON_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")
endif()

set(EP_COMMON_ARGS
  -DBUILD_TESTING:BOOL=${EP_BUILD_TESTING}
  -DBUILD_SHARED_LIBS:BOOL=${EP_BUILD_SHARED_LIBS}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
  -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
  -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
  -DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
  -DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
  -DCMAKE_EXE_LINKER_FLAGS:STRING=${EP_COMMON_EXE_LINKER_FLAGS}
  -DCMAKE_MODULE_LINKER_FLAGS:STRING=${EP_COMMON_MODULE_LINKER_FLAGS}
  -DCMAKE_SHARED_LINKER_FLAGS:STRING=${EP_COMMON_SHARED_LINKER_FLAGS}
  #debug flags
  -DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG}
  -DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG}
  #release flags
  -DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE}
  -DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE}
  #relwithdebinfo
  -DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}
  -DCMAKE_C_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_C_FLAGS_RELWITHDEBINFO}
)


# Compute -G arg for configuring external projects with the same CMake generator:
if(CMAKE_EXTRA_GENERATOR)
  set(GEN "${CMAKE_EXTRA_GENERATOR} - ${CMAKE_GENERATOR}")
else()
  set(GEN "${CMAKE_GENERATOR}")
endif()

######################################################################
# Include RCCPP helper macros
######################################################################
include(rccppExternalProjectHelperMacros)


######################################################################
# Loop round for each external project, compiling it
######################################################################
set(EXTERNAL_PROJECTS
  Eigen
  Boost
  ITK
)
foreach(p ${EXTERNAL_PROJECTS})
  include("CMake/CMakeExternals/${p}.cmake")
endforeach()


######################################################################
# Now compile RCCPP, using the packages we just provided.
######################################################################
if(NOT DEFINED SUPERBUILD_EXCLUDE_RCCPPBUILD_TARGET OR NOT SUPERBUILD_EXCLUDE_RCCPPBUILD_TARGET)

  set(proj RCCPP)
  set(proj_DEPENDENCIES ${Eigen_DEPENDS} ${Boost_DEPENDS} ${ITK_DEPENDS})

  ExternalProject_Add(${proj}
    DOWNLOAD_COMMAND ""
    INSTALL_COMMAND ""
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    BINARY_DIR ${proj}-build
    PREFIX ${proj}-cmake
    CMAKE_GENERATOR ${GEN}
    CMAKE_ARGS
      ${EP_COMMON_ARGS}
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
      -DBUILD_TESTING:BOOL=${BUILD_TESTING} # The value set in EP_COMMON_ARGS normally forces this off, but we may need RCCPP to be on.
      -DBUILD_SUPERBUILD:BOOL=OFF           # Must force this to be off, or else you will loop forever.
      -DEigen_DIR:PATH=${Eigen_DIR}
      -DEigen_ROOT:PATH=${Eigen_ROOT}
      -DEigen_INCLUDE_DIR:PATH=${Eigen_INCLUDE_DIR}
      -DBOOST_ROOT:PATH=${BOOST_ROOT}
      -DBOOST_INCLUDEDIR:PATH=${BOOST_INCLUDEDIR}
      -DBOOST_LIBRARYDIR:PATH=${BOOST_LIBRARYDIR}
      -DITK_DIR:PATH=${ITK_DIR}
      DEPENDS ${proj_DEPENDENCIES}
  )

endif()
