cmake_minimum_required(VERSION 3.20)

project(yarpdevcarlepuck2
  LANGUAGES CXX
  VERSION 1.0.0)
 
include(FeatureSummary)

find_package(YCM 0.12 REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")  
  get_property(_USER_CMAKE_CXX_FLAGS CACHE CMAKE_CXX_FLAGS PROPERTY VALUE)
  string(FIND ${_USER_CMAKE_CXX_FLAGS} "/MP" _MSVC_MP_IDX)
  if(_MSVC_MP_IDX EQUAL -1)
    message("enforce parallel build")
    set(CMAKE_CXX_FLAGS "${_USER_CMAKE_CXX_FLAGS} /MP" CACHE STRING "enforce parallel build" FORCE)    
  endif()
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "yarp-device-ovrheadset is always built with dynamic plugins")
set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "Build libraries as shared as opposed to static")

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

include(AddUninstallTarget)

find_package(YARP 3.4 COMPONENTS os sig dev carl REQUIRED)     
set(CMAKE_C_FLAGS "${YARP_C_FLAGS} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${YARP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

yarp_configure_plugins_installation(yarp-device-CarlEPuck2)

find_package(ACE 6.5)

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

add_subdirectory(CarlEpuck2)
