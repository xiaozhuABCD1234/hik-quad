cmake_minimum_required(VERSION 3.16)

project(
  hik_quad
  VERSION 0.2.0
  DESCRIPTION "Standalone Hikrobot USB3 multi-camera capture, calibration and stitching"
  LANGUAGES CXX
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(HIK_QUAD_BUILD_TESTS "Build unit tests" ON)
option(HIK_QUAD_BUILD_HW_TESTS "Register tests that need the physical cameras" OFF)
option(HIK_QUAD_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo)
endif()

include(CompilerWarnings)
include(HikSDK)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(Threads REQUIRED)

message(STATUS "OpenCV: ${OpenCV_VERSION} (${OpenCV_DIR})")
message(STATUS "MVS SDK: ${HIK_SDK_ROOT}")

add_subdirectory(src)
add_subdirectory(apps)

if(HIK_QUAD_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()

include(HikQuadInstall)
