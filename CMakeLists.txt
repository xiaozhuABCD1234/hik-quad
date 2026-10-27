cmake_minimum_required(VERSION 3.16)

project(hik_quad LANGUAGES CXX
  DESCRIPTION "4-camera Hikrobot calibration and video stitching")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(HikSDK)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio)
find_package(Threads REQUIRED)

message(STATUS "OpenCV: ${OpenCV_VERSION} (${OpenCV_DIR})")
message(STATUS "MVS SDK: ${HIK_SDK_ROOT}")

add_library(hikquad_warnings INTERFACE)
if(NOT MSVC)
  target_compile_options(hikquad_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

add_library(hikquad_core STATIC
  src/calib_io.cpp
  src/calibration.cpp
  src/camera.cpp
  src/camera_array.cpp
  src/cli.cpp
  src/config.cpp
  src/device_info.cpp
  src/error.cpp
  src/log.cpp
  src/lut.cpp
  src/media.cpp
  src/pixel_format.cpp
  src/stitcher.cpp
)

target_include_directories(hikquad_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(hikquad_core
  PUBLIC
    ${OpenCV_LIBS}
    HikSDK::MvCameraControl
    Threads::Threads
  PRIVATE
    hikquad_warnings
)
target_compile_definitions(hikquad_core PRIVATE
  HIK_QUAD_MVS_ROOT="${HIK_SDK_ROOT}"
  HIK_QUAD_MVS_LIB_DIR="${HIK_SDK_LIB_DIR}"
)

foreach(_app capture calib stitch)
  add_executable(hik_quad_${_app} apps/hik_quad_${_app}.cpp)
  target_include_directories(hik_quad_${_app} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/apps")
  target_link_libraries(hik_quad_${_app} PRIVATE hikquad_core hikquad_warnings)
  hik_quad_configure_rpath(hik_quad_${_app})
endforeach()
