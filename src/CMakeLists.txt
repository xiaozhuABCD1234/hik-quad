configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hikquad/version.hpp.in
  ${CMAKE_BINARY_DIR}/generated/hikquad/version.hpp
  @ONLY
)

add_library(hikquad_core STATIC
  log.cpp
  error.cpp
  pixel_format.cpp
  paths.cpp
  cli.cpp
  device_info.cpp
  camera.cpp
  camera_array.cpp
  config.cpp
  calibration.cpp
  calib_io.cpp
  lut.cpp
  stitcher.cpp
  version.cpp
)

target_include_directories(hikquad_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(hikquad_core
  PUBLIC
    ${OpenCV_LIBS}
    HikSDK::MvCameraControl
    Threads::Threads
)

target_compile_definitions(hikquad_core
  PRIVATE
    HIK_QUAD_MVS_ROOT="${HIK_SDK_ROOT}"
    HIK_QUAD_MVS_LIB_DIR="${HIK_SDK_LIB_DIR}"
    HIK_QUAD_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

hik_quad_enable_warnings(hikquad_core)
hik_quad_configure_rpath(hikquad_core)
set_target_properties(hikquad_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
