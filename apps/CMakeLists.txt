add_executable(hik_quad_list hik_quad_list.cpp)
add_executable(hik_quad_capture hik_quad_capture.cpp)
add_executable(hik_quad_calib hik_quad_calib.cpp)
add_executable(hik_quad_lut hik_quad_lut.cpp)
add_executable(hik_quad_stitch hik_quad_stitch.cpp)

foreach(_target
    hik_quad_list
    hik_quad_capture
    hik_quad_calib
    hik_quad_lut
    hik_quad_stitch)
  target_link_libraries(${_target} PRIVATE hikquad_core)
  target_include_directories(${_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  hik_quad_enable_warnings(${_target})
  hik_quad_configure_rpath(${_target})
endforeach()
