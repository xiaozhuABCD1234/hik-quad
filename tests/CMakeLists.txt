add_executable(hikquad_tests
  test_main.cpp
  test_pixel_format.cpp
  test_config.cpp
  test_calibration_io.cpp
  test_lut.cpp
  test_calibration_math.cpp
  test_cli.cpp
  test_pipeline.cpp
)
target_link_libraries(hikquad_tests PRIVATE hikquad_core)
target_include_directories(hikquad_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
hik_quad_enable_warnings(hikquad_tests)
hik_quad_configure_rpath(hikquad_tests)

add_test(NAME hikquad_unit COMMAND hikquad_tests)

if(HIK_QUAD_BUILD_HW_TESTS)
  add_test(NAME hikquad_hw_list COMMAND hik_quad_list)
  set_tests_properties(hikquad_hw_list PROPERTIES PASS_REGULAR_EXPRESSION "physical USB3 Vision cameras: 4")
endif()
