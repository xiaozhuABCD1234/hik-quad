# Locate the Hikrobot MVS machine-vision SDK (Camera SDK).
#
# The SDK is proprietary; this project never vendors it.  Override the root with
#   cmake -DHIK_SDK_ROOT=/opt/MVS ...
# or the MVCAM_SDK_PATH environment variable.

if(DEFINED ENV{MVCAM_SDK_PATH} AND NOT DEFINED HIK_SDK_ROOT)
  set(HIK_SDK_ROOT "$ENV{MVCAM_SDK_PATH}" CACHE PATH "Hikrobot MVS SDK root")
else()
  set(HIK_SDK_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_hik_arch "64")
else()
  set(_hik_arch "32")
endif()

if(WIN32)
  set(_hik_lib_subdir "lib")
  set(_hik_lib_name "MvCameraControl")
else()
  set(_hik_lib_subdir "lib/${_hik_arch}")
  set(_hik_lib_name "MvCameraControl")
endif()

find_path(
  HIK_SDK_INCLUDE_DIR
  NAMES MvCameraControl.h
  HINTS "${HIK_SDK_ROOT}/include"
  NO_DEFAULT_PATH
)
if(NOT HIK_SDK_INCLUDE_DIR)
  find_path(HIK_SDK_INCLUDE_DIR NAMES MvCameraControl.h)
endif()

find_library(
  HIK_SDK_LIBRARY
  NAMES ${_hik_lib_name}
  HINTS "${HIK_SDK_ROOT}/${_hik_lib_subdir}"
  NO_DEFAULT_PATH
)
if(NOT HIK_SDK_LIBRARY)
  find_library(HIK_SDK_LIBRARY NAMES ${_hik_lib_name})
endif()

if(NOT HIK_SDK_INCLUDE_DIR)
  message(FATAL_ERROR
    "MvCameraControl.h was not found. Install the Hikrobot MVS SDK and pass "
    "-DHIK_SDK_ROOT=/opt/MVS (or set MVCAM_SDK_PATH).")
endif()
if(NOT HIK_SDK_LIBRARY)
  message(FATAL_ERROR
    "libMvCameraControl was not found under ${HIK_SDK_ROOT}. Expected "
    "${HIK_SDK_ROOT}/${_hik_lib_subdir}.")
endif()

get_filename_component(HIK_SDK_LIB_DIR "${HIK_SDK_LIBRARY}" DIRECTORY)
set(HIK_SDK_LIB_DIR "${HIK_SDK_LIB_DIR}" CACHE PATH "Directory of libMvCameraControl" FORCE)
message(STATUS "MVS include: ${HIK_SDK_INCLUDE_DIR}")
message(STATUS "MVS library: ${HIK_SDK_LIBRARY}")

add_library(HikSDK::MvCameraControl SHARED IMPORTED)
set_target_properties(HikSDK::MvCameraControl PROPERTIES
  IMPORTED_LOCATION "${HIK_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${HIK_SDK_INCLUDE_DIR}"
)

# The MVS runtime dlopen()s libFormatConversion/libMvUsb3vTL and the GenICam
# producers.  An executable RUNPATH lets the loader find them without requiring
# the user to source /opt/MVS/bin/set_env_path.sh first.
function(hik_quad_configure_rpath target)
  if(UNIX AND NOT APPLE)
    set_target_properties(${target} PROPERTIES
      BUILD_RPATH "${HIK_SDK_LIB_DIR}"
      INSTALL_RPATH "${HIK_SDK_LIB_DIR};\$ORIGIN/../lib"
      INSTALL_RPATH_USE_LINK_PATH TRUE
    )
  endif()
endfunction()
