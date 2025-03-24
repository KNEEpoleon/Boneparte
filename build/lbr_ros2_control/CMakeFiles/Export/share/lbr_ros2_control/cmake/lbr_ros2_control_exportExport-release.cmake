#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lbr_ros2_control::lbr_ros2_control" for configuration "Release"
set_property(TARGET lbr_ros2_control::lbr_ros2_control APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lbr_ros2_control::lbr_ros2_control PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/lbr_ros2_control/liblbr_ros2_control.so"
  IMPORTED_SONAME_RELEASE "liblbr_ros2_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lbr_ros2_control::lbr_ros2_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_lbr_ros2_control::lbr_ros2_control "${_IMPORT_PREFIX}/lib/lbr_ros2_control/liblbr_ros2_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
