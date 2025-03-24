#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lbr_fri_ros2::lbr_fri_ros2" for configuration "Release"
set_property(TARGET lbr_fri_ros2::lbr_fri_ros2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lbr_fri_ros2::lbr_fri_ros2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblbr_fri_ros2.so"
  IMPORTED_SONAME_RELEASE "liblbr_fri_ros2.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lbr_fri_ros2::lbr_fri_ros2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_lbr_fri_ros2::lbr_fri_ros2 "${_IMPORT_PREFIX}/lib/liblbr_fri_ros2.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
