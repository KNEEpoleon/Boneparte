#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "FRIClient::FRIClient" for configuration "Release"
set_property(TARGET FRIClient::FRIClient APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(FRIClient::FRIClient PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libFRIClient.so"
  IMPORTED_SONAME_RELEASE "libFRIClient.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS FRIClient::FRIClient )
list(APPEND _IMPORT_CHECK_FILES_FOR_FRIClient::FRIClient "${_IMPORT_PREFIX}/lib/libFRIClient.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
