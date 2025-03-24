# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:lbr_fri_idl__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:lbr_fri_idl__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:lbr_fri_idl__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:lbr_fri_idl__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:lbr_fri_idl__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:lbr_fri_idl__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:lbr_fri_idl__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:lbr_fri_idl__rosidl_typesupport_cpp;__rosidl_generator_py:lbr_fri_idl__rosidl_generator_py")

# populate lbr_fri_idl_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "lbr_fri_idl::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'lbr_fri_idl' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND lbr_fri_idl_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
