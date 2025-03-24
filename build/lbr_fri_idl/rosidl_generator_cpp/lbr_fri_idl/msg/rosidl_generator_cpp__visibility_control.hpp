// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define LBR_FRI_IDL__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_lbr_fri_idl __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_lbr_fri_idl __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_lbr_fri_idl __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_lbr_fri_idl __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_lbr_fri_idl
    #define ROSIDL_GENERATOR_CPP_PUBLIC_lbr_fri_idl ROSIDL_GENERATOR_CPP_EXPORT_lbr_fri_idl
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_lbr_fri_idl ROSIDL_GENERATOR_CPP_IMPORT_lbr_fri_idl
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_lbr_fri_idl __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_lbr_fri_idl
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_lbr_fri_idl __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_lbr_fri_idl
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
