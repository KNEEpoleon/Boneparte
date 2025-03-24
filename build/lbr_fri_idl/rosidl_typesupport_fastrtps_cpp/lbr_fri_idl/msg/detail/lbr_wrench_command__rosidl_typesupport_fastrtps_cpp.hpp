// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from lbr_fri_idl:msg/LBRWrenchCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "lbr_fri_idl/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "lbr_fri_idl/msg/detail/lbr_wrench_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace lbr_fri_idl
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
cdr_serialize(
  const lbr_fri_idl::msg::LBRWrenchCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  lbr_fri_idl::msg::LBRWrenchCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
get_serialized_size(
  const lbr_fri_idl::msg::LBRWrenchCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
max_serialized_size_LBRWrenchCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace lbr_fri_idl

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, lbr_fri_idl, msg, LBRWrenchCommand)();

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
