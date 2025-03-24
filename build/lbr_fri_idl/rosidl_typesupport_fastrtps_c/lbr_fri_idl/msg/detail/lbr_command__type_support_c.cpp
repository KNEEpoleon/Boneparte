// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lbr_fri_idl:msg/LBRCommand.idl
// generated code does not contain a copyright notice
#include "lbr_fri_idl/msg/detail/lbr_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lbr_fri_idl/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lbr_fri_idl/msg/detail/lbr_command__struct.h"
#include "lbr_fri_idl/msg/detail/lbr_command__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LBRCommand__ros_msg_type = lbr_fri_idl__msg__LBRCommand;

static bool _LBRCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LBRCommand__ros_msg_type * ros_message = static_cast<const _LBRCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->joint_position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->torque;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: wrench
  {
    size_t size = 6;
    auto array_ptr = ros_message->wrench;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _LBRCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LBRCommand__ros_msg_type * ros_message = static_cast<_LBRCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->joint_position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->torque;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: wrench
  {
    size_t size = 6;
    auto array_ptr = ros_message->wrench;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lbr_fri_idl
size_t get_serialized_size_lbr_fri_idl__msg__LBRCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LBRCommand__ros_msg_type * ros_message = static_cast<const _LBRCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name joint_position
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->joint_position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name torque
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wrench
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->wrench;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LBRCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lbr_fri_idl__msg__LBRCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lbr_fri_idl
size_t max_serialized_size_lbr_fri_idl__msg__LBRCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: wrench
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lbr_fri_idl__msg__LBRCommand;
    is_plain =
      (
      offsetof(DataType, wrench) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LBRCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_lbr_fri_idl__msg__LBRCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LBRCommand = {
  "lbr_fri_idl::msg",
  "LBRCommand",
  _LBRCommand__cdr_serialize,
  _LBRCommand__cdr_deserialize,
  _LBRCommand__get_serialized_size,
  _LBRCommand__max_serialized_size
};

static rosidl_message_type_support_t _LBRCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LBRCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lbr_fri_idl, msg, LBRCommand)() {
  return &_LBRCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
