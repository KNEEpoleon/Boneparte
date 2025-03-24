// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice
#include "lbr_fri_idl/msg/detail/lbr_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lbr_fri_idl/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lbr_fri_idl/msg/detail/lbr_state__struct.h"
#include "lbr_fri_idl/msg/detail/lbr_state__functions.h"
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


using _LBRState__ros_msg_type = lbr_fri_idl__msg__LBRState;

static bool _LBRState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LBRState__ros_msg_type * ros_message = static_cast<const _LBRState__ros_msg_type *>(untyped_ros_message);
  // Field name: sample_time
  {
    cdr << ros_message->sample_time;
  }

  // Field name: session_state
  {
    cdr << ros_message->session_state;
  }

  // Field name: connection_quality
  {
    cdr << ros_message->connection_quality;
  }

  // Field name: safety_state
  {
    cdr << ros_message->safety_state;
  }

  // Field name: operation_mode
  {
    cdr << ros_message->operation_mode;
  }

  // Field name: drive_state
  {
    cdr << ros_message->drive_state;
  }

  // Field name: client_command_mode
  {
    cdr << ros_message->client_command_mode;
  }

  // Field name: overlay_type
  {
    cdr << ros_message->overlay_type;
  }

  // Field name: control_mode
  {
    cdr << ros_message->control_mode;
  }

  // Field name: time_stamp_sec
  {
    cdr << ros_message->time_stamp_sec;
  }

  // Field name: time_stamp_nano_sec
  {
    cdr << ros_message->time_stamp_nano_sec;
  }

  // Field name: measured_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->measured_joint_position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: commanded_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->commanded_joint_position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: measured_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->measured_torque;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: commanded_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->commanded_torque;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: external_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->external_torque;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: ipo_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->ipo_joint_position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: tracking_performance
  {
    cdr << ros_message->tracking_performance;
  }

  return true;
}

static bool _LBRState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LBRState__ros_msg_type * ros_message = static_cast<_LBRState__ros_msg_type *>(untyped_ros_message);
  // Field name: sample_time
  {
    cdr >> ros_message->sample_time;
  }

  // Field name: session_state
  {
    cdr >> ros_message->session_state;
  }

  // Field name: connection_quality
  {
    cdr >> ros_message->connection_quality;
  }

  // Field name: safety_state
  {
    cdr >> ros_message->safety_state;
  }

  // Field name: operation_mode
  {
    cdr >> ros_message->operation_mode;
  }

  // Field name: drive_state
  {
    cdr >> ros_message->drive_state;
  }

  // Field name: client_command_mode
  {
    cdr >> ros_message->client_command_mode;
  }

  // Field name: overlay_type
  {
    cdr >> ros_message->overlay_type;
  }

  // Field name: control_mode
  {
    cdr >> ros_message->control_mode;
  }

  // Field name: time_stamp_sec
  {
    cdr >> ros_message->time_stamp_sec;
  }

  // Field name: time_stamp_nano_sec
  {
    cdr >> ros_message->time_stamp_nano_sec;
  }

  // Field name: measured_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->measured_joint_position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: commanded_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->commanded_joint_position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: measured_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->measured_torque;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: commanded_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->commanded_torque;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: external_torque
  {
    size_t size = 7;
    auto array_ptr = ros_message->external_torque;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: ipo_joint_position
  {
    size_t size = 7;
    auto array_ptr = ros_message->ipo_joint_position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: tracking_performance
  {
    cdr >> ros_message->tracking_performance;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lbr_fri_idl
size_t get_serialized_size_lbr_fri_idl__msg__LBRState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LBRState__ros_msg_type * ros_message = static_cast<const _LBRState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name sample_time
  {
    size_t item_size = sizeof(ros_message->sample_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name session_state
  {
    size_t item_size = sizeof(ros_message->session_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name connection_quality
  {
    size_t item_size = sizeof(ros_message->connection_quality);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name safety_state
  {
    size_t item_size = sizeof(ros_message->safety_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name operation_mode
  {
    size_t item_size = sizeof(ros_message->operation_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_state
  {
    size_t item_size = sizeof(ros_message->drive_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name client_command_mode
  {
    size_t item_size = sizeof(ros_message->client_command_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name overlay_type
  {
    size_t item_size = sizeof(ros_message->overlay_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name control_mode
  {
    size_t item_size = sizeof(ros_message->control_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name time_stamp_sec
  {
    size_t item_size = sizeof(ros_message->time_stamp_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name time_stamp_nano_sec
  {
    size_t item_size = sizeof(ros_message->time_stamp_nano_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measured_joint_position
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->measured_joint_position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name commanded_joint_position
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->commanded_joint_position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measured_torque
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->measured_torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name commanded_torque
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->commanded_torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name external_torque
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->external_torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ipo_joint_position
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->ipo_joint_position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tracking_performance
  {
    size_t item_size = sizeof(ros_message->tracking_performance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LBRState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lbr_fri_idl__msg__LBRState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lbr_fri_idl
size_t max_serialized_size_lbr_fri_idl__msg__LBRState(
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

  // member: sample_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: session_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: connection_quality
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: safety_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: operation_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: drive_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: client_command_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: overlay_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: control_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: time_stamp_sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: time_stamp_nano_sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: measured_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: commanded_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: measured_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: commanded_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: external_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ipo_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tracking_performance
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lbr_fri_idl__msg__LBRState;
    is_plain =
      (
      offsetof(DataType, tracking_performance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LBRState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_lbr_fri_idl__msg__LBRState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LBRState = {
  "lbr_fri_idl::msg",
  "LBRState",
  _LBRState__cdr_serialize,
  _LBRState__cdr_deserialize,
  _LBRState__get_serialized_size,
  _LBRState__max_serialized_size
};

static rosidl_message_type_support_t _LBRState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LBRState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lbr_fri_idl, msg, LBRState)() {
  return &_LBRState__type_support;
}

#if defined(__cplusplus)
}
#endif
