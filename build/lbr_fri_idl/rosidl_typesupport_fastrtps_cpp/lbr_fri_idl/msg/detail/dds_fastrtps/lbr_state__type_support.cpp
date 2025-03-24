// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice
#include "lbr_fri_idl/msg/detail/lbr_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "lbr_fri_idl/msg/detail/lbr_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace lbr_fri_idl
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
cdr_serialize(
  const lbr_fri_idl::msg::LBRState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: sample_time
  cdr << ros_message.sample_time;
  // Member: session_state
  cdr << ros_message.session_state;
  // Member: connection_quality
  cdr << ros_message.connection_quality;
  // Member: safety_state
  cdr << ros_message.safety_state;
  // Member: operation_mode
  cdr << ros_message.operation_mode;
  // Member: drive_state
  cdr << ros_message.drive_state;
  // Member: client_command_mode
  cdr << ros_message.client_command_mode;
  // Member: overlay_type
  cdr << ros_message.overlay_type;
  // Member: control_mode
  cdr << ros_message.control_mode;
  // Member: time_stamp_sec
  cdr << ros_message.time_stamp_sec;
  // Member: time_stamp_nano_sec
  cdr << ros_message.time_stamp_nano_sec;
  // Member: measured_joint_position
  {
    cdr << ros_message.measured_joint_position;
  }
  // Member: commanded_joint_position
  {
    cdr << ros_message.commanded_joint_position;
  }
  // Member: measured_torque
  {
    cdr << ros_message.measured_torque;
  }
  // Member: commanded_torque
  {
    cdr << ros_message.commanded_torque;
  }
  // Member: external_torque
  {
    cdr << ros_message.external_torque;
  }
  // Member: ipo_joint_position
  {
    cdr << ros_message.ipo_joint_position;
  }
  // Member: tracking_performance
  cdr << ros_message.tracking_performance;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  lbr_fri_idl::msg::LBRState & ros_message)
{
  // Member: sample_time
  cdr >> ros_message.sample_time;

  // Member: session_state
  cdr >> ros_message.session_state;

  // Member: connection_quality
  cdr >> ros_message.connection_quality;

  // Member: safety_state
  cdr >> ros_message.safety_state;

  // Member: operation_mode
  cdr >> ros_message.operation_mode;

  // Member: drive_state
  cdr >> ros_message.drive_state;

  // Member: client_command_mode
  cdr >> ros_message.client_command_mode;

  // Member: overlay_type
  cdr >> ros_message.overlay_type;

  // Member: control_mode
  cdr >> ros_message.control_mode;

  // Member: time_stamp_sec
  cdr >> ros_message.time_stamp_sec;

  // Member: time_stamp_nano_sec
  cdr >> ros_message.time_stamp_nano_sec;

  // Member: measured_joint_position
  {
    cdr >> ros_message.measured_joint_position;
  }

  // Member: commanded_joint_position
  {
    cdr >> ros_message.commanded_joint_position;
  }

  // Member: measured_torque
  {
    cdr >> ros_message.measured_torque;
  }

  // Member: commanded_torque
  {
    cdr >> ros_message.commanded_torque;
  }

  // Member: external_torque
  {
    cdr >> ros_message.external_torque;
  }

  // Member: ipo_joint_position
  {
    cdr >> ros_message.ipo_joint_position;
  }

  // Member: tracking_performance
  cdr >> ros_message.tracking_performance;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
get_serialized_size(
  const lbr_fri_idl::msg::LBRState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: sample_time
  {
    size_t item_size = sizeof(ros_message.sample_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: session_state
  {
    size_t item_size = sizeof(ros_message.session_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: connection_quality
  {
    size_t item_size = sizeof(ros_message.connection_quality);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: safety_state
  {
    size_t item_size = sizeof(ros_message.safety_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: operation_mode
  {
    size_t item_size = sizeof(ros_message.operation_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: drive_state
  {
    size_t item_size = sizeof(ros_message.drive_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: client_command_mode
  {
    size_t item_size = sizeof(ros_message.client_command_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: overlay_type
  {
    size_t item_size = sizeof(ros_message.overlay_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: control_mode
  {
    size_t item_size = sizeof(ros_message.control_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: time_stamp_sec
  {
    size_t item_size = sizeof(ros_message.time_stamp_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: time_stamp_nano_sec
  {
    size_t item_size = sizeof(ros_message.time_stamp_nano_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_joint_position
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.measured_joint_position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: commanded_joint_position
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.commanded_joint_position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.measured_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: commanded_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.commanded_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: external_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.external_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ipo_joint_position
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.ipo_joint_position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tracking_performance
  {
    size_t item_size = sizeof(ros_message.tracking_performance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lbr_fri_idl
max_serialized_size_LBRState(
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


  // Member: sample_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: session_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: connection_quality
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: safety_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: operation_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: drive_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: client_command_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: overlay_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: control_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: time_stamp_sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: time_stamp_nano_sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: measured_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: commanded_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: measured_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: commanded_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: external_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ipo_joint_position
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tracking_performance
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
    using DataType = lbr_fri_idl::msg::LBRState;
    is_plain =
      (
      offsetof(DataType, tracking_performance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _LBRState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const lbr_fri_idl::msg::LBRState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LBRState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<lbr_fri_idl::msg::LBRState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LBRState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const lbr_fri_idl::msg::LBRState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LBRState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LBRState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LBRState__callbacks = {
  "lbr_fri_idl::msg",
  "LBRState",
  _LBRState__cdr_serialize,
  _LBRState__cdr_deserialize,
  _LBRState__get_serialized_size,
  _LBRState__max_serialized_size
};

static rosidl_message_type_support_t _LBRState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LBRState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace lbr_fri_idl

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_lbr_fri_idl
const rosidl_message_type_support_t *
get_message_type_support_handle<lbr_fri_idl::msg::LBRState>()
{
  return &lbr_fri_idl::msg::typesupport_fastrtps_cpp::_LBRState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, lbr_fri_idl, msg, LBRState)() {
  return &lbr_fri_idl::msg::typesupport_fastrtps_cpp::_LBRState__handle;
}

#ifdef __cplusplus
}
#endif
