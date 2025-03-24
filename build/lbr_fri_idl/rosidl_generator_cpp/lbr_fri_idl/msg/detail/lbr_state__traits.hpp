// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__TRAITS_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lbr_fri_idl/msg/detail/lbr_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace lbr_fri_idl
{

namespace msg
{

inline void to_flow_style_yaml(
  const LBRState & msg,
  std::ostream & out)
{
  out << "{";
  // member: sample_time
  {
    out << "sample_time: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_time, out);
    out << ", ";
  }

  // member: session_state
  {
    out << "session_state: ";
    rosidl_generator_traits::value_to_yaml(msg.session_state, out);
    out << ", ";
  }

  // member: connection_quality
  {
    out << "connection_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_quality, out);
    out << ", ";
  }

  // member: safety_state
  {
    out << "safety_state: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_state, out);
    out << ", ";
  }

  // member: operation_mode
  {
    out << "operation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_mode, out);
    out << ", ";
  }

  // member: drive_state
  {
    out << "drive_state: ";
    rosidl_generator_traits::value_to_yaml(msg.drive_state, out);
    out << ", ";
  }

  // member: client_command_mode
  {
    out << "client_command_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.client_command_mode, out);
    out << ", ";
  }

  // member: overlay_type
  {
    out << "overlay_type: ";
    rosidl_generator_traits::value_to_yaml(msg.overlay_type, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: time_stamp_sec
  {
    out << "time_stamp_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp_sec, out);
    out << ", ";
  }

  // member: time_stamp_nano_sec
  {
    out << "time_stamp_nano_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp_nano_sec, out);
    out << ", ";
  }

  // member: measured_joint_position
  {
    if (msg.measured_joint_position.size() == 0) {
      out << "measured_joint_position: []";
    } else {
      out << "measured_joint_position: [";
      size_t pending_items = msg.measured_joint_position.size();
      for (auto item : msg.measured_joint_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_joint_position
  {
    if (msg.commanded_joint_position.size() == 0) {
      out << "commanded_joint_position: []";
    } else {
      out << "commanded_joint_position: [";
      size_t pending_items = msg.commanded_joint_position.size();
      for (auto item : msg.commanded_joint_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: measured_torque
  {
    if (msg.measured_torque.size() == 0) {
      out << "measured_torque: []";
    } else {
      out << "measured_torque: [";
      size_t pending_items = msg.measured_torque.size();
      for (auto item : msg.measured_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_torque
  {
    if (msg.commanded_torque.size() == 0) {
      out << "commanded_torque: []";
    } else {
      out << "commanded_torque: [";
      size_t pending_items = msg.commanded_torque.size();
      for (auto item : msg.commanded_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: external_torque
  {
    if (msg.external_torque.size() == 0) {
      out << "external_torque: []";
    } else {
      out << "external_torque: [";
      size_t pending_items = msg.external_torque.size();
      for (auto item : msg.external_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ipo_joint_position
  {
    if (msg.ipo_joint_position.size() == 0) {
      out << "ipo_joint_position: []";
    } else {
      out << "ipo_joint_position: [";
      size_t pending_items = msg.ipo_joint_position.size();
      for (auto item : msg.ipo_joint_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tracking_performance
  {
    out << "tracking_performance: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_performance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LBRState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sample_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_time: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_time, out);
    out << "\n";
  }

  // member: session_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_state: ";
    rosidl_generator_traits::value_to_yaml(msg.session_state, out);
    out << "\n";
  }

  // member: connection_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_quality, out);
    out << "\n";
  }

  // member: safety_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_state: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_state, out);
    out << "\n";
  }

  // member: operation_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_mode, out);
    out << "\n";
  }

  // member: drive_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drive_state: ";
    rosidl_generator_traits::value_to_yaml(msg.drive_state, out);
    out << "\n";
  }

  // member: client_command_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "client_command_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.client_command_mode, out);
    out << "\n";
  }

  // member: overlay_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overlay_type: ";
    rosidl_generator_traits::value_to_yaml(msg.overlay_type, out);
    out << "\n";
  }

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << "\n";
  }

  // member: time_stamp_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_stamp_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp_sec, out);
    out << "\n";
  }

  // member: time_stamp_nano_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_stamp_nano_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp_nano_sec, out);
    out << "\n";
  }

  // member: measured_joint_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.measured_joint_position.size() == 0) {
      out << "measured_joint_position: []\n";
    } else {
      out << "measured_joint_position:\n";
      for (auto item : msg.measured_joint_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_joint_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_joint_position.size() == 0) {
      out << "commanded_joint_position: []\n";
    } else {
      out << "commanded_joint_position:\n";
      for (auto item : msg.commanded_joint_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: measured_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.measured_torque.size() == 0) {
      out << "measured_torque: []\n";
    } else {
      out << "measured_torque:\n";
      for (auto item : msg.measured_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_torque.size() == 0) {
      out << "commanded_torque: []\n";
    } else {
      out << "commanded_torque:\n";
      for (auto item : msg.commanded_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: external_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.external_torque.size() == 0) {
      out << "external_torque: []\n";
    } else {
      out << "external_torque:\n";
      for (auto item : msg.external_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ipo_joint_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ipo_joint_position.size() == 0) {
      out << "ipo_joint_position: []\n";
    } else {
      out << "ipo_joint_position:\n";
      for (auto item : msg.ipo_joint_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tracking_performance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_performance: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_performance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LBRState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lbr_fri_idl

namespace rosidl_generator_traits
{

[[deprecated("use lbr_fri_idl::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lbr_fri_idl::msg::LBRState & msg,
  std::ostream & out, size_t indentation = 0)
{
  lbr_fri_idl::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lbr_fri_idl::msg::to_yaml() instead")]]
inline std::string to_yaml(const lbr_fri_idl::msg::LBRState & msg)
{
  return lbr_fri_idl::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lbr_fri_idl::msg::LBRState>()
{
  return "lbr_fri_idl::msg::LBRState";
}

template<>
inline const char * name<lbr_fri_idl::msg::LBRState>()
{
  return "lbr_fri_idl/msg/LBRState";
}

template<>
struct has_fixed_size<lbr_fri_idl::msg::LBRState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lbr_fri_idl::msg::LBRState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lbr_fri_idl::msg::LBRState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__TRAITS_HPP_
