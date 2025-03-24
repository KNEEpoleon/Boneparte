// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lbr_fri_idl:msg/LBRTorqueCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__TRAITS_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lbr_fri_idl/msg/detail/lbr_torque_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace lbr_fri_idl
{

namespace msg
{

inline void to_flow_style_yaml(
  const LBRTorqueCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_position
  {
    if (msg.joint_position.size() == 0) {
      out << "joint_position: []";
    } else {
      out << "joint_position: [";
      size_t pending_items = msg.joint_position.size();
      for (auto item : msg.joint_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: torque
  {
    if (msg.torque.size() == 0) {
      out << "torque: []";
    } else {
      out << "torque: [";
      size_t pending_items = msg.torque.size();
      for (auto item : msg.torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LBRTorqueCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_position.size() == 0) {
      out << "joint_position: []\n";
    } else {
      out << "joint_position:\n";
      for (auto item : msg.joint_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.torque.size() == 0) {
      out << "torque: []\n";
    } else {
      out << "torque:\n";
      for (auto item : msg.torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LBRTorqueCommand & msg, bool use_flow_style = false)
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
  const lbr_fri_idl::msg::LBRTorqueCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  lbr_fri_idl::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lbr_fri_idl::msg::to_yaml() instead")]]
inline std::string to_yaml(const lbr_fri_idl::msg::LBRTorqueCommand & msg)
{
  return lbr_fri_idl::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lbr_fri_idl::msg::LBRTorqueCommand>()
{
  return "lbr_fri_idl::msg::LBRTorqueCommand";
}

template<>
inline const char * name<lbr_fri_idl::msg::LBRTorqueCommand>()
{
  return "lbr_fri_idl/msg/LBRTorqueCommand";
}

template<>
struct has_fixed_size<lbr_fri_idl::msg::LBRTorqueCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lbr_fri_idl::msg::LBRTorqueCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lbr_fri_idl::msg::LBRTorqueCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__TRAITS_HPP_
