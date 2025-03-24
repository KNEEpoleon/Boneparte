// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lbr_fri_idl:msg/LBRTorqueCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__BUILDER_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lbr_fri_idl/msg/detail/lbr_torque_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lbr_fri_idl
{

namespace msg
{

namespace builder
{

class Init_LBRTorqueCommand_torque
{
public:
  explicit Init_LBRTorqueCommand_torque(::lbr_fri_idl::msg::LBRTorqueCommand & msg)
  : msg_(msg)
  {}
  ::lbr_fri_idl::msg::LBRTorqueCommand torque(::lbr_fri_idl::msg::LBRTorqueCommand::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRTorqueCommand msg_;
};

class Init_LBRTorqueCommand_joint_position
{
public:
  Init_LBRTorqueCommand_joint_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LBRTorqueCommand_torque joint_position(::lbr_fri_idl::msg::LBRTorqueCommand::_joint_position_type arg)
  {
    msg_.joint_position = std::move(arg);
    return Init_LBRTorqueCommand_torque(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRTorqueCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lbr_fri_idl::msg::LBRTorqueCommand>()
{
  return lbr_fri_idl::msg::builder::Init_LBRTorqueCommand_joint_position();
}

}  // namespace lbr_fri_idl

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_TORQUE_COMMAND__BUILDER_HPP_
