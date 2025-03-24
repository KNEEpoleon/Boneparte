// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lbr_fri_idl:msg/LBRJointPositionCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__BUILDER_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lbr_fri_idl/msg/detail/lbr_joint_position_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lbr_fri_idl
{

namespace msg
{

namespace builder
{

class Init_LBRJointPositionCommand_joint_position
{
public:
  Init_LBRJointPositionCommand_joint_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lbr_fri_idl::msg::LBRJointPositionCommand joint_position(::lbr_fri_idl::msg::LBRJointPositionCommand::_joint_position_type arg)
  {
    msg_.joint_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRJointPositionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lbr_fri_idl::msg::LBRJointPositionCommand>()
{
  return lbr_fri_idl::msg::builder::Init_LBRJointPositionCommand_joint_position();
}

}  // namespace lbr_fri_idl

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__BUILDER_HPP_
