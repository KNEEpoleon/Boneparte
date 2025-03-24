// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lbr_fri_idl:msg/LBRWrenchCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__BUILDER_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lbr_fri_idl/msg/detail/lbr_wrench_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lbr_fri_idl
{

namespace msg
{

namespace builder
{

class Init_LBRWrenchCommand_wrench
{
public:
  explicit Init_LBRWrenchCommand_wrench(::lbr_fri_idl::msg::LBRWrenchCommand & msg)
  : msg_(msg)
  {}
  ::lbr_fri_idl::msg::LBRWrenchCommand wrench(::lbr_fri_idl::msg::LBRWrenchCommand::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRWrenchCommand msg_;
};

class Init_LBRWrenchCommand_joint_position
{
public:
  Init_LBRWrenchCommand_joint_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LBRWrenchCommand_wrench joint_position(::lbr_fri_idl::msg::LBRWrenchCommand::_joint_position_type arg)
  {
    msg_.joint_position = std::move(arg);
    return Init_LBRWrenchCommand_wrench(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRWrenchCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lbr_fri_idl::msg::LBRWrenchCommand>()
{
  return lbr_fri_idl::msg::builder::Init_LBRWrenchCommand_joint_position();
}

}  // namespace lbr_fri_idl

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_WRENCH_COMMAND__BUILDER_HPP_
