// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__BUILDER_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lbr_fri_idl/msg/detail/lbr_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lbr_fri_idl
{

namespace msg
{

namespace builder
{

class Init_LBRState_tracking_performance
{
public:
  explicit Init_LBRState_tracking_performance(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  ::lbr_fri_idl::msg::LBRState tracking_performance(::lbr_fri_idl::msg::LBRState::_tracking_performance_type arg)
  {
    msg_.tracking_performance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_ipo_joint_position
{
public:
  explicit Init_LBRState_ipo_joint_position(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_tracking_performance ipo_joint_position(::lbr_fri_idl::msg::LBRState::_ipo_joint_position_type arg)
  {
    msg_.ipo_joint_position = std::move(arg);
    return Init_LBRState_tracking_performance(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_external_torque
{
public:
  explicit Init_LBRState_external_torque(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_ipo_joint_position external_torque(::lbr_fri_idl::msg::LBRState::_external_torque_type arg)
  {
    msg_.external_torque = std::move(arg);
    return Init_LBRState_ipo_joint_position(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_commanded_torque
{
public:
  explicit Init_LBRState_commanded_torque(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_external_torque commanded_torque(::lbr_fri_idl::msg::LBRState::_commanded_torque_type arg)
  {
    msg_.commanded_torque = std::move(arg);
    return Init_LBRState_external_torque(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_measured_torque
{
public:
  explicit Init_LBRState_measured_torque(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_commanded_torque measured_torque(::lbr_fri_idl::msg::LBRState::_measured_torque_type arg)
  {
    msg_.measured_torque = std::move(arg);
    return Init_LBRState_commanded_torque(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_commanded_joint_position
{
public:
  explicit Init_LBRState_commanded_joint_position(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_measured_torque commanded_joint_position(::lbr_fri_idl::msg::LBRState::_commanded_joint_position_type arg)
  {
    msg_.commanded_joint_position = std::move(arg);
    return Init_LBRState_measured_torque(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_measured_joint_position
{
public:
  explicit Init_LBRState_measured_joint_position(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_commanded_joint_position measured_joint_position(::lbr_fri_idl::msg::LBRState::_measured_joint_position_type arg)
  {
    msg_.measured_joint_position = std::move(arg);
    return Init_LBRState_commanded_joint_position(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_time_stamp_nano_sec
{
public:
  explicit Init_LBRState_time_stamp_nano_sec(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_measured_joint_position time_stamp_nano_sec(::lbr_fri_idl::msg::LBRState::_time_stamp_nano_sec_type arg)
  {
    msg_.time_stamp_nano_sec = std::move(arg);
    return Init_LBRState_measured_joint_position(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_time_stamp_sec
{
public:
  explicit Init_LBRState_time_stamp_sec(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_time_stamp_nano_sec time_stamp_sec(::lbr_fri_idl::msg::LBRState::_time_stamp_sec_type arg)
  {
    msg_.time_stamp_sec = std::move(arg);
    return Init_LBRState_time_stamp_nano_sec(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_control_mode
{
public:
  explicit Init_LBRState_control_mode(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_time_stamp_sec control_mode(::lbr_fri_idl::msg::LBRState::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_LBRState_time_stamp_sec(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_overlay_type
{
public:
  explicit Init_LBRState_overlay_type(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_control_mode overlay_type(::lbr_fri_idl::msg::LBRState::_overlay_type_type arg)
  {
    msg_.overlay_type = std::move(arg);
    return Init_LBRState_control_mode(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_client_command_mode
{
public:
  explicit Init_LBRState_client_command_mode(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_overlay_type client_command_mode(::lbr_fri_idl::msg::LBRState::_client_command_mode_type arg)
  {
    msg_.client_command_mode = std::move(arg);
    return Init_LBRState_overlay_type(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_drive_state
{
public:
  explicit Init_LBRState_drive_state(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_client_command_mode drive_state(::lbr_fri_idl::msg::LBRState::_drive_state_type arg)
  {
    msg_.drive_state = std::move(arg);
    return Init_LBRState_client_command_mode(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_operation_mode
{
public:
  explicit Init_LBRState_operation_mode(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_drive_state operation_mode(::lbr_fri_idl::msg::LBRState::_operation_mode_type arg)
  {
    msg_.operation_mode = std::move(arg);
    return Init_LBRState_drive_state(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_safety_state
{
public:
  explicit Init_LBRState_safety_state(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_operation_mode safety_state(::lbr_fri_idl::msg::LBRState::_safety_state_type arg)
  {
    msg_.safety_state = std::move(arg);
    return Init_LBRState_operation_mode(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_connection_quality
{
public:
  explicit Init_LBRState_connection_quality(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_safety_state connection_quality(::lbr_fri_idl::msg::LBRState::_connection_quality_type arg)
  {
    msg_.connection_quality = std::move(arg);
    return Init_LBRState_safety_state(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_session_state
{
public:
  explicit Init_LBRState_session_state(::lbr_fri_idl::msg::LBRState & msg)
  : msg_(msg)
  {}
  Init_LBRState_connection_quality session_state(::lbr_fri_idl::msg::LBRState::_session_state_type arg)
  {
    msg_.session_state = std::move(arg);
    return Init_LBRState_connection_quality(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

class Init_LBRState_sample_time
{
public:
  Init_LBRState_sample_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LBRState_session_state sample_time(::lbr_fri_idl::msg::LBRState::_sample_time_type arg)
  {
    msg_.sample_time = std::move(arg);
    return Init_LBRState_session_state(msg_);
  }

private:
  ::lbr_fri_idl::msg::LBRState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lbr_fri_idl::msg::LBRState>()
{
  return lbr_fri_idl::msg::builder::Init_LBRState_sample_time();
}

}  // namespace lbr_fri_idl

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__BUILDER_HPP_
