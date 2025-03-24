// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_HPP_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__lbr_fri_idl__msg__LBRState __attribute__((deprecated))
#else
# define DEPRECATED__lbr_fri_idl__msg__LBRState __declspec(deprecated)
#endif

namespace lbr_fri_idl
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LBRState_
{
  using Type = LBRState_<ContainerAllocator>;

  explicit LBRState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_time = 0.0;
      this->session_state = 0;
      this->connection_quality = 0;
      this->safety_state = 0;
      this->operation_mode = 0;
      this->drive_state = 0;
      this->client_command_mode = 0;
      this->overlay_type = 0;
      this->control_mode = 0;
      this->time_stamp_sec = 0ul;
      this->time_stamp_nano_sec = 0ul;
      std::fill<typename std::array<double, 7>::iterator, double>(this->measured_joint_position.begin(), this->measured_joint_position.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_joint_position.begin(), this->commanded_joint_position.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->measured_torque.begin(), this->measured_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_torque.begin(), this->commanded_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->external_torque.begin(), this->external_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->ipo_joint_position.begin(), this->ipo_joint_position.end(), 0.0);
      this->tracking_performance = 0.0;
    }
  }

  explicit LBRState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : measured_joint_position(_alloc),
    commanded_joint_position(_alloc),
    measured_torque(_alloc),
    commanded_torque(_alloc),
    external_torque(_alloc),
    ipo_joint_position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_time = 0.0;
      this->session_state = 0;
      this->connection_quality = 0;
      this->safety_state = 0;
      this->operation_mode = 0;
      this->drive_state = 0;
      this->client_command_mode = 0;
      this->overlay_type = 0;
      this->control_mode = 0;
      this->time_stamp_sec = 0ul;
      this->time_stamp_nano_sec = 0ul;
      std::fill<typename std::array<double, 7>::iterator, double>(this->measured_joint_position.begin(), this->measured_joint_position.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_joint_position.begin(), this->commanded_joint_position.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->measured_torque.begin(), this->measured_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_torque.begin(), this->commanded_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->external_torque.begin(), this->external_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->ipo_joint_position.begin(), this->ipo_joint_position.end(), 0.0);
      this->tracking_performance = 0.0;
    }
  }

  // field types and members
  using _sample_time_type =
    double;
  _sample_time_type sample_time;
  using _session_state_type =
    int8_t;
  _session_state_type session_state;
  using _connection_quality_type =
    int8_t;
  _connection_quality_type connection_quality;
  using _safety_state_type =
    int8_t;
  _safety_state_type safety_state;
  using _operation_mode_type =
    int8_t;
  _operation_mode_type operation_mode;
  using _drive_state_type =
    int8_t;
  _drive_state_type drive_state;
  using _client_command_mode_type =
    int8_t;
  _client_command_mode_type client_command_mode;
  using _overlay_type_type =
    int8_t;
  _overlay_type_type overlay_type;
  using _control_mode_type =
    int8_t;
  _control_mode_type control_mode;
  using _time_stamp_sec_type =
    uint32_t;
  _time_stamp_sec_type time_stamp_sec;
  using _time_stamp_nano_sec_type =
    uint32_t;
  _time_stamp_nano_sec_type time_stamp_nano_sec;
  using _measured_joint_position_type =
    std::array<double, 7>;
  _measured_joint_position_type measured_joint_position;
  using _commanded_joint_position_type =
    std::array<double, 7>;
  _commanded_joint_position_type commanded_joint_position;
  using _measured_torque_type =
    std::array<double, 7>;
  _measured_torque_type measured_torque;
  using _commanded_torque_type =
    std::array<double, 7>;
  _commanded_torque_type commanded_torque;
  using _external_torque_type =
    std::array<double, 7>;
  _external_torque_type external_torque;
  using _ipo_joint_position_type =
    std::array<double, 7>;
  _ipo_joint_position_type ipo_joint_position;
  using _tracking_performance_type =
    double;
  _tracking_performance_type tracking_performance;

  // setters for named parameter idiom
  Type & set__sample_time(
    const double & _arg)
  {
    this->sample_time = _arg;
    return *this;
  }
  Type & set__session_state(
    const int8_t & _arg)
  {
    this->session_state = _arg;
    return *this;
  }
  Type & set__connection_quality(
    const int8_t & _arg)
  {
    this->connection_quality = _arg;
    return *this;
  }
  Type & set__safety_state(
    const int8_t & _arg)
  {
    this->safety_state = _arg;
    return *this;
  }
  Type & set__operation_mode(
    const int8_t & _arg)
  {
    this->operation_mode = _arg;
    return *this;
  }
  Type & set__drive_state(
    const int8_t & _arg)
  {
    this->drive_state = _arg;
    return *this;
  }
  Type & set__client_command_mode(
    const int8_t & _arg)
  {
    this->client_command_mode = _arg;
    return *this;
  }
  Type & set__overlay_type(
    const int8_t & _arg)
  {
    this->overlay_type = _arg;
    return *this;
  }
  Type & set__control_mode(
    const int8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__time_stamp_sec(
    const uint32_t & _arg)
  {
    this->time_stamp_sec = _arg;
    return *this;
  }
  Type & set__time_stamp_nano_sec(
    const uint32_t & _arg)
  {
    this->time_stamp_nano_sec = _arg;
    return *this;
  }
  Type & set__measured_joint_position(
    const std::array<double, 7> & _arg)
  {
    this->measured_joint_position = _arg;
    return *this;
  }
  Type & set__commanded_joint_position(
    const std::array<double, 7> & _arg)
  {
    this->commanded_joint_position = _arg;
    return *this;
  }
  Type & set__measured_torque(
    const std::array<double, 7> & _arg)
  {
    this->measured_torque = _arg;
    return *this;
  }
  Type & set__commanded_torque(
    const std::array<double, 7> & _arg)
  {
    this->commanded_torque = _arg;
    return *this;
  }
  Type & set__external_torque(
    const std::array<double, 7> & _arg)
  {
    this->external_torque = _arg;
    return *this;
  }
  Type & set__ipo_joint_position(
    const std::array<double, 7> & _arg)
  {
    this->ipo_joint_position = _arg;
    return *this;
  }
  Type & set__tracking_performance(
    const double & _arg)
  {
    this->tracking_performance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lbr_fri_idl::msg::LBRState_<ContainerAllocator> *;
  using ConstRawPtr =
    const lbr_fri_idl::msg::LBRState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lbr_fri_idl::msg::LBRState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lbr_fri_idl::msg::LBRState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lbr_fri_idl__msg__LBRState
    std::shared_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lbr_fri_idl__msg__LBRState
    std::shared_ptr<lbr_fri_idl::msg::LBRState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LBRState_ & other) const
  {
    if (this->sample_time != other.sample_time) {
      return false;
    }
    if (this->session_state != other.session_state) {
      return false;
    }
    if (this->connection_quality != other.connection_quality) {
      return false;
    }
    if (this->safety_state != other.safety_state) {
      return false;
    }
    if (this->operation_mode != other.operation_mode) {
      return false;
    }
    if (this->drive_state != other.drive_state) {
      return false;
    }
    if (this->client_command_mode != other.client_command_mode) {
      return false;
    }
    if (this->overlay_type != other.overlay_type) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->time_stamp_sec != other.time_stamp_sec) {
      return false;
    }
    if (this->time_stamp_nano_sec != other.time_stamp_nano_sec) {
      return false;
    }
    if (this->measured_joint_position != other.measured_joint_position) {
      return false;
    }
    if (this->commanded_joint_position != other.commanded_joint_position) {
      return false;
    }
    if (this->measured_torque != other.measured_torque) {
      return false;
    }
    if (this->commanded_torque != other.commanded_torque) {
      return false;
    }
    if (this->external_torque != other.external_torque) {
      return false;
    }
    if (this->ipo_joint_position != other.ipo_joint_position) {
      return false;
    }
    if (this->tracking_performance != other.tracking_performance) {
      return false;
    }
    return true;
  }
  bool operator!=(const LBRState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LBRState_

// alias to use template instance with default allocator
using LBRState =
  lbr_fri_idl::msg::LBRState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lbr_fri_idl

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_HPP_
