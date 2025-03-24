// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lbr_fri_idl:msg/LBRTorqueCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lbr_fri_idl/msg/detail/lbr_torque_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lbr_fri_idl
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LBRTorqueCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lbr_fri_idl::msg::LBRTorqueCommand(_init);
}

void LBRTorqueCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lbr_fri_idl::msg::LBRTorqueCommand *>(message_memory);
  typed_message->~LBRTorqueCommand();
}

size_t size_function__LBRTorqueCommand__joint_position(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__LBRTorqueCommand__joint_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__LBRTorqueCommand__joint_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__LBRTorqueCommand__joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LBRTorqueCommand__joint_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LBRTorqueCommand__joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LBRTorqueCommand__joint_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LBRTorqueCommand__torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__LBRTorqueCommand__torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__LBRTorqueCommand__torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__LBRTorqueCommand__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LBRTorqueCommand__torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LBRTorqueCommand__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LBRTorqueCommand__torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LBRTorqueCommand_message_member_array[2] = {
  {
    "joint_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl::msg::LBRTorqueCommand, joint_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__LBRTorqueCommand__joint_position,  // size() function pointer
    get_const_function__LBRTorqueCommand__joint_position,  // get_const(index) function pointer
    get_function__LBRTorqueCommand__joint_position,  // get(index) function pointer
    fetch_function__LBRTorqueCommand__joint_position,  // fetch(index, &value) function pointer
    assign_function__LBRTorqueCommand__joint_position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl::msg::LBRTorqueCommand, torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__LBRTorqueCommand__torque,  // size() function pointer
    get_const_function__LBRTorqueCommand__torque,  // get_const(index) function pointer
    get_function__LBRTorqueCommand__torque,  // get(index) function pointer
    fetch_function__LBRTorqueCommand__torque,  // fetch(index, &value) function pointer
    assign_function__LBRTorqueCommand__torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LBRTorqueCommand_message_members = {
  "lbr_fri_idl::msg",  // message namespace
  "LBRTorqueCommand",  // message name
  2,  // number of fields
  sizeof(lbr_fri_idl::msg::LBRTorqueCommand),
  LBRTorqueCommand_message_member_array,  // message members
  LBRTorqueCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  LBRTorqueCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LBRTorqueCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LBRTorqueCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace lbr_fri_idl


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lbr_fri_idl::msg::LBRTorqueCommand>()
{
  return &::lbr_fri_idl::msg::rosidl_typesupport_introspection_cpp::LBRTorqueCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lbr_fri_idl, msg, LBRTorqueCommand)() {
  return &::lbr_fri_idl::msg::rosidl_typesupport_introspection_cpp::LBRTorqueCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
