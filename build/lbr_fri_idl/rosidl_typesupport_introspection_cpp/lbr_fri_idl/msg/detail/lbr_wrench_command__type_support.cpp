// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lbr_fri_idl:msg/LBRWrenchCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lbr_fri_idl/msg/detail/lbr_wrench_command__struct.hpp"
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

void LBRWrenchCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lbr_fri_idl::msg::LBRWrenchCommand(_init);
}

void LBRWrenchCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lbr_fri_idl::msg::LBRWrenchCommand *>(message_memory);
  typed_message->~LBRWrenchCommand();
}

size_t size_function__LBRWrenchCommand__joint_position(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__LBRWrenchCommand__joint_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__LBRWrenchCommand__joint_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__LBRWrenchCommand__joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LBRWrenchCommand__joint_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LBRWrenchCommand__joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LBRWrenchCommand__joint_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LBRWrenchCommand__wrench(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__LBRWrenchCommand__wrench(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__LBRWrenchCommand__wrench(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__LBRWrenchCommand__wrench(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LBRWrenchCommand__wrench(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LBRWrenchCommand__wrench(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LBRWrenchCommand__wrench(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LBRWrenchCommand_message_member_array[2] = {
  {
    "joint_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl::msg::LBRWrenchCommand, joint_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__LBRWrenchCommand__joint_position,  // size() function pointer
    get_const_function__LBRWrenchCommand__joint_position,  // get_const(index) function pointer
    get_function__LBRWrenchCommand__joint_position,  // get(index) function pointer
    fetch_function__LBRWrenchCommand__joint_position,  // fetch(index, &value) function pointer
    assign_function__LBRWrenchCommand__joint_position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wrench",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl::msg::LBRWrenchCommand, wrench),  // bytes offset in struct
    nullptr,  // default value
    size_function__LBRWrenchCommand__wrench,  // size() function pointer
    get_const_function__LBRWrenchCommand__wrench,  // get_const(index) function pointer
    get_function__LBRWrenchCommand__wrench,  // get(index) function pointer
    fetch_function__LBRWrenchCommand__wrench,  // fetch(index, &value) function pointer
    assign_function__LBRWrenchCommand__wrench,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LBRWrenchCommand_message_members = {
  "lbr_fri_idl::msg",  // message namespace
  "LBRWrenchCommand",  // message name
  2,  // number of fields
  sizeof(lbr_fri_idl::msg::LBRWrenchCommand),
  LBRWrenchCommand_message_member_array,  // message members
  LBRWrenchCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  LBRWrenchCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LBRWrenchCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LBRWrenchCommand_message_members,
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
get_message_type_support_handle<lbr_fri_idl::msg::LBRWrenchCommand>()
{
  return &::lbr_fri_idl::msg::rosidl_typesupport_introspection_cpp::LBRWrenchCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lbr_fri_idl, msg, LBRWrenchCommand)() {
  return &::lbr_fri_idl::msg::rosidl_typesupport_introspection_cpp::LBRWrenchCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
