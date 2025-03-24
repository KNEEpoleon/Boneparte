// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lbr_fri_idl:msg/LBRTorqueCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lbr_fri_idl/msg/detail/lbr_torque_command__rosidl_typesupport_introspection_c.h"
#include "lbr_fri_idl/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lbr_fri_idl/msg/detail/lbr_torque_command__functions.h"
#include "lbr_fri_idl/msg/detail/lbr_torque_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lbr_fri_idl__msg__LBRTorqueCommand__init(message_memory);
}

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_fini_function(void * message_memory)
{
  lbr_fri_idl__msg__LBRTorqueCommand__fini(message_memory);
}

size_t lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__size_function__LBRTorqueCommand__joint_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__joint_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__joint_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__fetch_function__LBRTorqueCommand__joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__joint_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__assign_function__LBRTorqueCommand__joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__joint_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__size_function__LBRTorqueCommand__torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__fetch_function__LBRTorqueCommand__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__assign_function__LBRTorqueCommand__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_member_array[2] = {
  {
    "joint_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRTorqueCommand, joint_position),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__size_function__LBRTorqueCommand__joint_position,  // size() function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__joint_position,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__joint_position,  // get(index) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__fetch_function__LBRTorqueCommand__joint_position,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__assign_function__LBRTorqueCommand__joint_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRTorqueCommand, torque),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__size_function__LBRTorqueCommand__torque,  // size() function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_const_function__LBRTorqueCommand__torque,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__get_function__LBRTorqueCommand__torque,  // get(index) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__fetch_function__LBRTorqueCommand__torque,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__assign_function__LBRTorqueCommand__torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_members = {
  "lbr_fri_idl__msg",  // message namespace
  "LBRTorqueCommand",  // message name
  2,  // number of fields
  sizeof(lbr_fri_idl__msg__LBRTorqueCommand),
  lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_member_array,  // message members
  lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_type_support_handle = {
  0,
  &lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lbr_fri_idl
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lbr_fri_idl, msg, LBRTorqueCommand)() {
  if (!lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_type_support_handle.typesupport_identifier) {
    lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lbr_fri_idl__msg__LBRTorqueCommand__rosidl_typesupport_introspection_c__LBRTorqueCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
