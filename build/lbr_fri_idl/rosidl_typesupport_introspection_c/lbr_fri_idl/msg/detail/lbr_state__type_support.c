// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lbr_fri_idl/msg/detail/lbr_state__rosidl_typesupport_introspection_c.h"
#include "lbr_fri_idl/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lbr_fri_idl/msg/detail/lbr_state__functions.h"
#include "lbr_fri_idl/msg/detail/lbr_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lbr_fri_idl__msg__LBRState__init(message_memory);
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_fini_function(void * message_memory)
{
  lbr_fri_idl__msg__LBRState__fini(message_memory);
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__measured_joint_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_joint_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_joint_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__measured_joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_joint_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__measured_joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_joint_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__commanded_joint_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_joint_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_joint_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__commanded_joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_joint_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__commanded_joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_joint_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__measured_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__measured_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__measured_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__commanded_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__commanded_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__commanded_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__external_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__external_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__external_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__external_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__external_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__external_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__external_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__ipo_joint_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__ipo_joint_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__ipo_joint_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__ipo_joint_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__ipo_joint_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__ipo_joint_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__ipo_joint_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_member_array[18] = {
  {
    "sample_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, sample_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, session_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "connection_quality",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, connection_quality),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, safety_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operation_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, operation_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "drive_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, drive_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "client_command_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, client_command_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "overlay_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, overlay_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, control_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_stamp_sec",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, time_stamp_sec),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_stamp_nano_sec",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, time_stamp_nano_sec),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measured_joint_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, measured_joint_position),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__measured_joint_position,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_joint_position,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_joint_position,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__measured_joint_position,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__measured_joint_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "commanded_joint_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, commanded_joint_position),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__commanded_joint_position,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_joint_position,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_joint_position,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__commanded_joint_position,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__commanded_joint_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measured_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, measured_torque),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__measured_torque,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__measured_torque,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__measured_torque,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__measured_torque,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__measured_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "commanded_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, commanded_torque),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__commanded_torque,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__commanded_torque,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__commanded_torque,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__commanded_torque,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__commanded_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "external_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, external_torque),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__external_torque,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__external_torque,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__external_torque,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__external_torque,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__external_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ipo_joint_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, ipo_joint_position),  // bytes offset in struct
    NULL,  // default value
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__size_function__LBRState__ipo_joint_position,  // size() function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_const_function__LBRState__ipo_joint_position,  // get_const(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__get_function__LBRState__ipo_joint_position,  // get(index) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__fetch_function__LBRState__ipo_joint_position,  // fetch(index, &value) function pointer
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__assign_function__LBRState__ipo_joint_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking_performance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lbr_fri_idl__msg__LBRState, tracking_performance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_members = {
  "lbr_fri_idl__msg",  // message namespace
  "LBRState",  // message name
  18,  // number of fields
  sizeof(lbr_fri_idl__msg__LBRState),
  lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_member_array,  // message members
  lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_init_function,  // function to initialize message memory (memory has to be allocated)
  lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_type_support_handle = {
  0,
  &lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lbr_fri_idl
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lbr_fri_idl, msg, LBRState)() {
  if (!lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_type_support_handle.typesupport_identifier) {
    lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lbr_fri_idl__msg__LBRState__rosidl_typesupport_introspection_c__LBRState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
