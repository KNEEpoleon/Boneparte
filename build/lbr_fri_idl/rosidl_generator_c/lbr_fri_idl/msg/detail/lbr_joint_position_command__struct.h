// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lbr_fri_idl:msg/LBRJointPositionCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__STRUCT_H_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LBRJointPositionCommand in the package lbr_fri_idl.
/**
  * Position command for the Fast Robot Interface (FRI).
  * Refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation.
  *
  *  * joint_position
 */
typedef struct lbr_fri_idl__msg__LBRJointPositionCommand
{
  double joint_position[7];
} lbr_fri_idl__msg__LBRJointPositionCommand;

// Struct for a sequence of lbr_fri_idl__msg__LBRJointPositionCommand.
typedef struct lbr_fri_idl__msg__LBRJointPositionCommand__Sequence
{
  lbr_fri_idl__msg__LBRJointPositionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lbr_fri_idl__msg__LBRJointPositionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_JOINT_POSITION_COMMAND__STRUCT_H_
