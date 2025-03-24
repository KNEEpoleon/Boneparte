// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_H_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LBRState in the package lbr_fri_idl.
/**
  * Message that holds states that are exposed through the
  * Fast Robot Interface (FRI).
  * Following states are exposed (refer to KUKA::FRI::LBRState in friLBRState.h for documentation):
  *
  *  * sample_time [s]
  *  * session_state [#]
  *  * connection_quality [#]
  *  * safety_state [#]
  *  * operation_mode [#]
  *  * drive_state [#]: 
  *  * client_command_mode [#]
  *  * overlay_type [#]
  *  * control_mode [#]
  *
  *  * time_stamp_sec [s]
  *  * time_stamp_nano_sec [ns]
  *
  *  * measured_joint_position [rad]
  *  * commanded_joint_position [rad]
  *  * measured_torque [Nm]
  *  * commanded_torque [Nm]
  *  * external_torque [Nm]
  *  * ipo_joint_position [rad]
  *  * tracking_performance [a.u.]
 */
typedef struct lbr_fri_idl__msg__LBRState
{
  double sample_time;
  int8_t session_state;
  int8_t connection_quality;
  int8_t safety_state;
  int8_t operation_mode;
  int8_t drive_state;
  int8_t client_command_mode;
  int8_t overlay_type;
  int8_t control_mode;
  uint32_t time_stamp_sec;
  uint32_t time_stamp_nano_sec;
  double measured_joint_position[7];
  double commanded_joint_position[7];
  double measured_torque[7];
  double commanded_torque[7];
  double external_torque[7];
  double ipo_joint_position[7];
  double tracking_performance;
} lbr_fri_idl__msg__LBRState;

// Struct for a sequence of lbr_fri_idl__msg__LBRState.
typedef struct lbr_fri_idl__msg__LBRState__Sequence
{
  lbr_fri_idl__msg__LBRState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lbr_fri_idl__msg__LBRState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__STRUCT_H_
