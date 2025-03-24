// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lbr_fri_idl:msg/LBRCommand.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_COMMAND__FUNCTIONS_H_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lbr_fri_idl/msg/rosidl_generator_c__visibility_control.h"

#include "lbr_fri_idl/msg/detail/lbr_command__struct.h"

/// Initialize msg/LBRCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lbr_fri_idl__msg__LBRCommand
 * )) before or use
 * lbr_fri_idl__msg__LBRCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__init(lbr_fri_idl__msg__LBRCommand * msg);

/// Finalize msg/LBRCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRCommand__fini(lbr_fri_idl__msg__LBRCommand * msg);

/// Create msg/LBRCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lbr_fri_idl__msg__LBRCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
lbr_fri_idl__msg__LBRCommand *
lbr_fri_idl__msg__LBRCommand__create();

/// Destroy msg/LBRCommand message.
/**
 * It calls
 * lbr_fri_idl__msg__LBRCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRCommand__destroy(lbr_fri_idl__msg__LBRCommand * msg);

/// Check for msg/LBRCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__are_equal(const lbr_fri_idl__msg__LBRCommand * lhs, const lbr_fri_idl__msg__LBRCommand * rhs);

/// Copy a msg/LBRCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__copy(
  const lbr_fri_idl__msg__LBRCommand * input,
  lbr_fri_idl__msg__LBRCommand * output);

/// Initialize array of msg/LBRCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * lbr_fri_idl__msg__LBRCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__Sequence__init(lbr_fri_idl__msg__LBRCommand__Sequence * array, size_t size);

/// Finalize array of msg/LBRCommand messages.
/**
 * It calls
 * lbr_fri_idl__msg__LBRCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRCommand__Sequence__fini(lbr_fri_idl__msg__LBRCommand__Sequence * array);

/// Create array of msg/LBRCommand messages.
/**
 * It allocates the memory for the array and calls
 * lbr_fri_idl__msg__LBRCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
lbr_fri_idl__msg__LBRCommand__Sequence *
lbr_fri_idl__msg__LBRCommand__Sequence__create(size_t size);

/// Destroy array of msg/LBRCommand messages.
/**
 * It calls
 * lbr_fri_idl__msg__LBRCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRCommand__Sequence__destroy(lbr_fri_idl__msg__LBRCommand__Sequence * array);

/// Check for msg/LBRCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__Sequence__are_equal(const lbr_fri_idl__msg__LBRCommand__Sequence * lhs, const lbr_fri_idl__msg__LBRCommand__Sequence * rhs);

/// Copy an array of msg/LBRCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRCommand__Sequence__copy(
  const lbr_fri_idl__msg__LBRCommand__Sequence * input,
  lbr_fri_idl__msg__LBRCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_COMMAND__FUNCTIONS_H_
