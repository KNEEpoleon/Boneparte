// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#ifndef LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__FUNCTIONS_H_
#define LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lbr_fri_idl/msg/rosidl_generator_c__visibility_control.h"

#include "lbr_fri_idl/msg/detail/lbr_state__struct.h"

/// Initialize msg/LBRState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lbr_fri_idl__msg__LBRState
 * )) before or use
 * lbr_fri_idl__msg__LBRState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRState__init(lbr_fri_idl__msg__LBRState * msg);

/// Finalize msg/LBRState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRState__fini(lbr_fri_idl__msg__LBRState * msg);

/// Create msg/LBRState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lbr_fri_idl__msg__LBRState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
lbr_fri_idl__msg__LBRState *
lbr_fri_idl__msg__LBRState__create();

/// Destroy msg/LBRState message.
/**
 * It calls
 * lbr_fri_idl__msg__LBRState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRState__destroy(lbr_fri_idl__msg__LBRState * msg);

/// Check for msg/LBRState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRState__are_equal(const lbr_fri_idl__msg__LBRState * lhs, const lbr_fri_idl__msg__LBRState * rhs);

/// Copy a msg/LBRState message.
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
lbr_fri_idl__msg__LBRState__copy(
  const lbr_fri_idl__msg__LBRState * input,
  lbr_fri_idl__msg__LBRState * output);

/// Initialize array of msg/LBRState messages.
/**
 * It allocates the memory for the number of elements and calls
 * lbr_fri_idl__msg__LBRState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRState__Sequence__init(lbr_fri_idl__msg__LBRState__Sequence * array, size_t size);

/// Finalize array of msg/LBRState messages.
/**
 * It calls
 * lbr_fri_idl__msg__LBRState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRState__Sequence__fini(lbr_fri_idl__msg__LBRState__Sequence * array);

/// Create array of msg/LBRState messages.
/**
 * It allocates the memory for the array and calls
 * lbr_fri_idl__msg__LBRState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
lbr_fri_idl__msg__LBRState__Sequence *
lbr_fri_idl__msg__LBRState__Sequence__create(size_t size);

/// Destroy array of msg/LBRState messages.
/**
 * It calls
 * lbr_fri_idl__msg__LBRState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
void
lbr_fri_idl__msg__LBRState__Sequence__destroy(lbr_fri_idl__msg__LBRState__Sequence * array);

/// Check for msg/LBRState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
bool
lbr_fri_idl__msg__LBRState__Sequence__are_equal(const lbr_fri_idl__msg__LBRState__Sequence * lhs, const lbr_fri_idl__msg__LBRState__Sequence * rhs);

/// Copy an array of msg/LBRState messages.
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
lbr_fri_idl__msg__LBRState__Sequence__copy(
  const lbr_fri_idl__msg__LBRState__Sequence * input,
  lbr_fri_idl__msg__LBRState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LBR_FRI_IDL__MSG__DETAIL__LBR_STATE__FUNCTIONS_H_
