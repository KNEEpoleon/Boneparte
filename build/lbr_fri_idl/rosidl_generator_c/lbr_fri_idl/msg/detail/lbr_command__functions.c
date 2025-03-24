// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lbr_fri_idl:msg/LBRCommand.idl
// generated code does not contain a copyright notice
#include "lbr_fri_idl/msg/detail/lbr_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
lbr_fri_idl__msg__LBRCommand__init(lbr_fri_idl__msg__LBRCommand * msg)
{
  if (!msg) {
    return false;
  }
  // joint_position
  // torque
  // wrench
  return true;
}

void
lbr_fri_idl__msg__LBRCommand__fini(lbr_fri_idl__msg__LBRCommand * msg)
{
  if (!msg) {
    return;
  }
  // joint_position
  // torque
  // wrench
}

bool
lbr_fri_idl__msg__LBRCommand__are_equal(const lbr_fri_idl__msg__LBRCommand * lhs, const lbr_fri_idl__msg__LBRCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_position
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->joint_position[i] != rhs->joint_position[i]) {
      return false;
    }
  }
  // torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->torque[i] != rhs->torque[i]) {
      return false;
    }
  }
  // wrench
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->wrench[i] != rhs->wrench[i]) {
      return false;
    }
  }
  return true;
}

bool
lbr_fri_idl__msg__LBRCommand__copy(
  const lbr_fri_idl__msg__LBRCommand * input,
  lbr_fri_idl__msg__LBRCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_position
  for (size_t i = 0; i < 7; ++i) {
    output->joint_position[i] = input->joint_position[i];
  }
  // torque
  for (size_t i = 0; i < 7; ++i) {
    output->torque[i] = input->torque[i];
  }
  // wrench
  for (size_t i = 0; i < 6; ++i) {
    output->wrench[i] = input->wrench[i];
  }
  return true;
}

lbr_fri_idl__msg__LBRCommand *
lbr_fri_idl__msg__LBRCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lbr_fri_idl__msg__LBRCommand * msg = (lbr_fri_idl__msg__LBRCommand *)allocator.allocate(sizeof(lbr_fri_idl__msg__LBRCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lbr_fri_idl__msg__LBRCommand));
  bool success = lbr_fri_idl__msg__LBRCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lbr_fri_idl__msg__LBRCommand__destroy(lbr_fri_idl__msg__LBRCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lbr_fri_idl__msg__LBRCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lbr_fri_idl__msg__LBRCommand__Sequence__init(lbr_fri_idl__msg__LBRCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lbr_fri_idl__msg__LBRCommand * data = NULL;

  if (size) {
    data = (lbr_fri_idl__msg__LBRCommand *)allocator.zero_allocate(size, sizeof(lbr_fri_idl__msg__LBRCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lbr_fri_idl__msg__LBRCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lbr_fri_idl__msg__LBRCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lbr_fri_idl__msg__LBRCommand__Sequence__fini(lbr_fri_idl__msg__LBRCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lbr_fri_idl__msg__LBRCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lbr_fri_idl__msg__LBRCommand__Sequence *
lbr_fri_idl__msg__LBRCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lbr_fri_idl__msg__LBRCommand__Sequence * array = (lbr_fri_idl__msg__LBRCommand__Sequence *)allocator.allocate(sizeof(lbr_fri_idl__msg__LBRCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lbr_fri_idl__msg__LBRCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lbr_fri_idl__msg__LBRCommand__Sequence__destroy(lbr_fri_idl__msg__LBRCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lbr_fri_idl__msg__LBRCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lbr_fri_idl__msg__LBRCommand__Sequence__are_equal(const lbr_fri_idl__msg__LBRCommand__Sequence * lhs, const lbr_fri_idl__msg__LBRCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lbr_fri_idl__msg__LBRCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lbr_fri_idl__msg__LBRCommand__Sequence__copy(
  const lbr_fri_idl__msg__LBRCommand__Sequence * input,
  lbr_fri_idl__msg__LBRCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lbr_fri_idl__msg__LBRCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lbr_fri_idl__msg__LBRCommand * data =
      (lbr_fri_idl__msg__LBRCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lbr_fri_idl__msg__LBRCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lbr_fri_idl__msg__LBRCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lbr_fri_idl__msg__LBRCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
