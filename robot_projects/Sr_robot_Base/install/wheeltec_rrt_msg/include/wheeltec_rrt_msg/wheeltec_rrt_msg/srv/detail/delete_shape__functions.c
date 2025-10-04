// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice
#include "wheeltec_rrt_msg/srv/detail/delete_shape__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `shape_type`
#include "rosidl_runtime_c/string_functions.h"

bool
wheeltec_rrt_msg__srv__DeleteShape_Request__init(wheeltec_rrt_msg__srv__DeleteShape_Request * msg)
{
  if (!msg) {
    return false;
  }
  // shape_type
  if (!rosidl_runtime_c__String__init(&msg->shape_type)) {
    wheeltec_rrt_msg__srv__DeleteShape_Request__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Request__fini(wheeltec_rrt_msg__srv__DeleteShape_Request * msg)
{
  if (!msg) {
    return;
  }
  // shape_type
  rosidl_runtime_c__String__fini(&msg->shape_type);
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Request__are_equal(const wheeltec_rrt_msg__srv__DeleteShape_Request * lhs, const wheeltec_rrt_msg__srv__DeleteShape_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // shape_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->shape_type), &(rhs->shape_type)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Request__copy(
  const wheeltec_rrt_msg__srv__DeleteShape_Request * input,
  wheeltec_rrt_msg__srv__DeleteShape_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // shape_type
  if (!rosidl_runtime_c__String__copy(
      &(input->shape_type), &(output->shape_type)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__srv__DeleteShape_Request *
wheeltec_rrt_msg__srv__DeleteShape_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Request * msg = (wheeltec_rrt_msg__srv__DeleteShape_Request *)allocator.allocate(sizeof(wheeltec_rrt_msg__srv__DeleteShape_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__srv__DeleteShape_Request));
  bool success = wheeltec_rrt_msg__srv__DeleteShape_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Request__destroy(wheeltec_rrt_msg__srv__DeleteShape_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__srv__DeleteShape_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__init(wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Request * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__srv__DeleteShape_Request *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__srv__DeleteShape_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__srv__DeleteShape_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__srv__DeleteShape_Request__fini(&data[i - 1]);
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
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__fini(wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * array)
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
      wheeltec_rrt_msg__srv__DeleteShape_Request__fini(&array->data[i]);
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

wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence *
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * array = (wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__destroy(wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__are_equal(const wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * lhs, const wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__srv__DeleteShape_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence__copy(
  const wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * input,
  wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__srv__DeleteShape_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__srv__DeleteShape_Request * data =
      (wheeltec_rrt_msg__srv__DeleteShape_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__srv__DeleteShape_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__srv__DeleteShape_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__srv__DeleteShape_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
wheeltec_rrt_msg__srv__DeleteShape_Response__init(wheeltec_rrt_msg__srv__DeleteShape_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Response__fini(wheeltec_rrt_msg__srv__DeleteShape_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Response__are_equal(const wheeltec_rrt_msg__srv__DeleteShape_Response * lhs, const wheeltec_rrt_msg__srv__DeleteShape_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Response__copy(
  const wheeltec_rrt_msg__srv__DeleteShape_Response * input,
  wheeltec_rrt_msg__srv__DeleteShape_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

wheeltec_rrt_msg__srv__DeleteShape_Response *
wheeltec_rrt_msg__srv__DeleteShape_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Response * msg = (wheeltec_rrt_msg__srv__DeleteShape_Response *)allocator.allocate(sizeof(wheeltec_rrt_msg__srv__DeleteShape_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__srv__DeleteShape_Response));
  bool success = wheeltec_rrt_msg__srv__DeleteShape_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Response__destroy(wheeltec_rrt_msg__srv__DeleteShape_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__srv__DeleteShape_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__init(wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Response * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__srv__DeleteShape_Response *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__srv__DeleteShape_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__srv__DeleteShape_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__srv__DeleteShape_Response__fini(&data[i - 1]);
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
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__fini(wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * array)
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
      wheeltec_rrt_msg__srv__DeleteShape_Response__fini(&array->data[i]);
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

wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence *
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * array = (wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__destroy(wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__are_equal(const wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * lhs, const wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__srv__DeleteShape_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence__copy(
  const wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * input,
  wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__srv__DeleteShape_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__srv__DeleteShape_Response * data =
      (wheeltec_rrt_msg__srv__DeleteShape_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__srv__DeleteShape_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__srv__DeleteShape_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__srv__DeleteShape_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
