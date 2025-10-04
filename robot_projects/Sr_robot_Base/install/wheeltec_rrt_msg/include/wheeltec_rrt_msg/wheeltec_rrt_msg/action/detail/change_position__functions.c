// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheeltec_rrt_msg:action/ChangePosition.idl
// generated code does not contain a copyright notice
#include "wheeltec_rrt_msg/action/detail/change_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
wheeltec_rrt_msg__action__ChangePosition_Goal__init(wheeltec_rrt_msg__action__ChangePosition_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // desired_x
  // desired_y
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_Goal__fini(wheeltec_rrt_msg__action__ChangePosition_Goal * msg)
{
  if (!msg) {
    return;
  }
  // desired_x
  // desired_y
}

bool
wheeltec_rrt_msg__action__ChangePosition_Goal__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Goal * lhs, const wheeltec_rrt_msg__action__ChangePosition_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // desired_x
  if (lhs->desired_x != rhs->desired_x) {
    return false;
  }
  // desired_y
  if (lhs->desired_y != rhs->desired_y) {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Goal__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Goal * input,
  wheeltec_rrt_msg__action__ChangePosition_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // desired_x
  output->desired_x = input->desired_x;
  // desired_y
  output->desired_y = input->desired_y;
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_Goal *
wheeltec_rrt_msg__action__ChangePosition_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Goal * msg = (wheeltec_rrt_msg__action__ChangePosition_Goal *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_Goal));
  bool success = wheeltec_rrt_msg__action__ChangePosition_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_Goal__destroy(wheeltec_rrt_msg__action__ChangePosition_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Goal * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_Goal *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_Goal__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_Goal__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence *
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_Goal * data =
      (wheeltec_rrt_msg__action__ChangePosition_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
wheeltec_rrt_msg__action__ChangePosition_Result__init(wheeltec_rrt_msg__action__ChangePosition_Result * msg)
{
  if (!msg) {
    return false;
  }
  // is_complete
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_Result__fini(wheeltec_rrt_msg__action__ChangePosition_Result * msg)
{
  if (!msg) {
    return;
  }
  // is_complete
}

bool
wheeltec_rrt_msg__action__ChangePosition_Result__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Result * lhs, const wheeltec_rrt_msg__action__ChangePosition_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_complete
  if (lhs->is_complete != rhs->is_complete) {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Result__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Result * input,
  wheeltec_rrt_msg__action__ChangePosition_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // is_complete
  output->is_complete = input->is_complete;
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_Result *
wheeltec_rrt_msg__action__ChangePosition_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Result * msg = (wheeltec_rrt_msg__action__ChangePosition_Result *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_Result));
  bool success = wheeltec_rrt_msg__action__ChangePosition_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_Result__destroy(wheeltec_rrt_msg__action__ChangePosition_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Result * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_Result *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_Result__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_Result__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_Result__Sequence *
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_Result__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Result__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_Result * data =
      (wheeltec_rrt_msg__action__ChangePosition_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__init(wheeltec_rrt_msg__action__ChangePosition_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_x
  // current_y
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(wheeltec_rrt_msg__action__ChangePosition_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_x
  // current_y
}

bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Feedback * lhs, const wheeltec_rrt_msg__action__ChangePosition_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_x
  if (lhs->current_x != rhs->current_x) {
    return false;
  }
  // current_y
  if (lhs->current_y != rhs->current_y) {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Feedback * input,
  wheeltec_rrt_msg__action__ChangePosition_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_x
  output->current_x = input->current_x;
  // current_y
  output->current_y = input->current_y;
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_Feedback *
wheeltec_rrt_msg__action__ChangePosition_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Feedback * msg = (wheeltec_rrt_msg__action__ChangePosition_Feedback *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_Feedback));
  bool success = wheeltec_rrt_msg__action__ChangePosition_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_Feedback__destroy(wheeltec_rrt_msg__action__ChangePosition_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Feedback * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_Feedback *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence *
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_Feedback * data =
      (wheeltec_rrt_msg__action__ChangePosition_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__functions.h"

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__init(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!wheeltec_rrt_msg__action__ChangePosition_Goal__init(&msg->goal)) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  wheeltec_rrt_msg__action__ChangePosition_Goal__fini(&msg->goal);
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__are_equal(const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * lhs, const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!wheeltec_rrt_msg__action__ChangePosition_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__copy(
  const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * input,
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!wheeltec_rrt_msg__action__ChangePosition_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request *
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * msg = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request));
  bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__destroy(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence *
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request * data =
      (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__init(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__are_equal(const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * lhs, const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__copy(
  const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * input,
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response *
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * msg = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response));
  bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__destroy(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence *
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response * data =
      (wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__init(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__are_equal(const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * lhs, const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__copy(
  const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * input,
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_GetResult_Request *
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * msg = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Request *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request));
  bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__destroy(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Request *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence *
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Request * data =
      (wheeltec_rrt_msg__action__ChangePosition_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__functions.h"

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__init(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!wheeltec_rrt_msg__action__ChangePosition_Result__init(&msg->result)) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  wheeltec_rrt_msg__action__ChangePosition_Result__fini(&msg->result);
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__are_equal(const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * lhs, const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!wheeltec_rrt_msg__action__ChangePosition_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__copy(
  const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * input,
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!wheeltec_rrt_msg__action__ChangePosition_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_GetResult_Response *
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * msg = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Response *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response));
  bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__destroy(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Response *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence *
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_GetResult_Response * data =
      (wheeltec_rrt_msg__action__ChangePosition_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__functions.h"

bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__init(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__init(&msg->feedback)) {
    wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  wheeltec_rrt_msg__action__ChangePosition_Feedback__fini(&msg->feedback);
}

bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__are_equal(const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * lhs, const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__copy(
  const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * input,
  wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!wheeltec_rrt_msg__action__ChangePosition_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage *
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * msg = (wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage));
  bool success = wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__destroy(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__init(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * data = NULL;

  if (size) {
    data = (wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage *)allocator.zero_allocate(size, sizeof(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(&data[i - 1]);
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
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__fini(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * array)
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
      wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(&array->data[i]);
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

wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence *
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * array = (wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence *)allocator.allocate(sizeof(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__destroy(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__are_equal(const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * lhs, const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence__copy(
  const wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * input,
  wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage * data =
      (wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_rrt_msg__action__ChangePosition_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
