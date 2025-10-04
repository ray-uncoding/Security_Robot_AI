// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_H_
#define WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'shape_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DeleteShape in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__srv__DeleteShape_Request
{
  rosidl_runtime_c__String shape_type;
} wheeltec_rrt_msg__srv__DeleteShape_Request;

// Struct for a sequence of wheeltec_rrt_msg__srv__DeleteShape_Request.
typedef struct wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence
{
  wheeltec_rrt_msg__srv__DeleteShape_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__srv__DeleteShape_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/DeleteShape in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__srv__DeleteShape_Response
{
  bool result;
} wheeltec_rrt_msg__srv__DeleteShape_Response;

// Struct for a sequence of wheeltec_rrt_msg__srv__DeleteShape_Response.
typedef struct wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence
{
  wheeltec_rrt_msg__srv__DeleteShape_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__srv__DeleteShape_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_H_
