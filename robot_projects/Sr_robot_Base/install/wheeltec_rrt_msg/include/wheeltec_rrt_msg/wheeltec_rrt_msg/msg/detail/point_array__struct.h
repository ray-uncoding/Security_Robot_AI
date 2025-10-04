// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheeltec_rrt_msg:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
#define WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/PointArray in the package wheeltec_rrt_msg.
typedef struct wheeltec_rrt_msg__msg__PointArray
{
  geometry_msgs__msg__Point__Sequence points;
} wheeltec_rrt_msg__msg__PointArray;

// Struct for a sequence of wheeltec_rrt_msg__msg__PointArray.
typedef struct wheeltec_rrt_msg__msg__PointArray__Sequence
{
  wheeltec_rrt_msg__msg__PointArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheeltec_rrt_msg__msg__PointArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
