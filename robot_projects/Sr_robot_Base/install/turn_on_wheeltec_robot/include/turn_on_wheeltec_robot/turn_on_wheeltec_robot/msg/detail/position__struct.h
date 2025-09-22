// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_H_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Position in the package turn_on_wheeltec_robot.
typedef struct turn_on_wheeltec_robot__msg__Position
{
  float angle_x;
  float angle_y;
  float distance;
} turn_on_wheeltec_robot__msg__Position;

// Struct for a sequence of turn_on_wheeltec_robot__msg__Position.
typedef struct turn_on_wheeltec_robot__msg__Position__Sequence
{
  turn_on_wheeltec_robot__msg__Position * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turn_on_wheeltec_robot__msg__Position__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_H_
