// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "turn_on_wheeltec_robot/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "turn_on_wheeltec_robot/msg/detail/position__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace turn_on_wheeltec_robot
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
cdr_serialize(
  const turn_on_wheeltec_robot::msg::Position & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  turn_on_wheeltec_robot::msg::Position & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
get_serialized_size(
  const turn_on_wheeltec_robot::msg::Position & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
max_serialized_size_Position(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace turn_on_wheeltec_robot

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turn_on_wheeltec_robot, msg, Position)();

#ifdef __cplusplus
}
#endif

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
