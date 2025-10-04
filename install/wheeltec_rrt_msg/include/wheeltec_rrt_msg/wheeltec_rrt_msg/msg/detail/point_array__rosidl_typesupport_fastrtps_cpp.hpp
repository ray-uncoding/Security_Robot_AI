// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from wheeltec_rrt_msg:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "wheeltec_rrt_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "wheeltec_rrt_msg/msg/detail/point_array__struct.hpp"

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

namespace wheeltec_rrt_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_serialize(
  const wheeltec_rrt_msg::msg::PointArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wheeltec_rrt_msg::msg::PointArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
get_serialized_size(
  const wheeltec_rrt_msg::msg::PointArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
max_serialized_size_PointArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace wheeltec_rrt_msg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, msg, PointArray)();

#ifdef __cplusplus
}
#endif

#endif  // WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
