// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "wheeltec_rrt_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "wheeltec_rrt_msg/srv/detail/delete_shape__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_serialize(
  const wheeltec_rrt_msg::srv::DeleteShape_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wheeltec_rrt_msg::srv::DeleteShape_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
get_serialized_size(
  const wheeltec_rrt_msg::srv::DeleteShape_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
max_serialized_size_DeleteShape_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace wheeltec_rrt_msg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, srv, DeleteShape_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "wheeltec_rrt_msg/srv/detail/delete_shape__struct.hpp"

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

// already included above
// #include "fastcdr/Cdr.h"

namespace wheeltec_rrt_msg
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_serialize(
  const wheeltec_rrt_msg::srv::DeleteShape_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wheeltec_rrt_msg::srv::DeleteShape_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
get_serialized_size(
  const wheeltec_rrt_msg::srv::DeleteShape_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
max_serialized_size_DeleteShape_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace wheeltec_rrt_msg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, srv, DeleteShape_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wheeltec_rrt_msg
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, srv, DeleteShape)();

#ifdef __cplusplus
}
#endif

#endif  // WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
