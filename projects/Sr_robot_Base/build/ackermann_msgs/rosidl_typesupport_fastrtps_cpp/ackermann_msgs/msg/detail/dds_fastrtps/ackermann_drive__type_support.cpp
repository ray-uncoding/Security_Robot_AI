// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice
#include "ackermann_msgs/msg/detail/ackermann_drive__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ackermann_msgs/msg/detail/ackermann_drive__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace ackermann_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ackermann_msgs
cdr_serialize(
  const ackermann_msgs::msg::AckermannDrive & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: steering_angle
  cdr << ros_message.steering_angle;
  // Member: steering_angle_velocity
  cdr << ros_message.steering_angle_velocity;
  // Member: speed
  cdr << ros_message.speed;
  // Member: acceleration
  cdr << ros_message.acceleration;
  // Member: jerk
  cdr << ros_message.jerk;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ackermann_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ackermann_msgs::msg::AckermannDrive & ros_message)
{
  // Member: steering_angle
  cdr >> ros_message.steering_angle;

  // Member: steering_angle_velocity
  cdr >> ros_message.steering_angle_velocity;

  // Member: speed
  cdr >> ros_message.speed;

  // Member: acceleration
  cdr >> ros_message.acceleration;

  // Member: jerk
  cdr >> ros_message.jerk;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ackermann_msgs
get_serialized_size(
  const ackermann_msgs::msg::AckermannDrive & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: steering_angle
  {
    size_t item_size = sizeof(ros_message.steering_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: steering_angle_velocity
  {
    size_t item_size = sizeof(ros_message.steering_angle_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acceleration
  {
    size_t item_size = sizeof(ros_message.acceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: jerk
  {
    size_t item_size = sizeof(ros_message.jerk);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ackermann_msgs
max_serialized_size_AckermannDrive(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: steering_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: steering_angle_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acceleration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: jerk
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ackermann_msgs::msg::AckermannDrive;
    is_plain =
      (
      offsetof(DataType, jerk) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AckermannDrive__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ackermann_msgs::msg::AckermannDrive *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AckermannDrive__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ackermann_msgs::msg::AckermannDrive *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AckermannDrive__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ackermann_msgs::msg::AckermannDrive *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AckermannDrive__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AckermannDrive(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AckermannDrive__callbacks = {
  "ackermann_msgs::msg",
  "AckermannDrive",
  _AckermannDrive__cdr_serialize,
  _AckermannDrive__cdr_deserialize,
  _AckermannDrive__get_serialized_size,
  _AckermannDrive__max_serialized_size
};

static rosidl_message_type_support_t _AckermannDrive__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AckermannDrive__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ackermann_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ackermann_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ackermann_msgs::msg::AckermannDrive>()
{
  return &ackermann_msgs::msg::typesupport_fastrtps_cpp::_AckermannDrive__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ackermann_msgs, msg, AckermannDrive)() {
  return &ackermann_msgs::msg::typesupport_fastrtps_cpp::_AckermannDrive__handle;
}

#ifdef __cplusplus
}
#endif
