// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice
#include "turn_on_wheeltec_robot/msg/detail/position__rosidl_typesupport_fastrtps_cpp.hpp"
#include "turn_on_wheeltec_robot/msg/detail/position__struct.hpp"

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: angle_x
  cdr << ros_message.angle_x;
  // Member: angle_y
  cdr << ros_message.angle_y;
  // Member: distance
  cdr << ros_message.distance;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  turn_on_wheeltec_robot::msg::Position & ros_message)
{
  // Member: angle_x
  cdr >> ros_message.angle_x;

  // Member: angle_y
  cdr >> ros_message.angle_y;

  // Member: distance
  cdr >> ros_message.distance;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
get_serialized_size(
  const turn_on_wheeltec_robot::msg::Position & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: angle_x
  {
    size_t item_size = sizeof(ros_message.angle_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: angle_y
  {
    size_t item_size = sizeof(ros_message.angle_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance
  {
    size_t item_size = sizeof(ros_message.distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turn_on_wheeltec_robot
max_serialized_size_Position(
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


  // Member: angle_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: angle_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance
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
    using DataType = turn_on_wheeltec_robot::msg::Position;
    is_plain =
      (
      offsetof(DataType, distance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Position__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const turn_on_wheeltec_robot::msg::Position *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Position__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<turn_on_wheeltec_robot::msg::Position *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Position__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const turn_on_wheeltec_robot::msg::Position *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Position__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Position(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Position__callbacks = {
  "turn_on_wheeltec_robot::msg",
  "Position",
  _Position__cdr_serialize,
  _Position__cdr_deserialize,
  _Position__get_serialized_size,
  _Position__max_serialized_size
};

static rosidl_message_type_support_t _Position__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Position__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace turn_on_wheeltec_robot

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_turn_on_wheeltec_robot
const rosidl_message_type_support_t *
get_message_type_support_handle<turn_on_wheeltec_robot::msg::Position>()
{
  return &turn_on_wheeltec_robot::msg::typesupport_fastrtps_cpp::_Position__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turn_on_wheeltec_robot, msg, Position)() {
  return &turn_on_wheeltec_robot::msg::typesupport_fastrtps_cpp::_Position__handle;
}

#ifdef __cplusplus
}
#endif
