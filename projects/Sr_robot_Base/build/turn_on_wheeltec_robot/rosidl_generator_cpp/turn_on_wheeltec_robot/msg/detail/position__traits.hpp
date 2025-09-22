// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turn_on_wheeltec_robot/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turn_on_wheeltec_robot
{

namespace msg
{

inline void to_flow_style_yaml(
  const Position & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle_x
  {
    out << "angle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_x, out);
    out << ", ";
  }

  // member: angle_y
  {
    out << "angle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_y, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_x, out);
    out << "\n";
  }

  // member: angle_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_y, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace turn_on_wheeltec_robot

namespace rosidl_generator_traits
{

[[deprecated("use turn_on_wheeltec_robot::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turn_on_wheeltec_robot::msg::Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  turn_on_wheeltec_robot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turn_on_wheeltec_robot::msg::to_yaml() instead")]]
inline std::string to_yaml(const turn_on_wheeltec_robot::msg::Position & msg)
{
  return turn_on_wheeltec_robot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turn_on_wheeltec_robot::msg::Position>()
{
  return "turn_on_wheeltec_robot::msg::Position";
}

template<>
inline const char * name<turn_on_wheeltec_robot::msg::Position>()
{
  return "turn_on_wheeltec_robot/msg/Position";
}

template<>
struct has_fixed_size<turn_on_wheeltec_robot::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turn_on_wheeltec_robot::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turn_on_wheeltec_robot::msg::Position>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_
