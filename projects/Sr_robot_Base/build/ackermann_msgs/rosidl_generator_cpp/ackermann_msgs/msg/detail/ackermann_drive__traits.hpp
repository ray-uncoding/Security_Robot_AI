// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ackermann_msgs/msg/detail/ackermann_drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ackermann_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AckermannDrive & msg,
  std::ostream & out)
{
  out << "{";
  // member: steering_angle
  {
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
    out << ", ";
  }

  // member: steering_angle_velocity
  {
    out << "steering_angle_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle_velocity, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: jerk
  {
    out << "jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.jerk, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AckermannDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: steering_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
    out << "\n";
  }

  // member: steering_angle_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_angle_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle_velocity, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
  }

  // member: jerk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.jerk, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AckermannDrive & msg, bool use_flow_style = false)
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

}  // namespace ackermann_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ackermann_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ackermann_msgs::msg::AckermannDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  ackermann_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ackermann_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ackermann_msgs::msg::AckermannDrive & msg)
{
  return ackermann_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ackermann_msgs::msg::AckermannDrive>()
{
  return "ackermann_msgs::msg::AckermannDrive";
}

template<>
inline const char * name<ackermann_msgs::msg::AckermannDrive>()
{
  return "ackermann_msgs/msg/AckermannDrive";
}

template<>
struct has_fixed_size<ackermann_msgs::msg::AckermannDrive>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ackermann_msgs::msg::AckermannDrive>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ackermann_msgs::msg::AckermannDrive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_
