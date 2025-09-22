// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__TRAITS_HPP_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ackermann_msgs/msg/detail/ackermann_drive_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'drive'
#include "ackermann_msgs/msg/detail/ackermann_drive__traits.hpp"

namespace ackermann_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AckermannDriveStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: drive
  {
    out << "drive: ";
    to_flow_style_yaml(msg.drive, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AckermannDriveStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: drive
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drive:\n";
    to_block_style_yaml(msg.drive, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AckermannDriveStamped & msg, bool use_flow_style = false)
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
  const ackermann_msgs::msg::AckermannDriveStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  ackermann_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ackermann_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ackermann_msgs::msg::AckermannDriveStamped & msg)
{
  return ackermann_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ackermann_msgs::msg::AckermannDriveStamped>()
{
  return "ackermann_msgs::msg::AckermannDriveStamped";
}

template<>
inline const char * name<ackermann_msgs::msg::AckermannDriveStamped>()
{
  return "ackermann_msgs/msg/AckermannDriveStamped";
}

template<>
struct has_fixed_size<ackermann_msgs::msg::AckermannDriveStamped>
  : std::integral_constant<bool, has_fixed_size<ackermann_msgs::msg::AckermannDrive>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ackermann_msgs::msg::AckermannDriveStamped>
  : std::integral_constant<bool, has_bounded_size<ackermann_msgs::msg::AckermannDrive>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ackermann_msgs::msg::AckermannDriveStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__TRAITS_HPP_
