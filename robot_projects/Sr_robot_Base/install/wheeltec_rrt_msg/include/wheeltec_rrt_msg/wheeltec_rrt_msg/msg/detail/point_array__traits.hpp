// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheeltec_rrt_msg:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
#define WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheeltec_rrt_msg/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointArray & msg, bool use_flow_style = false)
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

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::msg::PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::msg::PointArray & msg)
{
  return wheeltec_rrt_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::msg::PointArray>()
{
  return "wheeltec_rrt_msg::msg::PointArray";
}

template<>
inline const char * name<wheeltec_rrt_msg::msg::PointArray>()
{
  return "wheeltec_rrt_msg/msg/PointArray";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wheeltec_rrt_msg::msg::PointArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
