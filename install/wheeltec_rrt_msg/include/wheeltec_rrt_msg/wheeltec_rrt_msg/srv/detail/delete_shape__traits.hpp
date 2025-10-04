// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__TRAITS_HPP_
#define WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheeltec_rrt_msg/srv/detail/delete_shape__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace wheeltec_rrt_msg
{

namespace srv
{

inline void to_flow_style_yaml(
  const DeleteShape_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: shape_type
  {
    out << "shape_type: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DeleteShape_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: shape_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape_type: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DeleteShape_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::srv::DeleteShape_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::srv::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::srv::DeleteShape_Request & msg)
{
  return wheeltec_rrt_msg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::srv::DeleteShape_Request>()
{
  return "wheeltec_rrt_msg::srv::DeleteShape_Request";
}

template<>
inline const char * name<wheeltec_rrt_msg::srv::DeleteShape_Request>()
{
  return "wheeltec_rrt_msg/srv/DeleteShape_Request";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::srv::DeleteShape_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::srv::DeleteShape_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wheeltec_rrt_msg::srv::DeleteShape_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace wheeltec_rrt_msg
{

namespace srv
{

inline void to_flow_style_yaml(
  const DeleteShape_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DeleteShape_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DeleteShape_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::srv::DeleteShape_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::srv::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::srv::DeleteShape_Response & msg)
{
  return wheeltec_rrt_msg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::srv::DeleteShape_Response>()
{
  return "wheeltec_rrt_msg::srv::DeleteShape_Response";
}

template<>
inline const char * name<wheeltec_rrt_msg::srv::DeleteShape_Response>()
{
  return "wheeltec_rrt_msg/srv/DeleteShape_Response";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::srv::DeleteShape_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::srv::DeleteShape_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wheeltec_rrt_msg::srv::DeleteShape_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<wheeltec_rrt_msg::srv::DeleteShape>()
{
  return "wheeltec_rrt_msg::srv::DeleteShape";
}

template<>
inline const char * name<wheeltec_rrt_msg::srv::DeleteShape>()
{
  return "wheeltec_rrt_msg/srv/DeleteShape";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::srv::DeleteShape>
  : std::integral_constant<
    bool,
    has_fixed_size<wheeltec_rrt_msg::srv::DeleteShape_Request>::value &&
    has_fixed_size<wheeltec_rrt_msg::srv::DeleteShape_Response>::value
  >
{
};

template<>
struct has_bounded_size<wheeltec_rrt_msg::srv::DeleteShape>
  : std::integral_constant<
    bool,
    has_bounded_size<wheeltec_rrt_msg::srv::DeleteShape_Request>::value &&
    has_bounded_size<wheeltec_rrt_msg::srv::DeleteShape_Response>::value
  >
{
};

template<>
struct is_service<wheeltec_rrt_msg::srv::DeleteShape>
  : std::true_type
{
};

template<>
struct is_service_request<wheeltec_rrt_msg::srv::DeleteShape_Request>
  : std::true_type
{
};

template<>
struct is_service_response<wheeltec_rrt_msg::srv::DeleteShape_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__TRAITS_HPP_
