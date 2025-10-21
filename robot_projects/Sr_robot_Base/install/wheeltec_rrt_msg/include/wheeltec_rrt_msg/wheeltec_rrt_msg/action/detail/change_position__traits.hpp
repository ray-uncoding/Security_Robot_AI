// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheeltec_rrt_msg:action/ChangePosition.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__TRAITS_HPP_
#define WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: desired_x
  {
    out << "desired_x: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_x, out);
    out << ", ";
  }

  // member: desired_y
  {
    out << "desired_y: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: desired_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_x: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_x, out);
    out << "\n";
  }

  // member: desired_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_y: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_Goal & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_Goal>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_Goal";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_Goal>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_Goal";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_complete
  {
    out << "is_complete: ";
    rosidl_generator_traits::value_to_yaml(msg.is_complete, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_complete
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_complete: ";
    rosidl_generator_traits::value_to_yaml(msg.is_complete, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_Result & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_Result>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_Result";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_Result>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_Result";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_x
  {
    out << "current_x: ";
    rosidl_generator_traits::value_to_yaml(msg.current_x, out);
    out << ", ";
  }

  // member: current_y
  {
    out << "current_y: ";
    rosidl_generator_traits::value_to_yaml(msg.current_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_x: ";
    rosidl_generator_traits::value_to_yaml(msg.current_x, out);
    out << "\n";
  }

  // member: current_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_y: ";
    rosidl_generator_traits::value_to_yaml(msg.current_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_Feedback & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_Feedback>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_Feedback";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_Feedback>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_Feedback";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "wheeltec_rrt_msg/action/detail/change_position__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_SendGoal_Request";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Goal>::value> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Goal>::value> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_SendGoal_Response";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_SendGoal>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_SendGoal";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_SendGoal>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_SendGoal";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>::value &&
    has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>::value &&
    has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<wheeltec_rrt_msg::action::ChangePosition_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_GetResult_Request & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_GetResult_Request";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_GetResult_Request";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_GetResult_Response & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_GetResult_Response";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_GetResult_Response";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Result>::value> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Result>::value> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_GetResult>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_GetResult";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_GetResult>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_GetResult";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>::value &&
    has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>::value &&
    has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>::value
  >
{
};

template<>
struct is_service<wheeltec_rrt_msg::action::ChangePosition_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__traits.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const ChangePosition_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangePosition_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangePosition_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_generator_traits
{

[[deprecated("use wheeltec_rrt_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheeltec_rrt_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheeltec_rrt_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage & msg)
{
  return wheeltec_rrt_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>()
{
  return "wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage";
}

template<>
inline const char * name<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>()
{
  return "wheeltec_rrt_msg/action/ChangePosition_FeedbackMessage";
}

template<>
struct has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<wheeltec_rrt_msg::action::ChangePosition_Feedback>::value> {};

template<>
struct has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<wheeltec_rrt_msg::action::ChangePosition_Feedback>::value> {};

template<>
struct is_message<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<wheeltec_rrt_msg::action::ChangePosition>
  : std::true_type
{
};

template<>
struct is_action_goal<wheeltec_rrt_msg::action::ChangePosition_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<wheeltec_rrt_msg::action::ChangePosition_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<wheeltec_rrt_msg::action::ChangePosition_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__TRAITS_HPP_
