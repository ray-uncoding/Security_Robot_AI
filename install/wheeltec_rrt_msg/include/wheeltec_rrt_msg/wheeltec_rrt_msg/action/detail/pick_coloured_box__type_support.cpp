// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wheeltec_rrt_msg:action/PickColouredBox.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_Goal_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_Goal(_init);
}

void PickColouredBox_Goal_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_Goal *>(message_memory);
  typed_message->~PickColouredBox_Goal();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_Goal_message_member_array[1] = {
  {
    "box_colour",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_Goal, box_colour),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_Goal_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_Goal",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_Goal),
  PickColouredBox_Goal_message_member_array,  // message members
  PickColouredBox_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_Goal_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_Goal_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_Goal_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Goal>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Goal_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_Goal)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_Result_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_Result(_init);
}

void PickColouredBox_Result_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_Result *>(message_memory);
  typed_message->~PickColouredBox_Result();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_Result_message_member_array[1] = {
  {
    "result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Empty>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_Result, result),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_Result_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_Result",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_Result),
  PickColouredBox_Result_message_member_array,  // message members
  PickColouredBox_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_Result_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_Result_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_Result_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Result>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Result_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_Result)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_Feedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_Feedback(_init);
}

void PickColouredBox_Feedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_Feedback *>(message_memory);
  typed_message->~PickColouredBox_Feedback();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_Feedback_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_Feedback",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_Feedback),
  PickColouredBox_Feedback_message_member_array,  // message members
  PickColouredBox_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_Feedback_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Feedback>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Feedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_Feedback)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_SendGoal_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request(_init);
}

void PickColouredBox_SendGoal_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request *>(message_memory);
  typed_message->~PickColouredBox_SendGoal_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Goal>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request, goal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_SendGoal_Request_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request),
  PickColouredBox_SendGoal_Request_message_member_array,  // message members
  PickColouredBox_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_SendGoal_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Request)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_SendGoal_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response(_init);
}

void PickColouredBox_SendGoal_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response *>(message_memory);
  typed_message->~PickColouredBox_SendGoal_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response, accepted),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_SendGoal_Response_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response),
  PickColouredBox_SendGoal_Response_message_member_array,  // message members
  PickColouredBox_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_SendGoal_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Response)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers PickColouredBox_SendGoal_service_members = {
  "wheeltec_rrt_msg::action",  // service namespace
  "PickColouredBox_SendGoal",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_SendGoal>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t PickColouredBox_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_SendGoal>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_SendGoal_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_SendGoal)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_GetResult_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request(_init);
}

void PickColouredBox_GetResult_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request *>(message_memory);
  typed_message->~PickColouredBox_GetResult_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_GetResult_Request_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request),
  PickColouredBox_GetResult_Request_message_member_array,  // message members
  PickColouredBox_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_GetResult_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Request)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_GetResult_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response(_init);
}

void PickColouredBox_GetResult_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response *>(message_memory);
  typed_message->~PickColouredBox_GetResult_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Result>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response, result),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_GetResult_Response_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response),
  PickColouredBox_GetResult_Response_message_member_array,  // message members
  PickColouredBox_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_GetResult_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Response)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers PickColouredBox_GetResult_service_members = {
  "wheeltec_rrt_msg::action",  // service namespace
  "PickColouredBox_GetResult",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_GetResult>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t PickColouredBox_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_GetResult_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_GetResult>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_GetResult_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_GetResult)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void PickColouredBox_FeedbackMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage(_init);
}

void PickColouredBox_FeedbackMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage *>(message_memory);
  typed_message->~PickColouredBox_FeedbackMessage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickColouredBox_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feedback",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_Feedback>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage, feedback),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickColouredBox_FeedbackMessage_message_members = {
  "wheeltec_rrt_msg::action",  // message namespace
  "PickColouredBox_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage),
  PickColouredBox_FeedbackMessage_message_member_array,  // message members
  PickColouredBox_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  PickColouredBox_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickColouredBox_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickColouredBox_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_FeedbackMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, PickColouredBox_FeedbackMessage)() {
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_introspection_cpp::PickColouredBox_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
