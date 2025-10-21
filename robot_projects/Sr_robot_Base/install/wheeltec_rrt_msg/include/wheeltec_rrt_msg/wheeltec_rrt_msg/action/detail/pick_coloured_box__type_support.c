// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wheeltec_rrt_msg:action/PickColouredBox.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
#include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `box_colour`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_Goal__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_member_array[1] = {
  {
    "box_colour",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_Goal, box_colour),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_Goal",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_Goal),
  wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Goal)() {
  if (!wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_Goal__rosidl_typesupport_introspection_c__PickColouredBox_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `result`
#include "std_msgs/msg/empty.h"
// Member `result`
#include "std_msgs/msg/detail/empty__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_Result__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_Result, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_Result",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_Result),
  wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Result)() {
  wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Empty)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_Result__rosidl_typesupport_introspection_c__PickColouredBox_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_Feedback__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_Feedback",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_Feedback),
  wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Feedback)() {
  if (!wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_Feedback__rosidl_typesupport_introspection_c__PickColouredBox_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "wheeltec_rrt_msg/action/pick_coloured_box.h"
// Member `goal`
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request),
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Request)() {
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Goal)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response),
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Response)() {
  wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_members = {
  "wheeltec_rrt_msg__action",  // service namespace
  "PickColouredBox_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal)() {
  if (!wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_SendGoal_Response)()->data;
  }

  return &wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request),
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Request)() {
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "wheeltec_rrt_msg/action/pick_coloured_box.h"
// Member `result`
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response),
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Response)() {
  wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Result)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_members = {
  "wheeltec_rrt_msg__action",  // service namespace
  "PickColouredBox_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult)() {
  if (!wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_GetResult_Response)()->data;
  }

  return &wheeltec_rrt_msg__action__detail__pick_coloured_box__rosidl_typesupport_introspection_c__PickColouredBox_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"
// already included above
// #include "wheeltec_rrt_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__functions.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "wheeltec_rrt_msg/action/pick_coloured_box.h"
// Member `feedback`
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__init(message_memory);
}

void wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_fini_function(void * message_memory)
{
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_members = {
  "wheeltec_rrt_msg__action",  // message namespace
  "PickColouredBox_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage),
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_member_array,  // message members
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_type_support_handle = {
  0,
  &wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_rrt_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_FeedbackMessage)() {
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_rrt_msg, action, PickColouredBox_Feedback)();
  if (!wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage__rosidl_typesupport_introspection_c__PickColouredBox_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
