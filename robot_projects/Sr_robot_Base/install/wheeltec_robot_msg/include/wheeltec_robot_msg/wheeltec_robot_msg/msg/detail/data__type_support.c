// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wheeltec_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wheeltec_robot_msg/msg/detail/data__rosidl_typesupport_introspection_c.h"
#include "wheeltec_robot_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wheeltec_robot_msg/msg/detail/data__functions.h"
#include "wheeltec_robot_msg/msg/detail/data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wheeltec_robot_msg__msg__Data__init(message_memory);
}

void wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_fini_function(void * message_memory)
{
  wheeltec_robot_msg__msg__Data__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_robot_msg__msg__Data, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_robot_msg__msg__Data, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wheeltec_robot_msg__msg__Data, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_members = {
  "wheeltec_robot_msg__msg",  // message namespace
  "Data",  // message name
  3,  // number of fields
  sizeof(wheeltec_robot_msg__msg__Data),
  wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_member_array,  // message members
  wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_init_function,  // function to initialize message memory (memory has to be allocated)
  wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_type_support_handle = {
  0,
  &wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wheeltec_robot_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wheeltec_robot_msg, msg, Data)() {
  if (!wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_type_support_handle.typesupport_identifier) {
    wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wheeltec_robot_msg__msg__Data__rosidl_typesupport_introspection_c__Data_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
