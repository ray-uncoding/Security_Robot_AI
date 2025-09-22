// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from wheeltec_rrt_msg:action/ChangePosition.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_Goal_type_support_ids_t;

static const _ChangePosition_Goal_type_support_ids_t _ChangePosition_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_Goal_type_support_symbol_names_t _ChangePosition_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_Goal)),
  }
};

typedef struct _ChangePosition_Goal_type_support_data_t
{
  void * data[2];
} _ChangePosition_Goal_type_support_data_t;

static _ChangePosition_Goal_type_support_data_t _ChangePosition_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_Goal_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Goal>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_Goal)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_Result_type_support_ids_t;

static const _ChangePosition_Result_type_support_ids_t _ChangePosition_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_Result_type_support_symbol_names_t _ChangePosition_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_Result)),
  }
};

typedef struct _ChangePosition_Result_type_support_data_t
{
  void * data[2];
} _ChangePosition_Result_type_support_data_t;

static _ChangePosition_Result_type_support_data_t _ChangePosition_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_Result_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_Result_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_Result_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Result>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_Result)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_Feedback_type_support_ids_t;

static const _ChangePosition_Feedback_type_support_ids_t _ChangePosition_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_Feedback_type_support_symbol_names_t _ChangePosition_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_Feedback)),
  }
};

typedef struct _ChangePosition_Feedback_type_support_data_t
{
  void * data[2];
} _ChangePosition_Feedback_type_support_data_t;

static _ChangePosition_Feedback_type_support_data_t _ChangePosition_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_Feedback_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Feedback>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_Feedback)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_SendGoal_Request_type_support_ids_t;

static const _ChangePosition_SendGoal_Request_type_support_ids_t _ChangePosition_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_SendGoal_Request_type_support_symbol_names_t _ChangePosition_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Request)),
  }
};

typedef struct _ChangePosition_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _ChangePosition_SendGoal_Request_type_support_data_t;

static _ChangePosition_SendGoal_Request_type_support_data_t _ChangePosition_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_SendGoal_Request_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Request)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_SendGoal_Response_type_support_ids_t;

static const _ChangePosition_SendGoal_Response_type_support_ids_t _ChangePosition_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_SendGoal_Response_type_support_symbol_names_t _ChangePosition_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Response)),
  }
};

typedef struct _ChangePosition_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _ChangePosition_SendGoal_Response_type_support_data_t;

static _ChangePosition_SendGoal_Response_type_support_data_t _ChangePosition_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_SendGoal_Response_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal_Response)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_SendGoal_type_support_ids_t;

static const _ChangePosition_SendGoal_type_support_ids_t _ChangePosition_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_SendGoal_type_support_symbol_names_t _ChangePosition_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal)),
  }
};

typedef struct _ChangePosition_SendGoal_type_support_data_t
{
  void * data[2];
} _ChangePosition_SendGoal_type_support_data_t;

static _ChangePosition_SendGoal_type_support_data_t _ChangePosition_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_SendGoal_service_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ChangePosition_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_GetResult_Request_type_support_ids_t;

static const _ChangePosition_GetResult_Request_type_support_ids_t _ChangePosition_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_GetResult_Request_type_support_symbol_names_t _ChangePosition_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Request)),
  }
};

typedef struct _ChangePosition_GetResult_Request_type_support_data_t
{
  void * data[2];
} _ChangePosition_GetResult_Request_type_support_data_t;

static _ChangePosition_GetResult_Request_type_support_data_t _ChangePosition_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_GetResult_Request_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Request)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_GetResult_Response_type_support_ids_t;

static const _ChangePosition_GetResult_Response_type_support_ids_t _ChangePosition_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_GetResult_Response_type_support_symbol_names_t _ChangePosition_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Response)),
  }
};

typedef struct _ChangePosition_GetResult_Response_type_support_data_t
{
  void * data[2];
} _ChangePosition_GetResult_Response_type_support_data_t;

static _ChangePosition_GetResult_Response_type_support_data_t _ChangePosition_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_GetResult_Response_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult_Response)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_GetResult_type_support_ids_t;

static const _ChangePosition_GetResult_type_support_ids_t _ChangePosition_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_GetResult_type_support_symbol_names_t _ChangePosition_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult)),
  }
};

typedef struct _ChangePosition_GetResult_type_support_data_t
{
  void * data[2];
} _ChangePosition_GetResult_type_support_data_t;

static _ChangePosition_GetResult_type_support_data_t _ChangePosition_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_GetResult_service_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ChangePosition_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ChangePosition_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChangePosition_FeedbackMessage_type_support_ids_t;

static const _ChangePosition_FeedbackMessage_type_support_ids_t _ChangePosition_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChangePosition_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChangePosition_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChangePosition_FeedbackMessage_type_support_symbol_names_t _ChangePosition_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wheeltec_rrt_msg, action, ChangePosition_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wheeltec_rrt_msg, action, ChangePosition_FeedbackMessage)),
  }
};

typedef struct _ChangePosition_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _ChangePosition_FeedbackMessage_type_support_data_t;

static _ChangePosition_FeedbackMessage_type_support_data_t _ChangePosition_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChangePosition_FeedbackMessage_message_typesupport_map = {
  2,
  "wheeltec_rrt_msg",
  &_ChangePosition_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_ChangePosition_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_ChangePosition_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChangePosition_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChangePosition_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>()
{
  return &::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition_FeedbackMessage)() {
  return get_message_type_support_handle<wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t ChangePosition_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<wheeltec_rrt_msg::action::ChangePosition>()
{
  using ::wheeltec_rrt_msg::action::rosidl_typesupport_cpp::ChangePosition_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  ChangePosition_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::wheeltec_rrt_msg::action::ChangePosition::Impl::SendGoalService>();
  ChangePosition_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::wheeltec_rrt_msg::action::ChangePosition::Impl::GetResultService>();
  ChangePosition_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::wheeltec_rrt_msg::action::ChangePosition::Impl::CancelGoalService>();
  ChangePosition_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::wheeltec_rrt_msg::action::ChangePosition::Impl::FeedbackMessage>();
  ChangePosition_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::wheeltec_rrt_msg::action::ChangePosition::Impl::GoalStatusMessage>();
  return &ChangePosition_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, wheeltec_rrt_msg, action, ChangePosition)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<wheeltec_rrt_msg::action::ChangePosition>();
}

#ifdef __cplusplus
}
#endif
