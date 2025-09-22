// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__BUILDER_HPP_
#define WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheeltec_rrt_msg/srv/detail/delete_shape__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheeltec_rrt_msg
{

namespace srv
{

namespace builder
{

class Init_DeleteShape_Request_shape_type
{
public:
  Init_DeleteShape_Request_shape_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::srv::DeleteShape_Request shape_type(::wheeltec_rrt_msg::srv::DeleteShape_Request::_shape_type_type arg)
  {
    msg_.shape_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::srv::DeleteShape_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::srv::DeleteShape_Request>()
{
  return wheeltec_rrt_msg::srv::builder::Init_DeleteShape_Request_shape_type();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace srv
{

namespace builder
{

class Init_DeleteShape_Response_result
{
public:
  Init_DeleteShape_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::srv::DeleteShape_Response result(::wheeltec_rrt_msg::srv::DeleteShape_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::srv::DeleteShape_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::srv::DeleteShape_Response>()
{
  return wheeltec_rrt_msg::srv::builder::Init_DeleteShape_Response_result();
}

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__BUILDER_HPP_
