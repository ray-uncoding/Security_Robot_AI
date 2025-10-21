// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheeltec_rrt_msg:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
#define WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheeltec_rrt_msg/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheeltec_rrt_msg
{

namespace msg
{

namespace builder
{

class Init_PointArray_points
{
public:
  Init_PointArray_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::msg::PointArray points(::wheeltec_rrt_msg::msg::PointArray::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::msg::PointArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::msg::PointArray>()
{
  return wheeltec_rrt_msg::msg::builder::Init_PointArray_points();
}

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
