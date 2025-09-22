// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheeltec_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_
#define WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheeltec_robot_msg/msg/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheeltec_robot_msg
{

namespace msg
{

namespace builder
{

class Init_Data_z
{
public:
  explicit Init_Data_z(::wheeltec_robot_msg::msg::Data & msg)
  : msg_(msg)
  {}
  ::wheeltec_robot_msg::msg::Data z(::wheeltec_robot_msg::msg::Data::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_robot_msg::msg::Data msg_;
};

class Init_Data_y
{
public:
  explicit Init_Data_y(::wheeltec_robot_msg::msg::Data & msg)
  : msg_(msg)
  {}
  Init_Data_z y(::wheeltec_robot_msg::msg::Data::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Data_z(msg_);
  }

private:
  ::wheeltec_robot_msg::msg::Data msg_;
};

class Init_Data_x
{
public:
  Init_Data_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Data_y x(::wheeltec_robot_msg::msg::Data::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Data_y(msg_);
  }

private:
  ::wheeltec_robot_msg::msg::Data msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_robot_msg::msg::Data>()
{
  return wheeltec_robot_msg::msg::builder::Init_Data_x();
}

}  // namespace wheeltec_robot_msg

#endif  // WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_
