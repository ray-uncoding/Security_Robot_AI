// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__BUILDER_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turn_on_wheeltec_robot/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turn_on_wheeltec_robot
{

namespace msg
{

namespace builder
{

class Init_Position_distance
{
public:
  explicit Init_Position_distance(::turn_on_wheeltec_robot::msg::Position & msg)
  : msg_(msg)
  {}
  ::turn_on_wheeltec_robot::msg::Position distance(::turn_on_wheeltec_robot::msg::Position::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turn_on_wheeltec_robot::msg::Position msg_;
};

class Init_Position_angle_y
{
public:
  explicit Init_Position_angle_y(::turn_on_wheeltec_robot::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_distance angle_y(::turn_on_wheeltec_robot::msg::Position::_angle_y_type arg)
  {
    msg_.angle_y = std::move(arg);
    return Init_Position_distance(msg_);
  }

private:
  ::turn_on_wheeltec_robot::msg::Position msg_;
};

class Init_Position_angle_x
{
public:
  Init_Position_angle_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_angle_y angle_x(::turn_on_wheeltec_robot::msg::Position::_angle_x_type arg)
  {
    msg_.angle_x = std::move(arg);
    return Init_Position_angle_y(msg_);
  }

private:
  ::turn_on_wheeltec_robot::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turn_on_wheeltec_robot::msg::Position>()
{
  return turn_on_wheeltec_robot::msg::builder::Init_Position_angle_x();
}

}  // namespace turn_on_wheeltec_robot

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__BUILDER_HPP_
