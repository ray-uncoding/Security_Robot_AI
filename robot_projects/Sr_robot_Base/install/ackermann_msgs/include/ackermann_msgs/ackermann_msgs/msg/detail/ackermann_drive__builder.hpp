// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__BUILDER_HPP_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ackermann_msgs/msg/detail/ackermann_drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ackermann_msgs
{

namespace msg
{

namespace builder
{

class Init_AckermannDrive_jerk
{
public:
  explicit Init_AckermannDrive_jerk(::ackermann_msgs::msg::AckermannDrive & msg)
  : msg_(msg)
  {}
  ::ackermann_msgs::msg::AckermannDrive jerk(::ackermann_msgs::msg::AckermannDrive::_jerk_type arg)
  {
    msg_.jerk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDrive msg_;
};

class Init_AckermannDrive_acceleration
{
public:
  explicit Init_AckermannDrive_acceleration(::ackermann_msgs::msg::AckermannDrive & msg)
  : msg_(msg)
  {}
  Init_AckermannDrive_jerk acceleration(::ackermann_msgs::msg::AckermannDrive::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_AckermannDrive_jerk(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDrive msg_;
};

class Init_AckermannDrive_speed
{
public:
  explicit Init_AckermannDrive_speed(::ackermann_msgs::msg::AckermannDrive & msg)
  : msg_(msg)
  {}
  Init_AckermannDrive_acceleration speed(::ackermann_msgs::msg::AckermannDrive::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_AckermannDrive_acceleration(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDrive msg_;
};

class Init_AckermannDrive_steering_angle_velocity
{
public:
  explicit Init_AckermannDrive_steering_angle_velocity(::ackermann_msgs::msg::AckermannDrive & msg)
  : msg_(msg)
  {}
  Init_AckermannDrive_speed steering_angle_velocity(::ackermann_msgs::msg::AckermannDrive::_steering_angle_velocity_type arg)
  {
    msg_.steering_angle_velocity = std::move(arg);
    return Init_AckermannDrive_speed(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDrive msg_;
};

class Init_AckermannDrive_steering_angle
{
public:
  Init_AckermannDrive_steering_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AckermannDrive_steering_angle_velocity steering_angle(::ackermann_msgs::msg::AckermannDrive::_steering_angle_type arg)
  {
    msg_.steering_angle = std::move(arg);
    return Init_AckermannDrive_steering_angle_velocity(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDrive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ackermann_msgs::msg::AckermannDrive>()
{
  return ackermann_msgs::msg::builder::Init_AckermannDrive_steering_angle();
}

}  // namespace ackermann_msgs

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__BUILDER_HPP_
