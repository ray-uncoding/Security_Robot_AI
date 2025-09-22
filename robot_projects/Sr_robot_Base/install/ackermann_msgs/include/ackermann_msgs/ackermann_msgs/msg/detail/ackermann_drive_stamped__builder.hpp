// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__BUILDER_HPP_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ackermann_msgs/msg/detail/ackermann_drive_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ackermann_msgs
{

namespace msg
{

namespace builder
{

class Init_AckermannDriveStamped_drive
{
public:
  explicit Init_AckermannDriveStamped_drive(::ackermann_msgs::msg::AckermannDriveStamped & msg)
  : msg_(msg)
  {}
  ::ackermann_msgs::msg::AckermannDriveStamped drive(::ackermann_msgs::msg::AckermannDriveStamped::_drive_type arg)
  {
    msg_.drive = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDriveStamped msg_;
};

class Init_AckermannDriveStamped_header
{
public:
  Init_AckermannDriveStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AckermannDriveStamped_drive header(::ackermann_msgs::msg::AckermannDriveStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AckermannDriveStamped_drive(msg_);
  }

private:
  ::ackermann_msgs::msg::AckermannDriveStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ackermann_msgs::msg::AckermannDriveStamped>()
{
  return ackermann_msgs::msg::builder::Init_AckermannDriveStamped_header();
}

}  // namespace ackermann_msgs

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE_STAMPED__BUILDER_HPP_
