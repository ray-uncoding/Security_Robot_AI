// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turn_on_wheeltec_robot__msg__Position __attribute__((deprecated))
#else
# define DEPRECATED__turn_on_wheeltec_robot__msg__Position __declspec(deprecated)
#endif

namespace turn_on_wheeltec_robot
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Position_
{
  using Type = Position_<ContainerAllocator>;

  explicit Position_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_x = 0.0f;
      this->angle_y = 0.0f;
      this->distance = 0.0f;
    }
  }

  explicit Position_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_x = 0.0f;
      this->angle_y = 0.0f;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _angle_x_type =
    float;
  _angle_x_type angle_x;
  using _angle_y_type =
    float;
  _angle_y_type angle_y;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__angle_x(
    const float & _arg)
  {
    this->angle_x = _arg;
    return *this;
  }
  Type & set__angle_y(
    const float & _arg)
  {
    this->angle_y = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> *;
  using ConstRawPtr =
    const turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turn_on_wheeltec_robot__msg__Position
    std::shared_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turn_on_wheeltec_robot__msg__Position
    std::shared_ptr<turn_on_wheeltec_robot::msg::Position_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Position_ & other) const
  {
    if (this->angle_x != other.angle_x) {
      return false;
    }
    if (this->angle_y != other.angle_y) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Position_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Position_

// alias to use template instance with default allocator
using Position =
  turn_on_wheeltec_robot::msg::Position_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turn_on_wheeltec_robot

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__STRUCT_HPP_
