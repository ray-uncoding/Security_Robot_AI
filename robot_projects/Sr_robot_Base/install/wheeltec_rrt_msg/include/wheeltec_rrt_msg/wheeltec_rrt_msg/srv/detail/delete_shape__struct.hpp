// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheeltec_rrt_msg:srv/DeleteShape.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_HPP_
#define WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Request __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Request __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DeleteShape_Request_
{
  using Type = DeleteShape_Request_<ContainerAllocator>;

  explicit DeleteShape_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shape_type = "";
    }
  }

  explicit DeleteShape_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : shape_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shape_type = "";
    }
  }

  // field types and members
  using _shape_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _shape_type_type shape_type;

  // setters for named parameter idiom
  Type & set__shape_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->shape_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Request
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Request
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DeleteShape_Request_ & other) const
  {
    if (this->shape_type != other.shape_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const DeleteShape_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DeleteShape_Request_

// alias to use template instance with default allocator
using DeleteShape_Request =
  wheeltec_rrt_msg::srv::DeleteShape_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace wheeltec_rrt_msg


#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Response __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Response __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DeleteShape_Response_
{
  using Type = DeleteShape_Response_<ContainerAllocator>;

  explicit DeleteShape_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit DeleteShape_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Response
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__srv__DeleteShape_Response
    std::shared_ptr<wheeltec_rrt_msg::srv::DeleteShape_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DeleteShape_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const DeleteShape_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DeleteShape_Response_

// alias to use template instance with default allocator
using DeleteShape_Response =
  wheeltec_rrt_msg::srv::DeleteShape_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace wheeltec_rrt_msg

namespace wheeltec_rrt_msg
{

namespace srv
{

struct DeleteShape
{
  using Request = wheeltec_rrt_msg::srv::DeleteShape_Request;
  using Response = wheeltec_rrt_msg::srv::DeleteShape_Response;
};

}  // namespace srv

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__SRV__DETAIL__DELETE_SHAPE__STRUCT_HPP_
