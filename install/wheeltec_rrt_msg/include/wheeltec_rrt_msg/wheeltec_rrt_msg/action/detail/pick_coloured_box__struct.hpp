// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheeltec_rrt_msg:action/PickColouredBox.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_HPP_
#define WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Goal __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Goal __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_Goal_
{
  using Type = PickColouredBox_Goal_<ContainerAllocator>;

  explicit PickColouredBox_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->box_colour = "";
    }
  }

  explicit PickColouredBox_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : box_colour(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->box_colour = "";
    }
  }

  // field types and members
  using _box_colour_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _box_colour_type box_colour;

  // setters for named parameter idiom
  Type & set__box_colour(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->box_colour = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Goal
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Goal
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_Goal_ & other) const
  {
    if (this->box_colour != other.box_colour) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_Goal_

// alias to use template instance with default allocator
using PickColouredBox_Goal =
  wheeltec_rrt_msg::action::PickColouredBox_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'result'
#include "std_msgs/msg/detail/empty__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Result __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Result __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_Result_
{
  using Type = PickColouredBox_Result_<ContainerAllocator>;

  explicit PickColouredBox_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    (void)_init;
  }

  explicit PickColouredBox_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _result_type =
    std_msgs::msg::Empty_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const std_msgs::msg::Empty_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Result
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Result
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_Result_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_Result_

// alias to use template instance with default allocator
using PickColouredBox_Result =
  wheeltec_rrt_msg::action::PickColouredBox_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg


#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Feedback __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_Feedback_
{
  using Type = PickColouredBox_Feedback_<ContainerAllocator>;

  explicit PickColouredBox_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit PickColouredBox_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Feedback
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_Feedback
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_Feedback_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_Feedback_

// alias to use template instance with default allocator
using PickColouredBox_Feedback =
  wheeltec_rrt_msg::action::PickColouredBox_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_SendGoal_Request_
{
  using Type = PickColouredBox_SendGoal_Request_<ContainerAllocator>;

  explicit PickColouredBox_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PickColouredBox_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const wheeltec_rrt_msg::action::PickColouredBox_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Request
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_SendGoal_Request_

// alias to use template instance with default allocator
using PickColouredBox_SendGoal_Request =
  wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_SendGoal_Response_
{
  using Type = PickColouredBox_SendGoal_Response_<ContainerAllocator>;

  explicit PickColouredBox_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PickColouredBox_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_SendGoal_Response
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_SendGoal_Response_

// alias to use template instance with default allocator
using PickColouredBox_SendGoal_Response =
  wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace wheeltec_rrt_msg
{

namespace action
{

struct PickColouredBox_SendGoal
{
  using Request = wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request;
  using Response = wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response;
};

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_GetResult_Request_
{
  using Type = PickColouredBox_GetResult_Request_<ContainerAllocator>;

  explicit PickColouredBox_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PickColouredBox_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Request
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_GetResult_Request_

// alias to use template instance with default allocator
using PickColouredBox_GetResult_Request =
  wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'result'
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_GetResult_Response_
{
  using Type = PickColouredBox_GetResult_Response_<ContainerAllocator>;

  explicit PickColouredBox_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PickColouredBox_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const wheeltec_rrt_msg::action::PickColouredBox_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_GetResult_Response
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_GetResult_Response_

// alias to use template instance with default allocator
using PickColouredBox_GetResult_Response =
  wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg

namespace wheeltec_rrt_msg
{

namespace action
{

struct PickColouredBox_GetResult
{
  using Request = wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request;
  using Response = wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response;
};

}  // namespace action

}  // namespace wheeltec_rrt_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage __declspec(deprecated)
#endif

namespace wheeltec_rrt_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickColouredBox_FeedbackMessage_
{
  using Type = PickColouredBox_FeedbackMessage_<ContainerAllocator>;

  explicit PickColouredBox_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PickColouredBox_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const wheeltec_rrt_msg::action::PickColouredBox_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheeltec_rrt_msg__action__PickColouredBox_FeedbackMessage
    std::shared_ptr<wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickColouredBox_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickColouredBox_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickColouredBox_FeedbackMessage_

// alias to use template instance with default allocator
using PickColouredBox_FeedbackMessage =
  wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace wheeltec_rrt_msg

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace wheeltec_rrt_msg
{

namespace action
{

struct PickColouredBox
{
  /// The goal message defined in the action definition.
  using Goal = wheeltec_rrt_msg::action::PickColouredBox_Goal;
  /// The result message defined in the action definition.
  using Result = wheeltec_rrt_msg::action::PickColouredBox_Result;
  /// The feedback message defined in the action definition.
  using Feedback = wheeltec_rrt_msg::action::PickColouredBox_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = wheeltec_rrt_msg::action::PickColouredBox_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = wheeltec_rrt_msg::action::PickColouredBox_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PickColouredBox PickColouredBox;

}  // namespace action

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__STRUCT_HPP_
