// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheeltec_rrt_msg:action/PickColouredBox.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__BUILDER_HPP_
#define WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheeltec_rrt_msg/action/detail/pick_coloured_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_Goal_box_colour
{
public:
  Init_PickColouredBox_Goal_box_colour()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_Goal box_colour(::wheeltec_rrt_msg::action::PickColouredBox_Goal::_box_colour_type arg)
  {
    msg_.box_colour = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_Goal>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_Goal_box_colour();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_Result_result
{
public:
  Init_PickColouredBox_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_Result result(::wheeltec_rrt_msg::action::PickColouredBox_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_Result>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_Result_result();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_Feedback>()
{
  return ::wheeltec_rrt_msg::action::PickColouredBox_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_SendGoal_Request_goal
{
public:
  explicit Init_PickColouredBox_SendGoal_Request_goal(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request goal(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request msg_;
};

class Init_PickColouredBox_SendGoal_Request_goal_id
{
public:
  Init_PickColouredBox_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickColouredBox_SendGoal_Request_goal goal_id(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PickColouredBox_SendGoal_Request_goal(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Request>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_SendGoal_Request_goal_id();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_SendGoal_Response_stamp
{
public:
  explicit Init_PickColouredBox_SendGoal_Response_stamp(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response stamp(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response msg_;
};

class Init_PickColouredBox_SendGoal_Response_accepted
{
public:
  Init_PickColouredBox_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickColouredBox_SendGoal_Response_stamp accepted(::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PickColouredBox_SendGoal_Response_stamp(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_SendGoal_Response>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_SendGoal_Response_accepted();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_GetResult_Request_goal_id
{
public:
  Init_PickColouredBox_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request goal_id(::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Request>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_GetResult_Request_goal_id();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_GetResult_Response_result
{
public:
  explicit Init_PickColouredBox_GetResult_Response_result(::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response result(::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response msg_;
};

class Init_PickColouredBox_GetResult_Response_status
{
public:
  Init_PickColouredBox_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickColouredBox_GetResult_Response_result status(::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PickColouredBox_GetResult_Response_result(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_GetResult_Response>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_GetResult_Response_status();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_PickColouredBox_FeedbackMessage_feedback
{
public:
  explicit Init_PickColouredBox_FeedbackMessage_feedback(::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage feedback(::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage msg_;
};

class Init_PickColouredBox_FeedbackMessage_goal_id
{
public:
  Init_PickColouredBox_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickColouredBox_FeedbackMessage_feedback goal_id(::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PickColouredBox_FeedbackMessage_feedback(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::PickColouredBox_FeedbackMessage>()
{
  return wheeltec_rrt_msg::action::builder::Init_PickColouredBox_FeedbackMessage_goal_id();
}

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__ACTION__DETAIL__PICK_COLOURED_BOX__BUILDER_HPP_
