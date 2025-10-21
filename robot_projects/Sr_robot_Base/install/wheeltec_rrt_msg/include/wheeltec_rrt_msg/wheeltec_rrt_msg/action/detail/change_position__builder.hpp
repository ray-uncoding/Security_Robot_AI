// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheeltec_rrt_msg:action/ChangePosition.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__BUILDER_HPP_
#define WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheeltec_rrt_msg/action/detail/change_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_Goal_desired_y
{
public:
  explicit Init_ChangePosition_Goal_desired_y(::wheeltec_rrt_msg::action::ChangePosition_Goal & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_Goal desired_y(::wheeltec_rrt_msg::action::ChangePosition_Goal::_desired_y_type arg)
  {
    msg_.desired_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_Goal msg_;
};

class Init_ChangePosition_Goal_desired_x
{
public:
  Init_ChangePosition_Goal_desired_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_Goal_desired_y desired_x(::wheeltec_rrt_msg::action::ChangePosition_Goal::_desired_x_type arg)
  {
    msg_.desired_x = std::move(arg);
    return Init_ChangePosition_Goal_desired_y(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_Goal>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_Goal_desired_x();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_Result_is_complete
{
public:
  Init_ChangePosition_Result_is_complete()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_Result is_complete(::wheeltec_rrt_msg::action::ChangePosition_Result::_is_complete_type arg)
  {
    msg_.is_complete = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_Result>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_Result_is_complete();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_Feedback_current_y
{
public:
  explicit Init_ChangePosition_Feedback_current_y(::wheeltec_rrt_msg::action::ChangePosition_Feedback & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_Feedback current_y(::wheeltec_rrt_msg::action::ChangePosition_Feedback::_current_y_type arg)
  {
    msg_.current_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_Feedback msg_;
};

class Init_ChangePosition_Feedback_current_x
{
public:
  Init_ChangePosition_Feedback_current_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_Feedback_current_y current_x(::wheeltec_rrt_msg::action::ChangePosition_Feedback::_current_x_type arg)
  {
    msg_.current_x = std::move(arg);
    return Init_ChangePosition_Feedback_current_y(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_Feedback>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_Feedback_current_x();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_SendGoal_Request_goal
{
public:
  explicit Init_ChangePosition_SendGoal_Request_goal(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request goal(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request msg_;
};

class Init_ChangePosition_SendGoal_Request_goal_id
{
public:
  Init_ChangePosition_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_SendGoal_Request_goal goal_id(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ChangePosition_SendGoal_Request_goal(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Request>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_SendGoal_Request_goal_id();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_SendGoal_Response_stamp
{
public:
  explicit Init_ChangePosition_SendGoal_Response_stamp(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response stamp(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response msg_;
};

class Init_ChangePosition_SendGoal_Response_accepted
{
public:
  Init_ChangePosition_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_SendGoal_Response_stamp accepted(::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ChangePosition_SendGoal_Response_stamp(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_SendGoal_Response>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_SendGoal_Response_accepted();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_GetResult_Request_goal_id
{
public:
  Init_ChangePosition_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_GetResult_Request goal_id(::wheeltec_rrt_msg::action::ChangePosition_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_GetResult_Request>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_GetResult_Request_goal_id();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_GetResult_Response_result
{
public:
  explicit Init_ChangePosition_GetResult_Response_result(::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response result(::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response msg_;
};

class Init_ChangePosition_GetResult_Response_status
{
public:
  Init_ChangePosition_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_GetResult_Response_result status(::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ChangePosition_GetResult_Response_result(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_GetResult_Response>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_GetResult_Response_status();
}

}  // namespace wheeltec_rrt_msg


namespace wheeltec_rrt_msg
{

namespace action
{

namespace builder
{

class Init_ChangePosition_FeedbackMessage_feedback
{
public:
  explicit Init_ChangePosition_FeedbackMessage_feedback(::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage feedback(::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage msg_;
};

class Init_ChangePosition_FeedbackMessage_goal_id
{
public:
  Init_ChangePosition_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChangePosition_FeedbackMessage_feedback goal_id(::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ChangePosition_FeedbackMessage_feedback(msg_);
  }

private:
  ::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheeltec_rrt_msg::action::ChangePosition_FeedbackMessage>()
{
  return wheeltec_rrt_msg::action::builder::Init_ChangePosition_FeedbackMessage_goal_id();
}

}  // namespace wheeltec_rrt_msg

#endif  // WHEELTEC_RRT_MSG__ACTION__DETAIL__CHANGE_POSITION__BUILDER_HPP_
