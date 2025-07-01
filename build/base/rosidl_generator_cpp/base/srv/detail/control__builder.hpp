// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef BASE__SRV__DETAIL__CONTROL__BUILDER_HPP_
#define BASE__SRV__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base
{

namespace srv
{

namespace builder
{

class Init_Control_Request_params
{
public:
  explicit Init_Control_Request_params(::base::srv::Control_Request & msg)
  : msg_(msg)
  {}
  ::base::srv::Control_Request params(::base::srv::Control_Request::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base::srv::Control_Request msg_;
};

class Init_Control_Request_flag
{
public:
  explicit Init_Control_Request_flag(::base::srv::Control_Request & msg)
  : msg_(msg)
  {}
  Init_Control_Request_params flag(::base::srv::Control_Request::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return Init_Control_Request_params(msg_);
  }

private:
  ::base::srv::Control_Request msg_;
};

class Init_Control_Request_func
{
public:
  explicit Init_Control_Request_func(::base::srv::Control_Request & msg)
  : msg_(msg)
  {}
  Init_Control_Request_flag func(::base::srv::Control_Request::_func_type arg)
  {
    msg_.func = std::move(arg);
    return Init_Control_Request_flag(msg_);
  }

private:
  ::base::srv::Control_Request msg_;
};

class Init_Control_Request_topic
{
public:
  Init_Control_Request_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Request_func topic(::base::srv::Control_Request::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return Init_Control_Request_func(msg_);
  }

private:
  ::base::srv::Control_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::base::srv::Control_Request>()
{
  return base::srv::builder::Init_Control_Request_topic();
}

}  // namespace base


namespace base
{

namespace srv
{

namespace builder
{

class Init_Control_Response_value
{
public:
  explicit Init_Control_Response_value(::base::srv::Control_Response & msg)
  : msg_(msg)
  {}
  ::base::srv::Control_Response value(::base::srv::Control_Response::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base::srv::Control_Response msg_;
};

class Init_Control_Response_code
{
public:
  Init_Control_Response_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Response_value code(::base::srv::Control_Response::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_Control_Response_value(msg_);
  }

private:
  ::base::srv::Control_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::base::srv::Control_Response>()
{
  return base::srv::builder::Init_Control_Response_code();
}

}  // namespace base

#endif  // BASE__SRV__DETAIL__CONTROL__BUILDER_HPP_
