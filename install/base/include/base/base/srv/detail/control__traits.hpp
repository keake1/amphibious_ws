// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef BASE__SRV__DETAIL__CONTROL__TRAITS_HPP_
#define BASE__SRV__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace base
{

namespace srv
{

inline void to_flow_style_yaml(
  const Control_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic
  {
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << ", ";
  }

  // member: func
  {
    out << "func: ";
    rosidl_generator_traits::value_to_yaml(msg.func, out);
    out << ", ";
  }

  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << ", ";
  }

  // member: params
  {
    out << "params: ";
    rosidl_generator_traits::value_to_yaml(msg.params, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << "\n";
  }

  // member: func
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "func: ";
    rosidl_generator_traits::value_to_yaml(msg.func, out);
    out << "\n";
  }

  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }

  // member: params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "params: ";
    rosidl_generator_traits::value_to_yaml(msg.params, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace base

namespace rosidl_generator_traits
{

[[deprecated("use base::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const base::srv::Control_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  base::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base::srv::to_yaml() instead")]]
inline std::string to_yaml(const base::srv::Control_Request & msg)
{
  return base::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base::srv::Control_Request>()
{
  return "base::srv::Control_Request";
}

template<>
inline const char * name<base::srv::Control_Request>()
{
  return "base/srv/Control_Request";
}

template<>
struct has_fixed_size<base::srv::Control_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base::srv::Control_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base::srv::Control_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace base
{

namespace srv
{

inline void to_flow_style_yaml(
  const Control_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: code
  {
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace base

namespace rosidl_generator_traits
{

[[deprecated("use base::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const base::srv::Control_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  base::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base::srv::to_yaml() instead")]]
inline std::string to_yaml(const base::srv::Control_Response & msg)
{
  return base::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base::srv::Control_Response>()
{
  return "base::srv::Control_Response";
}

template<>
inline const char * name<base::srv::Control_Response>()
{
  return "base/srv/Control_Response";
}

template<>
struct has_fixed_size<base::srv::Control_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base::srv::Control_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base::srv::Control_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<base::srv::Control>()
{
  return "base::srv::Control";
}

template<>
inline const char * name<base::srv::Control>()
{
  return "base/srv/Control";
}

template<>
struct has_fixed_size<base::srv::Control>
  : std::integral_constant<
    bool,
    has_fixed_size<base::srv::Control_Request>::value &&
    has_fixed_size<base::srv::Control_Response>::value
  >
{
};

template<>
struct has_bounded_size<base::srv::Control>
  : std::integral_constant<
    bool,
    has_bounded_size<base::srv::Control_Request>::value &&
    has_bounded_size<base::srv::Control_Response>::value
  >
{
};

template<>
struct is_service<base::srv::Control>
  : std::true_type
{
};

template<>
struct is_service_request<base::srv::Control_Request>
  : std::true_type
{
};

template<>
struct is_service_response<base::srv::Control_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BASE__SRV__DETAIL__CONTROL__TRAITS_HPP_
